#include <Pipeline.hpp>

std::atomic<bool> Pipeline::running_{true};
std::condition_variable Pipeline::globalCV_;
boost::lockfree::spsc_queue<DataFrame, boost::lockfree::capacity<128>> Pipeline::decodedPoint_buffer_;

// -----------------------------------------------------------------------------

Pipeline::Pipeline(const std::string& json_path) : lidarCallback(json_path) {}
Pipeline::Pipeline(const nlohmann::json& json_data) : lidarCallback(json_data) {}
// Pipeline::~Pipeline() {}

// -----------------------------------------------------------------------------

void Pipeline::signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        running_.store(false, std::memory_order_release);
        globalCV_.notify_all();

        constexpr const char* message = "[signalHandler] Shutting down...\n";
        constexpr size_t messageLen = sizeof(message) - 1;
        ssize_t result = write(STDOUT_FILENO, message, messageLen);
    }
}

// -----------------------------------------------------------------------------

void Pipeline::setThreadAffinity(const std::vector<int>& coreIDs) {
    if (coreIDs.empty()) {return;}
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    const unsigned int maxCores = std::thread::hardware_concurrency();
    uint32_t validCores = 0;

    for (int coreID : coreIDs) {
        if (coreID >= 0 && static_cast<unsigned>(coreID) < maxCores) {
            CPU_SET(coreID, &cpuset);
            validCores |= (1 << coreID);
        }
    }
    if (!validCores) {
            return;
        }

    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0) {
        running_.store(false); // Optionally terminate
    }

}

// -----------------------------------------------------------------------------

void Pipeline::runOusterLidarListener(boost::asio::io_context& ioContext, const std::string& host, uint16_t port, uint32_t bufferSize,  const std::vector<int>& allowedCores) {
    
    setThreadAffinity(allowedCores);
    if (host.empty() || port == 0) {return;}
    
    try {
        UdpSocket listener(ioContext, host, port, [&](const std::vector<uint8_t>& data) {
            lidarCallback.decode_packet_single_return(data, decodedPoint);
            if(decodedPoint.frame_id != frame_id_){
                // std::cerr << "[Pipeline] frame_id: " << decodedPoint.frame_id << " numberpoints: " << decodedPoint.numberpoints << std::endl;
                frame_id_ = decodedPoint.frame_id;
                if (!decodedPoint_buffer_.push(decodedPoint)) {
                    std::cerr << "[Pipeline] Buffer push failed" << std::endl;
                }
            }
        },bufferSize);

        while (running_.load(std::memory_order_acquire)) {
            try {
                ioContext.run();
                break;
            } catch (const std::exception& e) {
                ioContext.restart();
            }
        }
    }
    catch(const std::exception& e){
        std::cerr << "[runOusterLidarListener] Exception: " << e.what() << std::endl;
    }
    ioContext.stop();
}

// -----------------------------------------------------------------------------

void Pipeline::runVisualizer(const std::vector<int>& allowedCores) {

    setThreadAffinity(allowedCores);

    try {
        if (!vis.CreateVisualizerWindow("3D Voxel Visualization", 1280, 720)) {
            return;
        }
        vis.GetRenderOption().background_color_ = Eigen::Vector3d(0, 0, 0); // background color
        vis.GetRenderOption().point_size_ = 1.0; // Set point size for visibility

        // Initialize point cloud with default data
        if (!point_cloud_ptr_) {
            point_cloud_ptr_ = std::make_shared<open3d::geometry::PointCloud>();
            point_cloud_ptr_->points_.push_back(Eigen::Vector3d(0, 0, 0));
            point_cloud_ptr_->colors_.push_back(Eigen::Vector3d(1, 0, 0)); // Red point at origin
        }

        // Add point cloud
        vis.AddGeometry(point_cloud_ptr_);

        // Add coordinate frame for reference
        auto coord_frame = open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.8);
        vis.AddGeometry(coord_frame);

        // Set initial camera parameters
        auto& view = vis.GetViewControl();
        view.SetLookat({0.0, 0.0, 0.0}); // Center on origin
        view.SetFront({0, 0, -1});
        view.SetUp({0, 1, 0});
        view.SetZoom(6.0); // Wide zoom

        // Register animation callback
        vis.RegisterAnimationCallback([&](open3d::visualization::Visualizer* vis_ptr) {
            try {
                return updateVisualizer(vis_ptr);
            } catch (const std::exception& e) {
                return running_.load(std::memory_order_acquire);
            }
        });
        vis.Run();
        vis.DestroyVisualizerWindow();
    } catch (const std::exception& e) {
        vis.DestroyVisualizerWindow();
    }
}

// -----------------------------------------------------------------------------

bool Pipeline::updateVisualizer(open3d::visualization::Visualizer* vis_ptr) {
    bool updated = false;
    auto startTime = std::chrono::steady_clock::now();
    DataFrame decodedPoint_local_;
    size_t decodedPoint_size_ = decodedPoint_buffer_.read_available();
    if (decodedPoint_size_ > 0) {while (decodedPoint_buffer_.pop(decodedPoint_local_)) {}}
    // std::cerr << "[updateVisualizer] frame_id: " << decodedPoint_local_.frame_id << " numberpoints: " << decodedPoint_local_.numberpoints << std::endl;
    // size_t numToPrint = std::min<size_t>(5, decodedPoint_local_.numberpoints);
    // for (size_t i = 0; i < numToPrint; ++i) {
    //             std::cerr << "[Debug] Point " << i 
    //                     << ": x=" << decodedPoint_local_.x[i]
    //                     << ", y=" << decodedPoint_local_.y[i]
    //                     << ", z=" << decodedPoint_local_.z[i]
    //                     << ", reflectivity=" << decodedPoint_local_.reflectivity[i] 
    //                     << std::endl;
    // }
    if (!decodedPoint_local_.numberpoints <= 0) {
        updatePtCloudStream(point_cloud_ptr_, decodedPoint_local_);
        if (vis_ptr->UpdateGeometry(point_cloud_ptr_)) {updated = true;}
    }

    // Frame rate limiter
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    if (elapsedTime < targetFrameDuration) {std::this_thread::sleep_for(targetFrameDuration - elapsedTime);}
            
    return running_.load(std::memory_order_acquire) || updated;
}

// -----------------------------------------------------------------------------

void Pipeline::updatePtCloudStream(std::shared_ptr<open3d::geometry::PointCloud>& ptCloud_ptr, const DataFrame& frame) {
    if (frame.numberpoints <= 0) { return; }

    // Resize point cloud to accommodate new data
    ptCloud_ptr->points_.resize(frame.numberpoints);
    ptCloud_ptr->colors_.resize(frame.numberpoints);

    // Single loop for coordinates and color
    constexpr double log2_255_over_100 = 1.35049869499; // log2(255/100)
    for (size_t i = 0; i < frame.numberpoints; ++i) {
        // Assign transformed coordinates: Y→X, Z→-Y, X→Z
        ptCloud_ptr->points_[i] = Eigen::Vector3d(frame.x[i], -frame.y[i], -frame.z[i]);

        // Map reflectivity to parameter t
        double reflectivity = frame.reflectivity[i];
        double t;
        if (reflectivity <= 100) {
            // Linear mapping for lambertian targets (0 to 100)
            t = reflectivity / 200.0; // Maps 0 to 0, 100 to 0.5
        } else {
            // Logarithmic mapping for retroreflective targets (101 to 255)
            double s = 1.0 + 863.0 * (std::log2(reflectivity / 100.0) / log2_255_over_100);
            t = 0.5 + 0.5 * (s - 1.0) / 863.0; // Maps s=1 to 0.5, s=864 to 1.0
        }

        // Interpolate color:
        // - t = 0 (reflectivity = 0): royal blue (0.12, 0.29, 0.69)
        // - t = 0.5 (reflectivity = 100): neon green (0.0, 1.0, 0.0)
        // - t = 1 (reflectivity = 255): white (1.0, 1.0, 1.0)
        if (t < 0.5) {
            // Royal blue to neon green
            double u = t * 2.0; // [0, 1]
            ptCloud_ptr->colors_[i] = Eigen::Vector3d(
                0.12 - 0.12 * u,           // R: 0.12 to 0.0
                0.29 + (1.0 - 0.29) * u,   // G: 0.29 to 1.0
                0.69 - 0.69 * u            // B: 0.69 to 0.0
            );
        } else {
            // Neon green to white
            double u = (t - 0.5) * 2.0; // [0, 1]
            ptCloud_ptr->colors_[i] = Eigen::Vector3d(
                0.0 + 1.0 * u,            // R: 0.0 to 1.0
                1.0,                      // G: 1.0 to 1.0
                0.0 + 1.0 * u             // B: 0.0 to 1.0
            );
        }
    }
    ptCloud_ptr->VoxelDownSample(1);
}

