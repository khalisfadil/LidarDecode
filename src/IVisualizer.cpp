#include <IVisualizer.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <limits>

Visualizer::Visualizer() : running_(false) {
    point_cloud_ = std::make_shared<open3d::geometry::PointCloud>();
}

Visualizer::~Visualizer() {
    stop();
}

void Visualizer::start() {
    if (running_) return;
    running_ = true;
    vis_thread_ = std::thread(&Visualizer::visualize_thread, this);
    std::cerr << "Visualizer thread started" << std::endl;
}

void Visualizer::stop() {
    if (!running_) return;
    running_ = false;
    if (vis_thread_.joinable()) {
        vis_thread_.join();
    }
    std::cerr << "Visualizer stopped" << std::endl;
}

bool Visualizer::push_frame(const DataFrame& frame) {
    if (!frame_queue_.push(frame)) {
        std::cerr << "Frame queue full, dropping frame with " << frame.numberpoints << " points" << std::endl;
        return false;
    }
    std::cerr << "Pushed frame with " << frame.numberpoints << " points to queue" << std::endl;
    return true;
}

void Visualizer::visualize_thread() {
    try {
        vis_ = std::make_shared<open3d::visualization::Visualizer>();
        vis_->CreateVisualizerWindow("Ouster Point Cloud", 1280, 720);
        vis_->AddGeometry(point_cloud_);
        auto& view = vis_->GetViewControl();
        view.SetLookat(Eigen::Vector3d(0, 0, 0));
        view.SetFront(Eigen::Vector3d(0, 0, -1));
        view.SetUp(Eigen::Vector3d(0, 1, 0));
        view.SetZoom(0.5);
        vis_->GetRenderOption().point_size_ = 5.0;
        vis_->GetRenderOption().background_color_ = Eigen::Vector3d(0.2, 0.2, 0.2);
        // Add test points
        point_cloud_->points_.push_back(Eigen::Vector3d(0, 0, 0));
        point_cloud_->points_.push_back(Eigen::Vector3d(1, 0, 0));
        point_cloud_->points_.push_back(Eigen::Vector3d(0, 1, 0));
        point_cloud_->colors_.push_back(Eigen::Vector3d(1, 0, 0));
        point_cloud_->colors_.push_back(Eigen::Vector3d(0, 1, 0));
        point_cloud_->colors_.push_back(Eigen::Vector3d(0, 0, 1));
        vis_->UpdateGeometry(point_cloud_);
        std::cerr << "Open3D visualizer initialized with test points" << std::endl;

        DataFrame frame;
        while (running_) {
            if (frame_queue_.pop(frame)) {
                if (frame.numberpoints == 0) {
                    std::cerr << "Skipping empty frame (frame_id: " << frame.frame_id << ")" << std::endl;
                    continue;
                }
                auto start = std::chrono::steady_clock::now();
                std::cerr << "Rendering frame with " << frame.numberpoints << " points (frame_id: " << frame.frame_id << ")" << std::endl;
                point_cloud_->points_.clear();
                point_cloud_->colors_.clear();
                // Re-add test points
                point_cloud_->points_.push_back(Eigen::Vector3d(0, 0, 0));
                point_cloud_->points_.push_back(Eigen::Vector3d(1, 0, 0));
                point_cloud_->points_.push_back(Eigen::Vector3d(0, 1, 0));
                point_cloud_->colors_.push_back(Eigen::Vector3d(1, 0, 0));
                point_cloud_->colors_.push_back(Eigen::Vector3d(0, 1, 0));
                point_cloud_->colors_.push_back(Eigen::Vector3d(0, 0, 1));
                point_cloud_->points_.reserve(frame.numberpoints + 3);
                point_cloud_->colors_.reserve(frame.numberpoints + 3);
                Eigen::Vector3d min_bound(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                Eigen::Vector3d max_bound(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
                std::vector<double> intensities;
                intensities.reserve(frame.numberpoints);
                size_t valid_points = 0;
                for (size_t i = 0; i < frame.numberpoints; ++i) {
                    double x = frame.x[i], y = -frame.y[i], z = -frame.z[i];
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                        std::cerr << "Invalid point at index " << i << ": (" << x << ", " << y << ", " << z << ")" << std::endl;
                        continue;
                    }
                    point_cloud_->points_.emplace_back(x, y, z);
                    intensities.push_back(frame.reflectivity[i] / 255.0);
                    Eigen::Vector3d point(x, y, z);
                    min_bound = min_bound.cwiseMin(point);
                    max_bound = max_bound.cwiseMax(point);
                    valid_points++;
                    if (i < 5) {
                        std::cerr << "Point " << i << ": (" << frame.x[i] << ", " << frame.y[i] << ", " << frame.z[i] << ") -> ("<< x << ", " << y << ", " << z << "), reflectivity: " << frame.reflectivity[i] << std::endl;
                    }
                }
                std::cerr << "Processed " << valid_points << " valid points out of " << frame.numberpoints << std::endl;
                size_t points_before = point_cloud_->points_.size() - 3; // Exclude test points
                if (points_before > 0) {
                    auto downsampled = point_cloud_->VoxelDownSample(0.05); // 5cm voxel
                    point_cloud_->points_ = downsampled->points_;
                    // Re-add test points after downsampling
                    point_cloud_->points_.push_back(Eigen::Vector3d(0, 0, 0));
                    point_cloud_->points_.push_back(Eigen::Vector3d(1, 0, 0));
                    point_cloud_->points_.push_back(Eigen::Vector3d(0, 1, 0));
                    point_cloud_->colors_.clear();
                    point_cloud_->colors_.reserve(point_cloud_->points_.size());
                    size_t intensity_idx = 0;
                    for (size_t i = 0; i < point_cloud_->points_.size() - 3; ++i) {
                        double intensity = intensity_idx < intensities.size() ? intensities[intensity_idx++] : 0.5;
                        intensity = std::min(intensity * 5.0, 1.0); // Amplify intensity
                        point_cloud_->colors_.emplace_back(intensity, intensity, intensity); // Grayscale
                    }
                    point_cloud_->colors_.push_back(Eigen::Vector3d(1, 0, 0));
                    point_cloud_->colors_.push_back(Eigen::Vector3d(0, 1, 0));
                    point_cloud_->colors_.push_back(Eigen::Vector3d(0, 0, 1));
                    std::cerr << "Downsampled from " << points_before << " to " << point_cloud_->points_.size() - 3 << " points" << std::endl;
                }
                if (!point_cloud_->points_.empty()) {
                    auto& view = vis_->GetViewControl();
                    Eigen::Vector3d centroid = (min_bound + max_bound) * 0.5;
                    double extent = (max_bound - min_bound).norm() * 0.5;
                    view.SetLookat(centroid);
                    view.SetFront(Eigen::Vector3d(0, 0, -1));
                    view.SetUp(Eigen::Vector3d(0, 1, 0));
                    view.SetZoom(extent > 0 ? 1.0 / extent : 1.0); // Dynamic zoom
                    std::cerr << "Point cloud bounds: min (" << min_bound.transpose() << "), max (" << max_bound.transpose() << ")" << std::endl;
                    std::cerr << "Camera centered at: " << centroid.transpose() << ", extent: " << extent << ", zoom: " << (extent > 0 ? 1.0 / extent : 1.0) << std::endl;
                } else {
                    std::cerr << "No valid points after downsampling" << std::endl;
                }
                vis_->UpdateGeometry(point_cloud_);
                vis_->PollEvents();
                vis_->UpdateRender();
                auto end = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                std::cerr << "Frame rendered in " << duration << " ms" << std::endl;
            } else {
                vis_->PollEvents();
                std::this_thread::yield();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Visualizer error: " << e.what() << std::endl;
        running_ = false;
    }

    if (vis_) {
        vis_->DestroyVisualizerWindow();
        vis_.reset();
        std::cerr << "Visualizer window destroyed" << std::endl;
    }
}