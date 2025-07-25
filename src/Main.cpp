#include <Pipeline.hpp>

namespace lidarDecode {

    int main() {
        std::string json_path = "/home/khalis/Sync/SensorSOW/Arbeitspakete/MATLAB/Developement/033_DecodeLidar_RNG19_RFL8_SIG16_NIR1/json/RNG19_RFL8_SIG16_NIR16_128x2048.json";
        uint32_t lidar_packet_size = 24832;

        // Read and parse JSON file to get udp_profile_lidar and udp_port_lidar
        std::string udp_profile_lidar;
        std::string udp_dest;
        uint16_t udp_port_lidar; // Fallback to default port
        uint16_t udp_port_imu;
        try {
            std::ifstream json_file(json_path);
            if (!json_file.is_open()) {
                std::cerr << "[Main] Error: Could not open JSON file: " << json_path << std::endl;
                return EXIT_FAILURE;
            }
            nlohmann::json metadata_;
            json_file >> metadata_;
            json_file.close(); // Explicitly close the file

            if (!metadata_.contains("lidar_data_format") || !metadata_["lidar_data_format"].is_object()) {
                throw std::runtime_error("Missing or invalid 'lidar_data_format' object");
                return EXIT_FAILURE;
            }
            if (!metadata_.contains("config_params") || !metadata_["config_params"].is_object()) {
                throw std::runtime_error("Missing or invalid 'config_params' object");
                return EXIT_FAILURE;
            }
            if (!metadata_.contains("beam_intrinsics") || !metadata_["beam_intrinsics"].is_object()) {
                throw std::runtime_error("Missing or invalid 'beam_intrinsics' object");
                return EXIT_FAILURE;
            }
            if (!metadata_.contains("lidar_intrinsics") || !metadata_["lidar_intrinsics"].is_object() ||
                !metadata_["lidar_intrinsics"].contains("lidar_to_sensor_transform")) {
                throw std::runtime_error("Missing or invalid 'lidar_intrinsics.lidar_to_sensor_transform'");
                return EXIT_FAILURE;
            }

            udp_profile_lidar = metadata_["config_params"]["udp_profile_lidar"].get<std::string>();
            udp_port_lidar = metadata_["config_params"]["udp_port_lidar"].get<uint16_t>();
            udp_port_imu = metadata_["config_params"]["udp_port_imu"].get<uint16_t>();
            udp_dest = metadata_["config_params"]["udp_dest"].get<std::string>();

        } catch (const std::exception& e) {
            std::cerr << "[Main] Error parsing JSON: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }

        Pipeline pipeline(json_path);

        struct sigaction sigIntHandler;
        sigIntHandler.sa_handler = Pipeline::signalHandler;
        sigemptyset(&sigIntHandler.sa_mask);
        sigIntHandler.sa_flags = 0;

        sigaction(SIGINT, &sigIntHandler, nullptr);
        sigaction(SIGTERM, &sigIntHandler, nullptr);

        std::cout << "[Main] Starting pipeline processes..." << std::endl;

        try {
            std::vector<std::thread> threads;
            boost::asio::io_context ioContextPoints;

            // Switch-case based on udp_profile_lidar
            enum class LidarProfile { RNG19_RFL8_SIG16_NIR16, LEGACY, UNKNOWN };
            LidarProfile profile;
            if (udp_profile_lidar == "RNG19_RFL8_SIG16_NIR16") {
                profile = LidarProfile::RNG19_RFL8_SIG16_NIR16;
                lidar_packet_size = 24832;
            } else if (udp_profile_lidar == "LEGACY") {
                profile = LidarProfile::LEGACY;
                lidar_packet_size = 24896;
            } else {
                profile = LidarProfile::UNKNOWN;
            }

            switch (profile) {
                case LidarProfile::RNG19_RFL8_SIG16_NIR16:
                    std::cout << "[Main] Detected RNG19_RFL8_SIG16_NIR16 lidar udp profile." << std::endl;
                    // Use default parameters or adjust if needed
                    threads.emplace_back([&]() { 
                        pipeline.runOusterLidarListenerSingleReturn(ioContextPoints, udp_dest, udp_port_lidar, lidar_packet_size, std::vector<int>{0}); 
                    });
                    break;

                case LidarProfile::LEGACY:
                    std::cout << "[Main] Detected LEGACY lidar udp profile." << std::endl;
                    // Example: Adjust buffer size or port for LEGACY mode if needed
                    // bufferSize = 16384; // Example adjustment
                    threads.emplace_back([&]() { 
                        pipeline.runOusterLidarListenerLegacy(ioContextPoints, udp_dest, udp_port_lidar, lidar_packet_size, std::vector<int>{0}); 
                    });
                    break;

                case LidarProfile::UNKNOWN:
                default:
                    std::cerr << "[Main] Error: Unknown or unsupported udp_profile_lidar: " << udp_profile_lidar << std::endl;
                    return EXIT_FAILURE;
            }

            threads.emplace_back([&]() { 
                        pipeline.runOusterLidarIMUListener(ioContextPoints, udp_dest, udp_port_imu, lidar_packet_size, std::vector<int>{1}); 
                    });

            threads.emplace_back([&]() { pipeline.runVisualizer(std::vector<int>{2}); });

            while (Pipeline::running_.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            ioContextPoints.stop();

            for (auto& thread : threads) {
                if (thread.joinable()) {
                    thread.join();
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: [Main] " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "[Main] All processes stopped. Exiting program." << std::endl;
        return EXIT_SUCCESS;
    }

} // namespace lidarDecode