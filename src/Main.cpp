#include <Pipeline.hpp>

int main() {

    std::string json_path = "/home/khalis/Sync/SensorSOW/Arbeitspakete/MATLAB/Developement/033_DecodeLidar_RNG19_RFL8_SIG16_NIR1/json/20250520_1329_OS-2-128_122446000745.json";
    std::string pointsHost = "192.168.75.10";
    uint16_t pointsPort = 7502;
    uint32_t bufferSize = 24832;

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
        uint32_t core = 0;
        boost::asio::io_context ioContextPoints;

        threads.emplace_back([&]() { pipeline.runOusterLidarListener(ioContextPoints, pointsHost, pointsPort, bufferSize, std::vector<int>{0}); });
        threads.emplace_back([&]() { pipeline.runVisualizer(std::vector<int>{1}); });
        
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
    