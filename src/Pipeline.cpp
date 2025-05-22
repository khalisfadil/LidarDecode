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
            DataFrame decodedPoint;
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