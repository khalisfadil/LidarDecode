#pragma once

#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <thread>
#include <iostream>

#include <UdpSocket.hpp>
#include <OusterLidarCallback.hpp>
#include <Dataframe.hpp>

class Pipeline {
    public:

        static std::atomic<bool> running_;
        static std::condition_variable globalCV_;
        static boost::lockfree::spsc_queue<DataFrame, boost::lockfree::capacity<128>> decodedPoint_buffer_;

        Pipeline(const std::string& json_path); // Constructor with JSON file path
        Pipeline(const nlohmann::json& json_data); // Constructor with JSON data
        // ~Pipeline();
        static void signalHandler(int signal);
        void setThreadAffinity(const std::vector<int>& coreIDs);
        void runOusterLidarListener(boost::asio::io_context& ioContext, const std::string& host, uint16_t port, uint32_t bufferSize, const std::vector<int>& allowedCores); 
        // void runVisualizer(const std::vector<int>& allowedCores);

    private:

        std::mutex consoleMutex;
        OusterLidarCallback lidarCallback;
        uint16_t frame_id_= 0;

};