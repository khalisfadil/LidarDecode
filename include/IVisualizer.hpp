#pragma once

#include <open3d/Open3D.h>
#include <dataframe.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <thread>
#include <atomic>

class Visualizer {
public:
    Visualizer();
    ~Visualizer();

    void start();
    void stop();
    bool push_frame(const DataFrame& frame);

private:
    void visualize_thread();

    std::shared_ptr<open3d::geometry::PointCloud> point_cloud_;
    std::shared_ptr<open3d::visualization::Visualizer> vis_;
    boost::lockfree::spsc_queue<DataFrame, boost::lockfree::capacity<32>> frame_queue_;
    std::thread vis_thread_;
    std::atomic<bool> running_;
};