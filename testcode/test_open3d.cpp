#include <open3d/Open3D.h>
#include <iostream>
#include <Eigen/Core>

int main() {
    try {
        auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();
        point_cloud->points_.push_back(Eigen::Vector3d(0, 0, 0));
        point_cloud->points_.push_back(Eigen::Vector3d(1, 0, 0));
        point_cloud->points_.push_back(Eigen::Vector3d(0, 1, 0));
        point_cloud->colors_.push_back(Eigen::Vector3d(1, 0, 0)); // Red
        point_cloud->colors_.push_back(Eigen::Vector3d(0, 1, 0)); // Green
        point_cloud->colors_.push_back(Eigen::Vector3d(0, 0, 1)); // Blue

        open3d::visualization::Visualizer vis;
        vis.CreateVisualizerWindow("Test Open3D", 1280, 720);
        vis.AddGeometry(point_cloud);
        vis.GetRenderOption().point_size_ = 10.0;
        vis.GetRenderOption().background_color_ = Eigen::Vector3d(0.2, 0.2, 0.2);
        auto& view = vis.GetViewControl();
        view.SetLookat(Eigen::Vector3d(0.5, 0.5, 0)); // Center of points
        view.SetFront(Eigen::Vector3d(0, 0, -1)); // Look along negative Z
        view.SetUp(Eigen::Vector3d(0, 1, 0)); // Y-axis up
        view.SetZoom(2.0); // Zoom out further
        std::cout << "Visualizer initialized. Points at (0,0,0), (1,0,0), (0,1,0)" << std::endl;
        vis.Run();
        vis.DestroyVisualizerWindow();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    std::cout << "Visualizer closed." << std::endl;
    return 0;
}