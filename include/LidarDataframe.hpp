#pragma once

#include <vector>
#include <cstdint>
#include <Eigen/Dense>

#include <Point.hpp>

namespace lidarDecode {

    struct LidarDataFrame {
        uint16_t frame_id = 0;
        double timestamp = 0.0;                       // Current timestamp
        double interframe_timedelta = 0.0;            // Time difference between first point in current frame and last point in last frame
        uint32_t numberpoints = 0;

        // Structure-of-Arrays for point cloud data
        std::vector<double, Eigen::aligned_allocator<double>> x; // X coordinates
        std::vector<double, Eigen::aligned_allocator<double>> y; // Y coordinates
        std::vector<double, Eigen::aligned_allocator<double>> z; // Z coordinates
        std::vector<uint16_t> c_id;                     // Channel indices
        std::vector<uint16_t> m_id;                     // Measurement indices
        std::vector<double, Eigen::aligned_allocator<double>> timestamp_points; // Absolute timestamps
        std::vector<double, Eigen::aligned_allocator<double>> relative_timestamp; // Relative timestamps
        std::vector<uint16_t> reflectivity;             // Reflectivity values
        std::vector<uint16_t> signal;                   // Signal strengths
        std::vector<uint16_t> nir;                      // NIR values

        // Reserve space for a full frame
        void reserve(size_t size) {
            x.reserve(size);
            y.reserve(size);
            z.reserve(size);
            c_id.reserve(size);
            m_id.reserve(size);
            timestamp_points.reserve(size);
            relative_timestamp.reserve(size);
            reflectivity.reserve(size);
            signal.reserve(size);
            nir.reserve(size);
        }

        // Clear all vectors
        void clear() {
            x.clear();
            y.clear();
            z.clear();
            c_id.clear();
            m_id.clear();
            timestamp_points.clear();
            relative_timestamp.clear();
            reflectivity.clear();
            signal.clear();
            nir.clear();
            numberpoints = 0;
        }

        // Convert to AoS (std::vector<Point3D>) for compatibility
        std::vector<Point3D> toPoint3D() const {
            std::vector<Point3D> pointcloud;
            pointcloud.reserve(numberpoints);
            for (size_t i = 0; i < numberpoints; ++i) {
                pointcloud.emplace_back(Point3D{
                    x[i], y[i], z[i],
                    c_id[i], m_id[i],
                    timestamp_points[i], relative_timestamp[i],
                    reflectivity[i], signal[i], nir[i]
                });
            }
            return pointcloud;
        }

        // Convert to AoS (std::vector<Points3D>) for compatibility
        std::vector<Points3D> toPoints3D() const {
            std::vector<Points3D> pointcloud;
            pointcloud.reserve(numberpoints);
            for (size_t i = 0; i < numberpoints; ++i) {
                Points3D point;
                point.raw_pt = Eigen::Vector3d(x[i], y[i], z[i]); // Raw sensor coordinates
                point.pt = point.raw_pt; // No motion correction; set equal to raw_pt
                point.att = Eigen::Vector3i::Zero(); // Placeholder: no direct mapping
                point.relative_timestamp = relative_timestamp[i]; // Assumed to be in [0.0, 1.0]
                point.timestamp = timestamp_points[i]; // Absolute timestamp
                point.m_id = static_cast<int>(m_id[i]); // Convert uint16_t to int
                pointcloud.push_back(point);
            }
            return pointcloud;
        }
    };

} // namespace lidarDecode