#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include <mutex>

#include <Point.hpp>
#include <Dataframe.hpp>


class OusterLidarCallback {
    public:
        explicit OusterLidarCallback(const std::string& json_path);
        explicit OusterLidarCallback(const nlohmann::json& json_data);
        void decode_packet_single_return(const std::vector<uint8_t>& packet, DataFrame& frame);
        const DataFrame& get_latest_frame() const {return buffer_toggle_ ? data_buffer1_ : data_buffer2_;}
        const nlohmann::json& get_metadata() const {return metadata_; }

    private:
        nlohmann::json metadata_;
        // Lookup tables: [m_id][c_id] for contiguous access per measurement
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> x_1_; // Per-measurement, per-channel
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> y_1_;
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> z_1_;
        std::vector<double> x_2_;
        std::vector<double> y_2_;
        std::vector<double> z_2_;
        std::vector<double> r_min_;
        std::vector<double> sin_beam_azimuths_;
        std::vector<double> cos_beam_azimuths_;
        std::vector<double> sin_beam_altitudes_;
        std::vector<double> cos_beam_altitudes_;
        std::vector<int> pixel_shifts_;
        Eigen::Matrix4d lidar_to_sensor_transform_;
        Eigen::Matrix4d vehicle_to_lidar_transform_;
        double lidar_origin_to_beam_origin_mm_;

        size_t block_size_ = 1548;
        size_t header_size_ = 32;
        size_t footer_size_ = 32;
        size_t expected_size_ = header_size_ + block_size_ * 16 + footer_size_;

        int columns_per_frame_ = 2048;
        int pixels_per_column_ = 128;
        int columns_per_packet_ = 16;
        double signal_multiplier_ = 1.0;

        uint16_t frame_id_ = 0;
        uint32_t number_points_ = 0;
        DataFrame data_buffer1_;
        DataFrame data_buffer2_;
        bool buffer_toggle_ = true;
        double latest_timestamp_s = 0.0;

        void initialize();
        void parse_metadata(const nlohmann::json& json_data);
        void swap_buffers() {buffer_toggle_ = !buffer_toggle_;};
};