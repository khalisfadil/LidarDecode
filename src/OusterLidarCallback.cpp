#include <fstream>
#include <stdexcept>
#include <cmath>
#include <cstring>
#include <iostream>

#include <OusterLidarCallback.hpp>

#include <cstring>
#ifdef __AVX2__
    #include <immintrin.h>
#endif

using json = nlohmann::json;

OusterLidarCallback::OusterLidarCallback(const std::string& json_path) {
    std::ifstream file(json_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON file: " + json_path);
    }
    json json_data;
    try {
        file >> json_data;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parse error in " + json_path + ": " + e.what());
    }
    parse_metadata(json_data);
    initialize();
}

OusterLidarCallback::OusterLidarCallback(const json& json_data) {
    parse_metadata(json_data);
    initialize();
}

void OusterLidarCallback::parse_metadata(const json& json_data) {
    if (!json_data.is_object()) {
        throw std::runtime_error("JSON data must be an object");
    }
    json data = json_data;

    try {
        if (!data.contains("lidar_data_format") || !data["lidar_data_format"].is_object()) {
            throw std::runtime_error("Missing or invalid 'lidar_data_format' object");
        }
        if (!data.contains("config_params") || !data["config_params"].is_object()) {
            throw std::runtime_error("Missing or invalid 'config_params' object");
        }
        if (!data.contains("beam_intrinsics") || !data["beam_intrinsics"].is_object()) {
            throw std::runtime_error("Missing or invalid 'beam_intrinsics' object");
        }

        columns_per_frame_ = data["lidar_data_format"]["columns_per_frame"].get<int>();
        pixels_per_column_ = data["lidar_data_format"]["pixels_per_column"].get<int>();
        columns_per_packet_ = data["config_params"]["columns_per_packet"].get<int>();
        signal_multiplier_ = data["config_params"]["signal_multiplier"].get<double>();

        const auto& beam_intrinsics = data["beam_intrinsics"];
        lidar_origin_to_beam_origin_mm_ = beam_intrinsics["lidar_origin_to_beam_origin_mm"].get<double>();

        const auto& pixel_shift_by_row = data["lidar_data_format"]["pixel_shift_by_row"];
        if (!pixel_shift_by_row.is_array() || pixel_shift_by_row.size() != pixels_per_column_) {
            throw std::runtime_error("'pixel_shift_by_row' must be an array of " + std::to_string(pixels_per_column_) + " elements");
        }
        pixel_shifts_.resize(pixels_per_column_);
        for (size_t i = 0; i < pixels_per_column_; ++i) {
            pixel_shifts_[i] = pixel_shift_by_row[i].get<int>();
        }

        const auto& lidar_transform = data["lidar_intrinsics"]["lidar_to_sensor_transform"];
        if (!lidar_transform.is_array() || lidar_transform.size() != 16) {
            throw std::runtime_error("'lidar_to_sensor_transform' must be an array of 16 elements");
        }
        lidar_to_sensor_transform_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d raw_transform = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double value = lidar_transform[i * 4 + j].get<double>();
                raw_transform(i, j) = value;
                // Scale translation components (last column) from mm to m
                if (j == 3 && i < 3) {
                    lidar_to_sensor_transform_(i, j) = value * 0.001;
                } else {
                    lidar_to_sensor_transform_(i, j) = value;
                }
            }
        }
    } catch (const json::exception& e) {throw std::runtime_error("JSON parsing error: " + std::string(e.what()));}
    metadata_ = data;
}

void OusterLidarCallback::initialize() {
    const auto& beam_intrinsics = metadata_["beam_intrinsics"];
    const auto& azimuth_angles = beam_intrinsics["beam_azimuth_angles"];
    const auto& altitude_angles = beam_intrinsics["beam_altitude_angles"];

    data_buffer1_.reserve(columns_per_frame_ * pixels_per_column_);
    data_buffer2_.reserve(columns_per_frame_ * pixels_per_column_);

    sin_beam_azimuths_.resize(pixels_per_column_);
    cos_beam_azimuths_.resize(pixels_per_column_);
    sin_beam_altitudes_.resize(pixels_per_column_);
    cos_beam_altitudes_.resize(pixels_per_column_);
    r_min_.resize(pixels_per_column_, 0.1);

    // Initialize transposed lookup tables: [m_id][c_id]
    x_1_.clear();
    y_1_.clear();
    z_1_.clear();
    x_1_.reserve(columns_per_frame_);
    y_1_.reserve(columns_per_frame_);
    z_1_.reserve(columns_per_frame_);
    for (size_t m_id = 0; m_id < columns_per_frame_; ++m_id) {
        std::vector<double, Eigen::aligned_allocator<double>> x_inner(pixels_per_column_);
        std::vector<double, Eigen::aligned_allocator<double>> y_inner(pixels_per_column_);
        std::vector<double, Eigen::aligned_allocator<double>> z_inner(pixels_per_column_);
        x_1_.push_back(std::move(x_inner));
        y_1_.push_back(std::move(y_inner));
        z_1_.push_back(std::move(z_inner));
    }

    x_2_.resize(columns_per_frame_);
    y_2_.resize(columns_per_frame_);
    z_2_.resize(columns_per_frame_);

    // Transformation matrix: sensor (x=back, y=right, z=up) to desired (x=front, y=right, z=down)
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform(0, 0) = -1.0; // x' = -x (flip x-back to x-front)
    transform(1, 1) = 1.0;  // y' = y (keep y-right)
    transform(2, 2) = -1.0; // z' = -z (flip z-up to z-down)

    for (size_t i = 0; i < pixels_per_column_; ++i) {
        double az = azimuth_angles[i].get<double>() * M_PI / 180.0;
        double alt = altitude_angles[i].get<double>() * M_PI / 180.0;
        sin_beam_azimuths_[i] = std::sin(az);
        cos_beam_azimuths_[i] = std::cos(az);
        sin_beam_altitudes_[i] = std::sin(alt);
        cos_beam_altitudes_[i] = std::cos(alt);
    }

    for (size_t m_id = 0; m_id < columns_per_frame_; ++m_id) {
        double azimuth_rad = m_id * 2.0 * M_PI / columns_per_frame_;
        double cos_az = std::cos(azimuth_rad);
        double sin_az = std::sin(azimuth_rad);
        Eigen::Vector4d offset(lidar_origin_to_beam_origin_mm_ * 0.001 * cos_az,
                               lidar_origin_to_beam_origin_mm_ * 0.001 * sin_az,
                               0.0, 1.0);
        // Apply lidar_to_sensor_transform_ first, then sensor-to-desired transform
        offset = lidar_to_sensor_transform_ * offset;
        offset = transform * offset;
        x_2_[m_id] = offset.x();
        y_2_[m_id] = offset.y();
        z_2_[m_id] = offset.z();

        for (size_t ch = 0; ch < pixels_per_column_; ++ch) {
            double total_az = azimuth_rad + azimuth_angles[ch].get<double>() * M_PI / 180.0;
            double cos_total_az = std::cos(total_az);
            double sin_total_az = std::sin(total_az);
            double cos_alt = cos_beam_altitudes_[ch];
            double sin_alt = sin_beam_altitudes_[ch];
            Eigen::Vector4d dir(cos_alt * cos_total_az, cos_alt * sin_total_az, sin_alt, 1.0);
            // Apply lidar_to_sensor_transform_ first, then sensor-to-desired transform
            dir = lidar_to_sensor_transform_ * dir;
            dir = transform * dir;
            x_1_[m_id][ch] = dir.x();
            y_1_[m_id][ch] = dir.y();
            z_1_[m_id][ch] = dir.z();
        }
    }

    if (x_1_.size() != columns_per_frame_ || x_1_[0].size() != pixels_per_column_) {
        throw std::runtime_error("x_1_ lookup table size mismatch");
    }
    if (x_2_.size() != columns_per_frame_) {
        throw std::runtime_error("x_2_ lookup table size mismatch");
    }
    if (pixel_shifts_.size() != pixels_per_column_) {
        throw std::runtime_error("pixel_shifts_ size mismatch");
    }

#ifdef DEBUG
    assert(reinterpret_cast<uintptr_t>(x_1_[0].data()) % 32 == 0);
#endif
}

void OusterLidarCallback::decode_packet_single_return(const std::vector<uint8_t>& packet, DataFrame& frame) {
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    uint16_t packet_type;
    std::memcpy(&packet_type, packet.data(), sizeof(uint16_t));
    packet_type = le16toh(packet_type);
    if (packet_type != 0x1) {
        std::cerr << "Invalid packet type: " << packet_type << " (expected 0x1)" << std::endl;
        return;
    }

    uint16_t packet_frame_id;
    std::memcpy(&packet_frame_id, packet.data() + 2, sizeof(uint16_t));
    packet_frame_id = le16toh(packet_frame_id);

    double prevframe_latest_timestamp_s = 0.0;
    if (packet_frame_id != frame_id_) {
        // std::cerr << "frame_id_: " << frame_id_ << std::endl;
        // std::cerr << "numberpoints: " << number_points_ << std::endl;
        // std::cerr << "interframe_timedelta: " << interframe_timedelta << std::endl;
        // std::cerr << "timestamp: " << timestamp << std::endl;
        swap_buffers();
        number_points_ = 0;
        frame_id_ = packet_frame_id;
        prevframe_latest_timestamp_s = latest_timestamp_s;
        DataFrame& write_buffer = buffer_toggle_ ? data_buffer2_ : data_buffer1_;
        write_buffer.clear();
        write_buffer.reserve(columns_per_frame_ * pixels_per_column_);
    }

    DataFrame& write_buffer = buffer_toggle_ ? data_buffer2_ : data_buffer1_;
    bool is_first_point = (number_points_ == 0); // Track if this is the first point of the frame

    for (int col = 0; col < columns_per_packet_; ++col) {
        size_t block_offset = header_size_ + col * block_size_;

        uint64_t timestamp_ns;
        std::memcpy(&timestamp_ns, packet.data() + block_offset, sizeof(uint64_t));
        timestamp_ns = le64toh(timestamp_ns);
        double timestamp_s = timestamp_ns * 1e-9;
        if (timestamp_s < 0) {
            std::cerr << "Negative timestamp: " << timestamp_s << std::endl;
            continue;
        }
        latest_timestamp_s = timestamp_s;

        uint16_t m_id;
        std::memcpy(&m_id, packet.data() + block_offset + 8, sizeof(uint16_t));
        m_id = le16toh(m_id);
        if (m_id >= columns_per_frame_) {
            std::cerr << "Invalid measurement ID: " << m_id << std::endl;
            continue;
        }

        if (!(packet[block_offset + 10] & 0x01)) {
            continue;
        }

#ifdef __AVX2__
        // AVX2 implementation
        for (uint16_t c_id = 0; c_id < pixels_per_column_; c_id += 4) {
            size_t pixel_offset = block_offset + 12 + c_id * 12;

            alignas(32) double range_m[4];
            alignas(32) double r_min_vals[4];
            uint16_t c_ids[4], reflectivity[4], signal[4], nir[4];
            for (int i = 0; i < 4 && c_id + i < pixels_per_column_; ++i) {
                size_t offset = pixel_offset + i * 12;
                if (offset + 11 >= packet.size()) {
                    range_m[i] = 0.0;
                    continue;
                }
                uint32_t range_mm;
                uint8_t range_bytes[4] = {packet[offset], packet[offset + 1], packet[offset + 2], 0};
                std::memcpy(&range_mm, range_bytes, 4);
                range_mm = le32toh(range_mm) & 0x0007FFFF;
                range_m[i] = range_mm * 0.001;
                r_min_vals[i] = r_min_[c_id + i];
                c_ids[i] = c_id + i;
                reflectivity[i] = packet[offset + 4];
                std::memcpy(&signal[i], packet.data() + offset + 6, sizeof(uint16_t));
                std::memcpy(&nir[i], packet.data() + offset + 8, sizeof(uint16_t));
                signal[i] = le16toh(signal[i]);
                nir[i] = le16toh(nir[i]);
            }

            __m256d range = _mm256_load_pd(range_m);
            __m256d r_min_vec = _mm256_load_pd(r_min_vals);
            __m256d valid_mask = _mm256_cmp_pd(range, r_min_vec, _CMP_GE_OQ);

            __m256d x1 = _mm256_load_pd(x_1_[m_id].data() + c_id);
            __m256d y1 = _mm256_load_pd(y_1_[m_id].data() + c_id);
            __m256d z1 = _mm256_load_pd(z_1_[m_id].data() + c_id);
            __m256d x2 = _mm256_set1_pd(x_2_[m_id]);
            __m256d y2 = _mm256_set1_pd(y_2_[m_id]);
            __m256d z2 = _mm256_set1_pd(z_2_[m_id]);

            __m256d pt_x = _mm256_fmadd_pd(range, x1, x2);
            __m256d pt_y = _mm256_fmadd_pd(range, y1, y2);
            __m256d pt_z = _mm256_fmadd_pd(range, z1, z2);

            alignas(32) double pt_x_arr[4], pt_y_arr[4], pt_z_arr[4];
            _mm256_store_pd(pt_x_arr, pt_x);
            _mm256_store_pd(pt_y_arr, pt_y);
            _mm256_store_pd(pt_z_arr, pt_z);

            double relative_timestamp_s = write_buffer.numberpoints > 0
                ? std::max(0.0, timestamp_s - write_buffer.timestamp)
                : 0.0;

            for (int i = 0; i < 4 && c_id + i < pixels_per_column_; ++i) {
                if (range_m[i] < r_min_vals[i]) continue;
                write_buffer.x.push_back(pt_x_arr[i]);
                write_buffer.y.push_back(pt_y_arr[i]);
                write_buffer.z.push_back(pt_z_arr[i]);
                write_buffer.c_id.push_back(c_ids[i]);
                write_buffer.m_id.push_back(m_id);
                write_buffer.timestamp_points.push_back(timestamp_s);
                write_buffer.relative_timestamp.push_back(relative_timestamp_s);
                write_buffer.reflectivity.push_back(reflectivity[i]);
                write_buffer.signal.push_back(signal[i]);
                write_buffer.nir.push_back(nir[i]);
                ++number_points_;
                if (is_first_point) {
                    write_buffer.timestamp = timestamp_s;
                    write_buffer.frame_id = frame_id_;
                    write_buffer.interframe_timedelta = timestamp_s - prevframe_latest_timestamp_s;
                    is_first_point = false;
                }
            }
        }
#else
        // Scalar fallback
        for (uint16_t c_id = 0; c_id < pixels_per_column_; ++c_id) {
            size_t pixel_offset = block_offset + 12 + c_id * 12;
            if (pixel_offset + 11 >= packet.size()) {
                std::cerr << "Pixel offset out of bounds\n";
                continue;
            }

            uint32_t range_mm;
            uint8_t range_bytes[4] = {packet[pixel_offset], packet[pixel_offset + 1], packet[pixel_offset + 2], 0};
            std::memcpy(&range_mm, range_bytes, 4);
            range_mm = le32toh(range_mm) & 0x0007FFFF;
            double range_m = range_mm * 0.001;
            if (range_m < r_min_[c_id]) {
                continue;
            }

            uint8_t reflectivity = packet[pixel_offset + 4];
            uint16_t signal, nir;
            std::memcpy(&signal, packet.data() + pixel_offset + 6, sizeof(uint16_t));
            std::memcpy(&nir, packet.data() + pixel_offset + 8, sizeof(uint16_t));
            signal = le16toh(signal);
            nir = le16toh(nir);

            double pt_x = range_m * x_1_[m_id][c_id] + x_2_[m_id];
            double pt_y = range_m * y_1_[m_id][c_id] + y_2_[m_id];
            double pt_z = range_m * z_1_[m_id][c_id] + z_2_[m_id];

            double relative_timestamp_s = write_buffer.numberpoints > 0 ? std::max(0.0, timestamp_s - write_buffer.timestamp) : 0.0;

            write_buffer.x.push_back(pt_x);
            write_buffer.y.push_back(pt_y);
            write_buffer.z.push_back(pt_z);
            write_buffer.c_id.push_back(c_id);
            write_buffer.m_id.push_back(m_id);
            write_buffer.timestamp_points.push_back(timestamp_s);
            write_buffer.relative_timestamp.push_back(relative_timestamp_s);
            write_buffer.reflectivity.push_back(reflectivity);
            write_buffer.signal.push_back(signal);
            write_buffer.nir.push_back(nir);
            ++number_points_;
            if (is_first_point) {
                write_buffer.timestamp = timestamp_s;
                write_buffer.frame_id = frame_id_;
                write_buffer.interframe_timedelta = timestamp_s - prevframe_latest_timestamp_s;
                is_first_point = false;
            }
        }
#endif
    }
    write_buffer.numberpoints = number_points_;
    frame = get_latest_frame();
}

