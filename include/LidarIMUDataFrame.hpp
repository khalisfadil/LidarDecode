#pragma once

#include <cstdint>

namespace lidarDecode {

    struct LidarIMUDataFrame {
        double Normalized_Timestamp_s = 0.0;  // Normalized timestamp in seconds
        double IMU_Diagnostic_Time_s = 0.0;  // Diagnostic timestamp in seconds
        double Accelerometer_Read_Time_s = 0.0;  // Accelerometer timestamp in seconds
        double Gyroscope_Read_Time_s = 0.0;  // Gyroscope timestamp in seconds
        float Acceleration_X = 0.0;  // g
        float Acceleration_Y = 0.0;  // g
        float Acceleration_Z = 0.0;  // g
        float AngularVelocity_X = 0.0;  // deg/s
        float AngularVelocity_Y = 0.0;  // deg/s
        float AngularVelocity_Z = 0.0;  // deg/s

        // Clear all fields
        void clear() {
            Normalized_Timestamp_s = 0.0;
            IMU_Diagnostic_Time_s = 0.0;
            Accelerometer_Read_Time_s = 0.0;
            Gyroscope_Read_Time_s = 0.0;
            Acceleration_X = 0.0;
            Acceleration_Y = 0.0;
            Acceleration_Z = 0.0;
            AngularVelocity_X = 0.0;
            AngularVelocity_Y = 0.0;
            AngularVelocity_Z = 0.0;
        }
    };

} // namespace lidarDecode
