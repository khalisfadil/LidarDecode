#pragma once

#include <cstdint>

struct LidarIMUDataFrame {

    uint64_t IMU_Diagnostic_Time = 0.0;
    uint64_t Accelerometer_Read_Time = 0.0;
    uint64_t Gyroscope_Read_Time = 0.0;
    float Acceleration_X = 0.0;
    float Acceleration_Y = 0.0;
    float Acceleration_Z = 0.0;
    float AngularVelocity_X = 0.0;
    float AngularVelocity_Y = 0.0;
    float AngularVelocity_Z = 0.0;

    // Clear all vectors
    void clear() {
        IMU_Diagnostic_Time = 0.0;
        Accelerometer_Read_Time = 0.0;
        Gyroscope_Read_Time = 0.0;
        Acceleration_X = 0.0;
        Acceleration_Y = 0.0;
        Acceleration_Z = 0.0;
        AngularVelocity_X = 0.0;
        AngularVelocity_Y = 0.0;
        AngularVelocity_Z = 0.0;
    }
};