#pragma once

#include <cstdint>

namespace lidarDecode {

    struct Point3D {
        double x = 0.0;                // X coordinate in sensor frame
        double y = 0.0;                // Y coordinate in sensor frame
        double z = 0.0;                // Z coordinate in sensor frame
        uint16_t c_id = -1;            // Channel index (0 to 127, corresponding to row)
        uint16_t m_id = -1;            // Measurement index (0 to 2047, corresponding to column)
        double timestamp = 0.0;        // Absolute timestamp (seconds)
        double relative_timestamp = 0.0; // Relative timestamp from the initial frame (seconds)
        uint16_t reflectivity = 0;     // Reflectivity value
        uint16_t signal = 0;           // Signal strength
        uint16_t nir = 0;              // Near-infrared value
    };

} // namespace lidarDecode