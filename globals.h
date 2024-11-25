#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>

// Global variables
extern std::string filebase_path; // 'Path-1' or 'Path-2'
extern bool imu_data_flag;         // '0' for imu data: x, y, z; '1' for imu data: ax, ay, az
extern double fs_imu;             // IMU sample frequency
extern double fs_gps;             // GPS sample frequency
extern double R;                  // Measurement noise value
extern double Q;                  // Process noise value

#endif // GLOBALS_H
