#include "globals.h"

// Define global variables
std::string filebase_path = "./Path-1/"; // 'Path-1' or 'Path-2'
bool imu_data_flag =false ;                  // '0' for imu data: x, y, z; '1' for imu data: ax, ay, az
double fs_imu = 100.0;                  // IMU sample frequency
double fs_gps = 10.0;                   // GPS sample frequency
double R = 1.0;                         // Measurement noise value
double Q = 0.1;                         // Process noise value
