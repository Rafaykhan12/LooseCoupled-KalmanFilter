// git clone https://github.com/Rafaykhan12/LooseCoupled-KalmanFilter.git#include <iostream>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "kalmanfilter.h"
#include <iomanip>
#include "globals.h"
// Structs to store data from each CSV file
struct AccelData {
    double accel_x, accel_y, accel_z;
};

struct GPSData {
    double gps_x, gps_y, gps_z;
};

struct TimeData {
    double time;
};

struct RefPosData {
    double ref_pos_x, ref_pos_y, ref_pos_z;
};

// Function to read AccelData
std::vector<AccelData> readAccelData(const std::string& filename) {
    std::vector<AccelData> accelData;
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        AccelData data;
        char comma;
        ss >> data.accel_x >> comma >> data.accel_y >> comma >> data.accel_z;
        accelData.push_back(data);
    }
    return accelData;
}

// Function to read GPSData (without velocity components)
std::vector<GPSData> readGPSData(const std::string& filename) {
    std::vector<GPSData> gpsData;
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        GPSData data;
        char comma;
        ss >> data.gps_x >> comma >> data.gps_y >> comma >> data.gps_z;
        gpsData.push_back(data);
    }
    return gpsData;
}

// Function to read TimeData
std::vector<TimeData> readTimeData(const std::string& filename) {
    std::vector<TimeData> timeData;
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        TimeData data;
        ss >> data.time;
        timeData.push_back(data);
    }
    return timeData;
}

// Function to read RefPosData
std::vector<RefPosData> readRefPosData(const std::string& filename) {
    std::vector<RefPosData> refPosData;
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::istringstream ss(line);
        RefPosData data;
        char comma;
        ss >> data.ref_pos_x >> comma >> data.ref_pos_y >> comma >> data.ref_pos_z;
        refPosData.push_back(data);
    }
    return refPosData;
}
// Function to save kf_values to a CSV file
void saveToCSV(const std::vector<Eigen::Vector3d>& kf_values, const std::string& filename) {
    std::ofstream file(filename);

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write the header for CSV
    file << "X,Y,Z\n";

    // Set the precision and format
    file << std::fixed << std::setprecision(6);

    // Write each vector's values to the file
    for (const auto& state : kf_values) {
        file << state(0) << "," << state(1) << "," << state(2) << "\n";
    }

    // Close the file
    file.close();
    std::cout << "Data saved to " << filename << std::endl;
}
// Kalman Filter processing
int main() {
    // File paths
    std::string gps_data_file = filebase_path+"gps.csv";
    std::string time_data_file = filebase_path+"time.csv";
    std::string true_position_file = filebase_path+"true_pos.csv";
    std::string imu_data_file = filebase_path+"imu_pos.csv";
    if(imu_data_flag){
    std::string imu_data_file = filebase_path+"accel.csv";
    }
   
    // Read data from files with header
    std::vector<AccelData> imu_data = readAccelData(imu_data_file);
    std::vector<GPSData> gps_data = readGPSData(gps_data_file);
    std::vector<TimeData> time_data = readTimeData(time_data_file);
    std::vector<RefPosData> true_position = readRefPosData(true_position_file);

    double dt = 0.01; // Adjust as needed
    int gps_update_interval = fs_imu/fs_gps; // Update rate for GPS in terms of IMU rate

    // Process noise covariance, measurement noise covariance, initial state covariance
    Eigen::MatrixXd process_noise_cov = Eigen::MatrixXd::Identity(6, 6) *Q;
    Eigen::MatrixXd measurement_noise_cov = Eigen::MatrixXd::Identity(3, 3) * R;
    Eigen::MatrixXd state_cov = Eigen::MatrixXd::Identity(6, 6) * 0.1;

    // Initial state vector
    Eigen::VectorXd initial_state_vector(6);
    initial_state_vector << true_position[0].ref_pos_x, 0, true_position[0].ref_pos_y, 0, true_position[0].ref_pos_z, 0;

    // Initialize Kalman filter
    KalmanFilter kf(dt, process_noise_cov, measurement_noise_cov, state_cov, initial_state_vector);

    // Storage for results
    std::vector<Eigen::Vector3d> kf_values;

    // Kalman Filter Loop
    for (size_t t = 0; t < imu_data.size(); ++t) {
        Eigen::Vector3d imu_pos(imu_data[t].accel_x, imu_data[t].accel_y, imu_data[t].accel_z);
        Eigen::VectorXd predicted_state = kf.predict(imu_pos, imu_data_flag);

        if (t % gps_update_interval == 0 && t / gps_update_interval < gps_data.size()) {
            size_t gps_idx = t / gps_update_interval;
            Eigen::Vector3d gps_pos(gps_data[gps_idx].gps_x, gps_data[gps_idx].gps_y, gps_data[gps_idx].gps_z);
            Eigen::VectorXd updated_state = kf.update(gps_pos);

            kf_values.push_back(Eigen::Vector3d(updated_state(0), updated_state(2), updated_state(4)));
        } 
        
        else {
       
            kf_values.push_back(Eigen::Vector3d(predicted_state(0), predicted_state(2), predicted_state(4)));
        }
    }
    
    // Save results to CSV
    saveToCSV(kf_values, filebase_path+"kf_values.csv");

    return 0;
}
