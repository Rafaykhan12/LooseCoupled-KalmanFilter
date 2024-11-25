#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(double dt, const Eigen::MatrixXd& process_noise_cov, const Eigen::MatrixXd& measurement_noise_cov,
                 const Eigen::MatrixXd& state_cov, const Eigen::VectorXd& initial_state_vector, int state_dim = 6);

    Eigen::VectorXd predict(const Eigen::Vector3d& imu_position, bool imu_data_flag = true);
    Eigen::VectorXd update(const Eigen::Vector3d& gps_position);

private:
    double dt;
    int state_dim;
    Eigen::VectorXd state;
    Eigen::MatrixXd F, Q, R, P, H;
};

#endif // KALMANFILTER_H
