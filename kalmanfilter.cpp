#include "kalmanfilter.h"
#include <Eigen/Dense>
#include <iostream>


KalmanFilter::KalmanFilter(double dt, const Eigen::MatrixXd& process_noise_cov, const Eigen::MatrixXd& measurement_noise_cov,
                           const Eigen::MatrixXd& state_cov, const Eigen::VectorXd& initial_state_vector, int state_dim)
    : dt(dt), state_dim(state_dim), state(initial_state_vector) {
    F = Eigen::MatrixXd::Identity(state_dim, state_dim);
    F(0, 1) = dt;
    F(2, 3) = dt;
    F(4, 5) = dt;

    Q = process_noise_cov;
    R = measurement_noise_cov;
    P = state_cov;

    H = Eigen::MatrixXd::Zero(3, state_dim);
    H(0, 0) = 1;
    H(1, 2) = 1;
    H(2, 4) = 1;
}

Eigen::VectorXd KalmanFilter::predict(const Eigen::Vector3d& imu_position, bool imu_data_flag) {
    if (imu_data_flag) {
        state(1) += imu_position(0) * dt;
        state(3) += imu_position(1) * dt;
        state(5) += imu_position(2) * dt;

        state(0) += state(1) * dt;
        state(2) += state(3) * dt;
        state(4) += state(5) * dt;
    } else {
        state(0) = imu_position(0);
        state(2) = imu_position(1);
        state(4) = imu_position(2);
    }

    state = F * state;
    P = F * P * F.transpose() + Q;
    // for (int i=0;i<6;i++){

    // std::cout << "State of " <<i<<"is "<< state(i) << std::endl;

    // }
    return state;
}

Eigen::VectorXd KalmanFilter::update(const Eigen::Vector3d& gps_position) {
    Eigen::VectorXd z(3);
    z << gps_position(0), gps_position(1), gps_position(2);

    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    state = state + K * (z - H * state);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim, state_dim);
    P = (I - K * H) * P;

    return state;
}
