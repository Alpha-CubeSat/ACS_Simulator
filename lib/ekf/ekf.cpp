#include "ekf.h"
#include <Eigen/Dense>

EKF::EKF() {}

void EKF::Initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance, const Eigen::MatrixXd& process_noise_covariance, const Eigen::MatrixXd& magnetometer_noise_covariance, const Eigen::MatrixXd& gyroscope_noise_covariance) {
    state = initial_state;
    covariance = initial_covariance;
    Q = process_noise_covariance;
    R_mag = magnetometer_noise_covariance;
    R_gyro = gyroscope_noise_covariance;
}

void EKF::Prediction(const Eigen::MatrixXd& A, double delta_t) {
    Eigen::MatrixXd Ad = (A * delta_t).exp();
    state = Ad * state;
    covariance = Ad * covariance * Ad.transpose() + Q;
}

void EKF::UpdateWithMagnetometer(const Eigen::VectorXd& magnetometer_measurement, const Eigen::MatrixXd& H_mag) {
    Eigen::MatrixXd K_mag = covariance * H_mag.transpose() * (H_mag * covariance * H_mag.transpose() + R_mag).inverse(); // Kalman Gain Calculation
    state += K_mag * (magnetometer_measurement - H_mag * state);
    covariance = (Eigen::MatrixXd::Identity(state.size(), state.size()) - K_mag * H_mag) * covariance;
}

void EKF::UpdateWithGyroscope(const Eigen::VectorXd& gyroscope_measurement, const Eigen::MatrixXd& H_gyro) {
    Eigen::MatrixXd K_gyro = covariance * H_gyro.transpose() * (H_gyro * covariance * H_gyro.transpose() + R_gyro).inverse();
    state += K_gyro * (gyroscope_measurement - H_gyro * state);
    covariance = (Eigen::MatrixXd::Identity(state.size(), state.size()) - K_gyro * H_gyro) * covariance;
}

Eigen::VectorXd EKF::GetState() {
    return state;
}

Eigen::MatrixXd EKF::GetCovariance() {
    return covariance;
}

Eigen::MatrixXd EKF::CalculateJacobian(const Eigen::VectorXd& B_body, const Eigen::VectorXd& angular_velocities) {
    // Implement Jacobian calculation here just like Soumaryup's matlab code:
    double Bx = B_body(0);
    double By = B_body(1);
    double Bz = B_body(2);
    double wx = angular_velocities(0);
    double wy = angular_velocities(1);
    double wz = angular_velocities(2);

    Eigen::MatrixXd J(6, 6);
    
    J << 0, wz, -wy, 0, -Bz, By,
         -wz, 0, wx, Bz, 0, -Bx,
         wy, -wx, 0, -By, Bx, 0,
         0, 0, 0, 0, -wz, -wy,
         0, 0, 0, wz, 0, wx,
         0, 0, 0, -wx/3, -wy/3, 0;

    return J;
}

