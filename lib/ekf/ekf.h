#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <vector>

class EKF {
public:
    EKF();

    void Initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance, const Eigen::MatrixXd& process_noise_covariance, const Eigen::MatrixXd& magnetometer_noise_covariance, const Eigen::MatrixXd& gyroscope_noise_covariance);

    void Prediction(const Eigen::MatrixXd& A, double delta_t);
    void UpdateWithMagnetometer(const Eigen::VectorXd& magnetometer_measurement, const Eigen::MatrixXd& H_mag);
    void UpdateWithGyroscope(const Eigen::VectorXd& gyroscope_measurement, const Eigen::MatrixXd& H_gyro);

    Eigen::VectorXd GetState();
    Eigen::MatrixXd GetCovariance();

private:
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd R_mag;  // Magnetometer measurement noise covariance
    Eigen::MatrixXd R_gyro;  // Gyroscope measurement noise covariance

    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& B_body, const Eigen::VectorXd& angular_velocities);
};

#endif
