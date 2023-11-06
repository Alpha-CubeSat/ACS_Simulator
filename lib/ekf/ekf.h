#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <vector>

class EKF {
public:
    EKF();

    void initialize(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance, const Eigen::MatrixXd &process_noise_covariance, const Eigen::MatrixXd &noise_covariance, const Eigen::MatrixXd &Hd);
    void step();

    Eigen::VectorXd state;
    Eigen::VectorXd Z;
    Eigen::MatrixXd covariance;

private:

    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd R_d; // (measurement noise variance) Matrices
    Eigen::MatrixXd H_d;

    void predict(const Eigen::MatrixXd &J_k_k, double delta_t);
    void correct();
    Eigen::MatrixXd CalculateJacobian();
};