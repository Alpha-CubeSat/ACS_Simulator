#include "ekf.h"

EKF::EKF() {}

typedef Eigen::Matrix<double, 6, 1> state_type;

inline Eigen::Matrix<double, 6, 1> f(const Eigen::Matrix<double, 6, 1> &x)
{
    Eigen::Matrix<double, 6, 1> dxdt;
    dxdt << -x(2) * x(4) + x(1) * x(5),
            x(2) * x(3) - x(0) * x(5),
            -x(1) * x(3) + x(0) * x(4),
            -x(4) * x(5),
            x(3) * x(5),
            -x(3) * x(4) / 3;
    return dxdt;
}

inline Eigen::Matrix<double, 6, 1> rk4_step(const Eigen::Matrix<double, 6, 1> &x, double step_size)
{
    Eigen::Matrix<double, 6, 1> k1 = f(x);
    k1 *= step_size;

    Eigen::Matrix<double, 6, 1> half_k1 = 0.5 * k1;

    Eigen::Matrix<double, 6, 1> k2 = f(x + half_k1);
    k2 *= step_size;

    Eigen::Matrix<double, 6, 1> half_k2 = 0.5 * k2;

    Eigen::Matrix<double, 6, 1> k3 = f(x + half_k2);
    k3 *= step_size;

    Eigen::Matrix<double, 6, 1> k4 = f(x + k3);
    k4 *= step_size;

    // Reuse the result matrix to minimize memory copies
    Eigen::Matrix<double, 6, 1> result = x + (1.0 / 6.0) * step_size * (k1 + 2 * k2 + 2 * k3 + k4);

    return result;
}

void EKF::initialize(double delta_t, const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance, const Eigen::MatrixXd &process_noise_covariance, const Eigen::MatrixXd &Rd, const Eigen::MatrixXd &Hd)
{
    state = initial_state;
    Z = initial_state;
    covariance = initial_covariance;
    Q = process_noise_covariance;
    R_d = Rd;
    H_d = Hd;
    dt = delta_t;
}

void EKF::step()
{
    Eigen::MatrixXd J = CalculateJacobian();
    predict(J);
    correct();
}

void EKF::predict(const Eigen::MatrixXd &J_k_k)
{
    // Use RK4 for state prediction
    state = rk4_step(state, dt);
    covariance = J_k_k * covariance * J_k_k.transpose() + Q;
}

void EKF::correct()
{
    Eigen::MatrixXd K_k1 = covariance * H_d.transpose() * (H_d * covariance * H_d.transpose() + R_d).inverse();
    covariance -= K_k1 * (H_d * covariance * H_d.transpose() + R_d) * K_k1.transpose();
    state += K_k1 * (Z - H_d * state);
}

Eigen::MatrixXd EKF::CalculateJacobian()
{
    double Bx = Z(0), By = Z(1), Bz = Z(2), wx = Z(3), wy = Z(4), wz = Z(5);
    Eigen::MatrixXd J(6, 6);

    J << 0, wz, -wy, 0, -Bz, By,
         -wz, 0, wx, Bz, 0, -Bx,
         wy, -wx, 0, -By, Bx, 0,
         0, 0, 0, 0, -wz, -wy,
         0, 0, 0, wz, 0, wx,
         0, 0, 0, -wx / 3, -wy / 3, 0;

    return J;
}