#include "ekf.h"

typedef Eigen::VectorXd state_type;
EKF::EKF() {}

Eigen::VectorXd f(const Eigen::VectorXd &x)
{
    Eigen::VectorXd dxdt(6);
    dxdt(0) = -x(2) * x(4) + x(1) * x(5);
    dxdt(1) = x(2) * x(3) - x(0) * x(5);
    dxdt(2) = -x(1) * x(3) + x(0) * x(4);
    dxdt(3) = -x(4) * x(5);
    dxdt(4) = x(3) * x(5);
    dxdt(5) = -x(3) * x(4) / 3;
    return dxdt;
}

Eigen::VectorXd rk4_step(const Eigen::VectorXd &x, double step_size)
{
    Eigen::VectorXd k1 = step_size * f(x);
    Eigen::VectorXd k2 = step_size * f(x + 0.5 * k1);
    Eigen::VectorXd k3 = step_size * f(x + 0.5 * k2);
    Eigen::VectorXd k4 = step_size * f(x + k3);
    return x + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
}

Eigen::VectorXd rk4(const Eigen::VectorXd &x_initial, double f_step_size, double t_start, double t_end)
{
    Eigen::VectorXd x = x_initial;
    for (double t = t_start; t <= t_end; t += f_step_size)
    {
        x = rk4_step(x, f_step_size);
    }
    return x;
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

Eigen::MatrixXd matrix_exp(const Eigen::MatrixXd &A, int order = 10)
{
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A.rows(), A.cols());
    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(A.rows(), A.cols());

    for (int i = 1; i <= order; ++i)
    {
        A_power = A_power * A / i;
        result += A_power;
    }

    return result;
}

void EKF::predict(const Eigen::MatrixXd &J_k_k)
{

    // Eigen::MatrixXd Ad = (J_k_k * dt).exp();

    Eigen::MatrixXd Ad = matrix_exp((J_k_k * dt));
    state = rk4(state, 0.001, 0.0, dt);
    covariance = Ad * covariance * Ad.transpose() + Q;
}

void EKF::correct()
{
    Eigen::MatrixXd K_k1 = covariance * H_d.transpose() * (H_d * covariance * H_d.transpose() + R_d).inverse();
    covariance = covariance - K_k1 * (H_d * covariance * H_d.transpose() + R_d) * K_k1.transpose();
    state = state + K_k1 * (Z - H_d * state);
}

Eigen::MatrixXd EKF::CalculateJacobian()
{
    // Implement Jacobian calculation here just like Soumaryup's matlab code:
    double Bx = Z(0);
    double By = Z(1);
    double Bz = Z(2);
    double wx = Z(3);
    double wy = Z(4);
    double wz = Z(5);

    Eigen::MatrixXd J(6, 6);

    J << 0, wz, -wy, 0, -Bz, By,
        -wz, 0, wx, Bz, 0, -Bx,
        wy, -wx, 0, -By, Bx, 0,
        0, 0, 0, 0, -wz, -wy,
        0, 0, 0, wz, 0, wx,
        0, 0, 0, -wx / 3, -wy / 3, 0;

    return J;
}
