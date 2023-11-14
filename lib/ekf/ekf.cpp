#include "ekf.h"

typedef Eigen::VectorXd state_type;

EKF::EKF() {}

Eigen::VectorXd f(const Eigen::VectorXd &x)
{
    Eigen::VectorXd dxdt(6);
    dxdt << -x(2) * x(4) + x(1) * x(5),
            x(2) * x(3) - x(0) * x(5),
            -x(1) * x(3) + x(0) * x(4),
            -x(4) * x(5),
            x(3) * x(5),
            -x(3) * x(4) / 3;
    return dxdt;
}

Eigen::VectorXd rk4_step(const Eigen::VectorXd &x, double step_size)
{
    Eigen::VectorXd k1 = f(x);
    k1 *= step_size;

    Eigen::VectorXd k2 = f(x + 0.5 * k1);
    k2 *= step_size;

    Eigen::VectorXd k3 = f(x + 0.5 * k2);
    k3 *= step_size;

    Eigen::VectorXd k4 = f(x + k3);
    k4 *= step_size;

    Eigen::VectorXd result = x + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

    return result;
}

Eigen::VectorXd rk4(const Eigen::VectorXd &x_initial, double f_step_size, double t_start, double t_end)
{
    Eigen::VectorXd x = x_initial;
    int num_steps = static_cast<int>((t_end - t_start) / f_step_size);

    for (int i = 0; i < num_steps; ++i)
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
    int n = A.rows();
    Eigen::MatrixXd result = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(n, n);

    for (int i = 1; i <= order; ++i)
    {
        A_power = A_power * A / static_cast<double>(i);
        result += A_power;

        if (i % 2 == 0)
        {
            A_power = A_power * A / static_cast<double>(i);
            result -= A_power;
        }
    }

    // Pade approximant. (For speedup: This is what SciPy uses.)
    Eigen::MatrixXd U = A * 0.5;
    Eigen::MatrixXd V = identity - U;
    Eigen::MatrixXd P = result * V + U;
    Eigen::MatrixXd Q = -result * U + V;

    result = P.inverse() * Q;

    return result;
}

void EKF::predict(const Eigen::MatrixXd &J_k_k)
{
    Eigen::MatrixXd Ad = matrix_exp(J_k_k * dt);
    state = rk4(state, 0.001, 0.0, dt);
    covariance = Ad * covariance * Ad.transpose() + Q;
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

