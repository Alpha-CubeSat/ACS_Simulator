#include "ekf.h"

EKF::EKF() {}

void EKF::initialize(const Eigen::VectorXd &initial_state, const Eigen::MatrixXd &initial_covariance, const Eigen::MatrixXd &process_noise_covariance, const Eigen::MatrixXd &Rd, const Eigen::MatrixXd &Hd)
{
    state = initial_state;
    Z = initial_state;

    covariance = initial_covariance;
    Q = process_noise_covariance;
    R_d = Rd;
    H_d = Hd;
}

void EKF::step()
{
    Eigen::MatrixXd J = CalculateJacobian();

    //step size
    predict(J, 0.2);
    correct();

}


typedef Eigen::VectorXd state_type;
struct SystemDynamics
{
    void operator()(const state_type &x, state_type &dxdt, const double /* t */)
    {
        dxdt[0] = -x[2] * x[4] + x[1] * x[5];
        dxdt[1] = x[2] * x[3] - x[0] * x[5];
        dxdt[2] = -x[1] * x[3] + x[0] * x[4];
        dxdt[3] = -x[4] * x[5];
        dxdt[4] = x[3] * x[5];
        dxdt[5] = -x[3] * x[4] / 3;
    }
    
};

void EKF::predict(const Eigen::MatrixXd &J_k_k,double delta_t)
{

    using namespace boost::numeric::odeint;
    state_type X_k1_k(6);
    SystemDynamics system_dynamics;
    double t_start = 0.0;
    double t_end = delta_t;
    double dt = 0.001;

    integrate(system_dynamics, state, t_start, t_end, dt, [&X_k1_k](const state_type &x, const double /* t */) { X_k1_k = x; });


    Eigen::MatrixXd Ad = (J_k_k * delta_t).exp();
    state = X_k1_k;
    covariance = Ad * covariance * Ad.transpose() + Q;
}

void EKF::correct()
{
    Eigen::MatrixXd K_k1 = covariance * H_d.transpose() * (H_d * covariance * H_d.transpose() + R_d).inverse();
    covariance = covariance - K_k1 * (H_d * covariance * H_d.transpose() + R_d) * K_k1.transpose();
    state = state + K_k1 * (Z - H_d * state);
}


Eigen::MatrixXd EKF::CalculateJacobian() {
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
         0, 0, 0, -wx/3, -wy/3, 0;

    return J;
}

