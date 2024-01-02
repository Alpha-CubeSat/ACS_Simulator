#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include "../lib/ACS_libs/Plant_ert_rtw/Plant.h"
#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"
#include "../lib/ACS_libs/ekf/ekf.h"



static Plant plantObj;
static StarshotACS starshotObj;
static EKF ekfObj;

int iteration = 0;
double RUN_TIME_HR =  24*2;

/// PLANT PARAMETERS///
double altitude_input = 400;
double I_input[9] = {0.00195761450869, -5.836632382E-5, 2.27638093E-6,
                     -5.836632382E-5, 0.00196346658902, 8.8920475E-7,
                     2.27638093E-6, 8.8920475E-7, 0.00204697265884};

// double I_input[9] = {0.00195761450869, 0.0, 0.0,
//                      0.0, 0.00196346658902, 0.0,
//                      0.0, 0.0, 0.00204697265884};

double inclination_input = 0.90058989402907408; // 51.6 deg in rad
double m_input = 1.3;                           // kg
double q0_input[4] = {0.5, 0.5, -0.18301270189221924, 0.6830127018922193};


double wx_input = 0.13;
double wy_input = 0.02;
double wz_input = 4.26;

// double wx_input = 0.05;
// double wy_input = -0.04;
// double wz_input = 0.01;

/// STARSHOT PARAMETERS///
double A_input = 4.0E-5;
double Id_input = 0.196;
double Kd_input = 0.0007935279615795299;
double Kp_input = 5.2506307629097953E-10;
double c_input = 1.0E-5;
double i_max_input = 0.25;
double k_input = 13.5;
double n_input = 500.0;

double plant_step_size_input = 0.001;      // s
double controller_step_size_input = 0.100; // s
int main()

{
  /// EKF PARAMETERS///
  Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd initial_cov = Eigen::MatrixXd::Zero(6, 6);
  // Q (process noise covariance) Matrix
  Eigen::MatrixXd Q = 0.02 * Eigen::MatrixXd::Identity(6, 6);
  Q.diagonal() << 0.008, 0.07, 0.005, 0.1, 0.1, 0.1;
  // Rd (measurement noise variance) Matrices
  Eigen::MatrixXd Rd(6, 6);
  Rd << 2.02559220e-01, 5.17515015e-03, -3.16669361e-02, -1.76503506e-04, -3.74891174e-05, -7.75657503e-05,
      5.17515015e-03, 1.55389381e-01, 1.07780468e-02, -2.90511952e-05, -8.02931174e-06, -1.26277622e-05,
      -3.16669361e-02, 1.07780468e-02, 3.93162684e-01, 9.29630074e-05, 1.22496815e-05, 5.67092127e-05,
      -1.76503506e-04, -2.90511952e-05, 9.29630074e-05, 1.80161545e-05, -2.27002599e-09, -6.07376965e-07,
      -3.74891174e-05, -8.02931174e-06, 1.22496815e-05, -2.27002599e-09, 6.70144060e-06, 2.97298687e-08,
      -7.75657503e-05, -1.26277622e-05, 5.67092127e-05, -6.07376965e-07, 2.97298687e-08, 8.52192033e-06;
  // Hd
  Eigen::MatrixXd Hd = Eigen::MatrixXd::Identity(6, 6);
  ////////////////////////////////////////////////////////////////////////////////
  std::ofstream outfile;
  outfile.open("output/test.txt");
  if (!outfile.is_open())
  { // check if file opened successfully
    return -1;
  }

  plantObj.initialize(plant_step_size_input, altitude_input, I_input, inclination_input, m_input, q0_input, wx_input, wy_input, wz_input);
  starshotObj.initialize(controller_step_size_input, A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);
  ekfObj.initialize(controller_step_size_input, initial_state, initial_cov, Q, Rd, Hd);


  std::cout << "TOTAL SIMULATION TIME: " << RUN_TIME_HR << " hours"
            << "\n ";

  while ((iteration * controller_step_size_input / 3600.0) < RUN_TIME_HR)
  {
    // step imu_deley
    for (int i = 0; i < (int)(controller_step_size_input / plant_step_size_input); i++)
    {

      // plantObj.rtU.current[0] = starshotObj.rtY.detumble[0];
      // plantObj.rtU.current[1] = starshotObj.rtY.detumble[1];
      // plantObj.rtU.current[2] = starshotObj.rtY.detumble[2];

      plantObj.rtU.current[0] = starshotObj.rtY.point[0];
      plantObj.rtU.current[1] = starshotObj.rtY.point[1];
      plantObj.rtU.current[2] = starshotObj.rtY.point[2];

      plantObj.step();
    }

    ///////////////////////////////////////////////////////////////////////////////

    // T to uT
    ekfObj.Z(0) = plantObj.rtY.magneticfield[0] * 1000000.0;
    ekfObj.Z(1) = plantObj.rtY.magneticfield[1] * 1000000.0;
    ekfObj.Z(2) = plantObj.rtY.magneticfield[2] * 1000000.0;

    ekfObj.Z(3) = plantObj.rtY.angularvelocity[0];
    ekfObj.Z(4) = plantObj.rtY.angularvelocity[1];
    ekfObj.Z(5) = plantObj.rtY.angularvelocity[2];


    ekfObj.step();


    starshotObj.rtU.Bfield_body[0] = ekfObj.state(0) / 1000000.0;
    starshotObj.rtU.Bfield_body[1] = ekfObj.state(1) / 1000000.0;
    starshotObj.rtU.Bfield_body[2] = ekfObj.state(2) / 1000000.0;

    starshotObj.rtU.w[0] = ekfObj.state(3);
    starshotObj.rtU.w[1] = ekfObj.state(4);
    starshotObj.rtU.w[2] = ekfObj.state(5);

    starshotObj.step();

    // std::cout << "time(s): " << (iteration * controller_step_size_input);
    // std::cout << "\n ";
    // std::cout << plantObj.rtY.magneticfield[0] << ", " << plantObj.rtY.magneticfield[1] << ", " << plantObj.rtY.magneticfield[2] << ", " << plantObj.rtY.angularvelocity[0] << ", " << plantObj.rtY.angularvelocity[1] << ", " << plantObj.rtY.angularvelocity[2];
    // std::cout << "\n ";
    // std::cout << ekfObj.state(0) / 1000000.0 << ", " << ekfObj.state(1) / 1000000.0 << ", " << ekfObj.state(2) / 1000000.0 << ", " << ekfObj.state(3) << ", " << ekfObj.state(4) << ", " << ekfObj.state(5);
    // std::cout << "\n ";

    outfile << (iteration * controller_step_size_input) << ", " << plantObj.rtY.magneticfield[0] << ", " << plantObj.rtY.magneticfield[1] << ", " << plantObj.rtY.magneticfield[2] << ", " << plantObj.rtY.angularvelocity[0] << ", " << plantObj.rtY.angularvelocity[1] << ", " << plantObj.rtY.angularvelocity[2] << ", " << ekfObj.state(0) / 1000000.0 << ", " << ekfObj.state(1) / 1000000.0 << ", " << ekfObj.state(2) / 1000000.0 << ", " << ekfObj.state(3) << ", " << ekfObj.state(4) << ", " << ekfObj.state(5) << ", "<< starshotObj.rtY.point[0] << ", " << starshotObj.rtY.point[1] << ", " << starshotObj.rtY.point[2] << ", "<<starshotObj.rtY.pt_error << std::endl;
    ;
    // print to console/ write to file
    if ((int)(iteration * 0.001) % 1 == 0)
    {

      std::cout << "time(s): " << (iteration * controller_step_size_input);
      // std::cout << ", pointing error(deg) : " << starshotObj.rtY.pt_error;
      std::cout << ", current(mA): [";

      for (int i = 0; i < 3; i++)
      {
        std::cout << starshotObj.rtY.detumble[i] * 1000.0;
        if (i < 2)
        {
          std::cout << ", ";
        }
        else
        {
          std::cout << "]";
        }
      }
      std::cout << ", mag: [";
      for (int i = 0; i < 3; i++)
      {
        std::cout << plantObj.rtY.magneticfield[i];
        if (i < 2)
        {
          std::cout << ", ";
        }
        else
        {
          std::cout << "]";
        }
      }
      std::cout << ", w: [";
      for (int i = 0; i < 3; i++)
      {
        std::cout << plantObj.rtY.angularvelocity[i];
        if (i < 2)
        {
          std::cout << ", ";
        }
        else
        {
          std::cout << "]";
        }
      }

      std::cout
          << "\n ";

    //   outfile << (iteration * controller_step_size_input) << ", " << starshotObj.rtY.point[0] * 1000.0 << ", " << starshotObj.rtY.point[1] * 1000.0 << ", " << starshotObj.rtY.point[2] * 1000.0 << ", " << plantObj.rtY.angularvelocity[0] << ", " << plantObj.rtY.angularvelocity[1] << ", " << plantObj.rtY.angularvelocity[2] << ", " << starshotObj.rtY.pt_error << std::endl;
     }

    iteration++;
  }

  outfile.close();
  std::cout << "\n"
            << "done."
            << "\n";
  return 0;
}
