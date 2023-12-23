#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"
#include "../lib/ACS_libs/Plant_ert_rtw/Plant.h"
#include "../lib/ACS_libs/ekf/ekf.h"


#define LED 13


static StarshotACS starshotObj;
static Plant plantObj;
static EKF ekfObj;

/// PLANT PARAMETERS///
double altitude_input = 400;
double I_input[9] = {0.00195761450869, -5.836632382E-5, 2.27638093E-6,
                     -5.836632382E-5, 0.00196346658902, 8.8920475E-7, 2.27638093E-6, 8.8920475E-7,
                     0.00204697265884};

double inclination_input = 0.90058989402907408; // 51.6 deg in rad
double m_input = 1.3;                           // kg
double q0_input[4] = {0.5, 0.5, -0.18301270189221924, 0.6830127018922193};

// ideally for pointing w xyz is [0,0,1], detumble test init w xyz could be, say, [ 0.008, -0.005, -0.0001] (total <5 deg/s)
double wx_input = 0.0;
double wy_input = 0.0;
double wz_input = 1.0;

// double wx_input = 0.008;
// double wy_input = -0.005;
// double wz_input = -0.0001;

/// STARSHOT PARAMETERS///
double A_input = 4.0E-5;
double Id_input = 0.196;
double Kd_input = 0.0007935279615795299;
double Kp_input = 5.2506307629097953E-10;
double c_input = 1.0E-5;
double i_max_input = 0.25;
double k_input = 13.5;
double n_input = 500.0;

double plant_step_size_input = 0.001; //s
double controller_step_size_input = 0.100; // s

void setup()
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
  Serial.begin(9600);

  digitalWrite(LED, HIGH);

  plantObj.initialize(plant_step_size_input, altitude_input, I_input, inclination_input, m_input, q0_input, wx_input, wy_input, wz_input);
  starshotObj.initialize(controller_step_size_input, A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);
  ekfObj.initialize(controller_step_size_input, initial_state, initial_cov, Q, Rd, Hd);



  delay(1000);
}

int iter = 0;
void loop()
{

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
  // uT to T
  ekfObj.Z(0) = plantObj.rtY.magneticfield[0] * 1000000.0;
  ekfObj.Z(1) = plantObj.rtY.magneticfield[1] * 1000000.0;
  ekfObj.Z(2) = plantObj.rtY.magneticfield[2] * 1000000.0;

  ekfObj.Z(3) = plantObj.rtY.angularvelocity[0];
  ekfObj.Z(4) = plantObj.rtY.angularvelocity[1];
  ekfObj.Z(5) = plantObj.rtY.angularvelocity[2];

  ekfObj.step();

  //uT to T
  starshotObj.rtU.Bfield_body[0] = ekfObj.state(0) / 1000000.0;
  starshotObj.rtU.Bfield_body[1] = ekfObj.state(1) / 1000000.0;
  starshotObj.rtU.Bfield_body[2] = ekfObj.state(2) / 1000000.0;

  starshotObj.rtU.w[0] = ekfObj.state(3);
  starshotObj.rtU.w[1] = ekfObj.state(4);
  starshotObj.rtU.w[2] = ekfObj.state(5);

  starshotObj.step();

  // IF STEP THE PLANT THE LAST, ALL DATA WILL BE NAN !!!
  // for (int i=0; i < (int)(controller_step_size_input / plant_step_size_input); i++)
  // {
  //   plantObj.rtU.current[0] = starshotObj.rtY.point[0];
  //   plantObj.rtU.current[1] = starshotObj.rtY.point[1];
  //   plantObj.rtU.current[2] = starshotObj.rtY.point[2];

  //   plantObj.step();
  // }
  
  Serial.print(iter * controller_step_size_input); // time stamp
  Serial.print(", ");
  Serial.print(starshotObj.rtY.pt_error); //deg
  Serial.print(", ");
  Serial.print(plantObj.rtU.current[0]);
  Serial.print(", ");
  Serial.print(plantObj.rtU.current[1]);
  Serial.print(", ");
  Serial.print(plantObj.rtU.current[2]);
  Serial.print(", ");
  Serial.print(plantObj.rtY.magneticfield[0] * 1000000.0); //uT
  Serial.print(", ");
  Serial.print(plantObj.rtY.magneticfield[1] * 1000000.0); // uT
  Serial.print(", ");
  Serial.print(plantObj.rtY.magneticfield[2] * 1000000.0); // uT
  Serial.print(", ");
  Serial.print(plantObj.rtY.angularvelocity[0]);
  Serial.print(", ");
  Serial.print(plantObj.rtY.angularvelocity[1]);
  Serial.print(", ");
  Serial.print(plantObj.rtY.angularvelocity[2]);
  Serial.println();
  iter++;
}
