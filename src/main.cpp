#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <ArduinoEigen.h>

#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"
#include "../lib/ekf/EKF.h"
#include "DataLogging.hpp"

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define AIN1 28
#define AIN2 27
#define PWMA 30
#define LED 13
#define file_name "test"

static StarshotACS starshotObj;
static EKF ekfObj;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();

int current2PWM(float current)
{
  if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) < 8.062)
    return 0;
  else if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) > 255)
    return 255;
  else
    return int(633.5 * pow(fabs(current), 0.6043) + 8.062);
}

// /// STARSHOT PARAMETERS///
double A_input = 4.0E-5;
double Id_input = 0.0021;

double Kd_input = 0.0001;
double Kp_input = 5;

double c_input = 0.004;
double i_max_input = 0.25;
double k_input = 13.5;
double n_input = 500.0;
double step_size_input = 0.2; // sec
// /// INPUT DATA///



void ACSWrite(float current)
{

  int PWM = current2PWM(current);

  if (PWM == 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }

  if (current < 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, PWM);
  }

  if (current > 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, PWM);
  }
}

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

  DataLogSetup(file_name);
  pinMode(LED, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  digitalWrite(LED, HIGH);
  starshotObj.initialize(step_size_input, A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);
  // ekfObj.initialize(step_size_input, initial_state, initial_cov, Q, Rd, Hd);
  // DataLogSetup();

  if (!imu.begin())
  {
    while (1)
    {
      Serial.println("wrong");
      delay(100);
    };
  }

  Serial.println("Setting up imu9DS1 9DOF");
  imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  imu.setupMag(imu.LSM9DS1_MAGGAIN_8GAUSS);
  imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  delay(1000);
}

void loop()
{
  sensors_event_t accel, mag, gyro, temp;
  imu.getEvent(&accel, &mag, &gyro, &temp);

  double time = millis();

  //Remap axis(rotate around x-axis by 90 deg)

  // ekfObj.Z(0) = mag.magnetic.x;
  // ekfObj.Z(1) = mag.magnetic.y;
  // ekfObj.Z(2) = mag.magnetic.z;

  // ekfObj.Z(3) = gyro.gyro.x;
  // ekfObj.Z(4) = gyro.gyro.y;
  // ekfObj.Z(5) = gyro.gyro.z;

  // ekfObj.step();

  // starshotObj.rtU.w[0] = ekfObj.state(3);
  // starshotObj.rtU.w[1] = ekfObj.state(5);
  // starshotObj.rtU.w[2] = -ekfObj.state(4);

  // starshotObj.rtU.Bfield_body[0] = ekfObj.state(0);
  // starshotObj.rtU.Bfield_body[1] = ekfObj.state(2);
  // starshotObj.rtU.Bfield_body[2] = -ekfObj.state(1);

  // starshotObj.rtU.w[0] = gyro.gyro.x;
  // starshotObj.rtU.w[1] = gyro.gyro.z;
  // starshotObj.rtU.w[2] = -gyro.gyro.y;

  // starshotObj.rtU.Bfield_body[0] = mag.magnetic.x;
  // starshotObj.rtU.Bfield_body[1] = mag.magnetic.z;
  // starshotObj.rtU.Bfield_body[2] = -mag.magnetic.y;
  //////
  // starshotObj.step();

  //test bench current adjust due to high B field
  double current_adjust = starshotObj.rtY.point[2] * 5.0;
  //ACSWrite(current_adjust);

  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.print(gyro.gyro.z);
  Serial.print(", ");
  Serial.print(mag.magnetic.x);
  Serial.print(", ");
  Serial.print(mag.magnetic.y);
  Serial.print(", ");
  Serial.print(mag.magnetic.z);

  double time_diff = millis()-time;
  Serial.print(", ");
  Serial.println(time_diff);
  // data
  // int PWM = current2PWM(current_adjust);
  //double IMUData[6] = {starshotObj.rtY.pt_error, current_adjust, PWM, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};
  //DataLog(IMUData, 6, file_name);
  delay(200);
}
