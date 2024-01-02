#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <ArduinoEigen.h>

#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"
#include "../lib/ACS_libs/ekf/ekf.h"
#include "DataLogging.hpp"


#define LED 13


static EKF ekfObj;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
int iter = 0;

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

  ekfObj.initialize(0.1, initial_state, initial_cov, Q, Rd, Hd);


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

  //Remap axis(rotate around x-axis by 90 deg) uT to T
  ekfObj.Z(0) = mag.magnetic.x;
  ekfObj.Z(1) = mag.magnetic.y ;
  ekfObj.Z(2) = mag.magnetic.z ;

  ekfObj.Z(3) = gyro.gyro.x;
  ekfObj.Z(4) = gyro.gyro.y;
  ekfObj.Z(5) = gyro.gyro.z;

  ekfObj.step();

  Serial.print(iter * 0.1);
  Serial.print(", ");

  // Print magnetometer readings
  Serial.print(mag.magnetic.x);
  Serial.print(", ");
  Serial.print(mag.magnetic.y);
  Serial.print(", ");
  Serial.print(mag.magnetic.z);
  Serial.print(", ");

  // Print gyroscope readings
  Serial.print(gyro.gyro.x);
  Serial.print(", ");
  Serial.print(gyro.gyro.y);
  Serial.print(", ");
  Serial.print(gyro.gyro.z);
  Serial.print(", ");

  // Print EKF states
  Serial.print(ekfObj.state(0));
  Serial.print(", ");
  Serial.print(ekfObj.state(1));
  Serial.print(", ");
  Serial.print(ekfObj.state(2));
  Serial.print(", ");
  Serial.print(ekfObj.state(3));
  Serial.print(", ");
  Serial.print(ekfObj.state(4));
  Serial.print(", ");
  Serial.print(ekfObj.state(5));

  // End the line
  Serial.println();
  double time_diff = millis()-time;

  //always taking 100 ms
  int delay_time = 100 - time_diff;

  if(delay_time<0){
    Serial.printf("too slow! \n");
    delay_time = 0;
  }

  delay(delay_time);
  iter++;
}
