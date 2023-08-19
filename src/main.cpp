
#include "Arduino.h"
#include <typeinfo>
#include <math.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

// #include <Plant.h>
// #include <StarshotACS.h>

#include "DataLogging.hpp"

// static Plant plantObj;
// static StarshotACS starshotObj;

Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();

// int iteration = 0;
// double imu_delay = 0.25; // sec
// double RUN_TIME_HR = 100;

// /// PLANT PARAMETERS///
// double altitude_input = 400;
// double I_input[9] = {0.00195761450869, -5.836632382E-5, 2.27638093E-6,
//                      -5.836632382E-5, 0.00196346658902, 8.8920475E-7, 2.27638093E-6, 8.8920475E-7,
//                      0.00204697265884};

// double inclination_input = 0.90058989402907408; // 51.6 deg in rad
// double m_input = 1.3;                           // kg
// double q0_input[4] = {0.5, 0.5, -0.18301270189221924, 0.6830127018922193};

// double wx_input = 0.0;
// double wy_input = 0.0;
// double wz_input = 1.0;

// /// STARSHOT PARAMETERS///
// double A_input = 4.0E-5;
// double Id_input = 0.0021;
// double Kd_input = 0.0007935279615795299;
// double Kp_input = 5.2506307629097953E-10;
// double c_input = 0.004;
// double i_max_input = 0.25;
// double k_input = 13.5;
// double n_input = 500.0;
// double step_size_input = imu_delay; // sec
// /// INPUT DATA///

void setup()
{
  Serial.begin(9600);
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
  double IMUData[6] = {gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z};
  DataLog(IMUData, 6);
  Serial.printf(" % f, % f, % f, % f, % f, % f,  \n ", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  delay(200);
}
