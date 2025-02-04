#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>

#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"

#include "DataLogging.hpp"

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins, 2.29.24 Note that some of the pins were re-arranged to make a cleaner setup
#define STBY 32
#define AIN1 31
#define AIN2 30
#define PWMA 29
#define LED 13

#define file_name "test"

static StarshotACS starshotObj;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();

int prev_pwm = 0; // For using previous pwm value for calculating the current softiron offsets
// unsigned long universal_start = 0.0; // For adding a delay in turning on the magnetorquer even with loop() running

/// HARD/SOFTIRON PARAMETERS ///
float mag_hardiron_x = -12.10905;
float mag_hardiron_y = 35.2236;
float mag_hardiron_z = -0.7527499999999989;

float gyro_hardiron_x = 0.07280736884261114;
float gyro_hardiron_y = 0.020224269122947534;
float gyro_hardiron_z = 0.016019223067681217;

float mag_softiron_coefs[7][9] = {{0.019462021688210852, -1.05E-07, 2.195769454371012E-08,
                                   -0.003319764, -8.19E-07, 4.5488393712589e-08,
                                   -0.116131841, 4.6269597985941555e-06, 1.0403272204973951e-07},
                                  {0.023756285063479793, -5.76E-07, -6.49E-09,
                                   -0.002081774, -2.12E-06, 4.233319947341387e-08,
                                   -0.115497365, 1.9072303973969682e-07, 3.897409095339711e-09},
                                  {0.021092495842071724, 6.463929548597455e-06, 4.684931543977051e-08,
                                   0.002149873, -1.19E-05, 1.095422849013733e-11,
                                   -0.124503588, -6.93E-06, 1.36182968591404e-07},
                                  {0.022327856, 7.939149696324287e-06, -1.73E-08,
                                   -0.001831145, 1.39666094626853e-05, -1.15E-08,
                                   -0.123188525, -1.27E-05, 3.9396863943664065e-08},
                                  {0.024360950836551362, -4.72E-06, -1.36E-08,
                                   -0.003801073, -1.87E-06, 5.4898640636709325e-08,
                                   -0.123800417, 8.112095184792033e-06, 3.884068275485318e-08},
                                  {0.01836786, -1.48E-06, 7.369282175053551e-08,
                                   -0.004570206, -2.74E-06, 4.7744513060828244e-08,
                                   -0.123778708, -5.42E-06, 2.1513728320978856e-08},
                                  {0.024306893790951856, -6.48E-06, -3.31E-09,
                                   7.214640626987035e-05, -2.71E-06, -2.24E-08,
                                   -0.13156585, 2.6434810337614613e-06, 4.67602511627662e-08}};

float pwmZ_ox = 0;
float pwmZ_oy = 0;
float pwmZ_oz = 0;

/// STARSHOT PARAMETERS///
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

int current2PWM(float current)
{
  if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) < 8.062)
    return 0;
  else if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) > 255)
    return 255;
  else
    return int(633.5 * pow(fabs(current), 0.6043) + 8.062);
}

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

void calc_softirons(int pwm_num, float voltage)
{
  voltage = round(voltage * 10) / 10;

  if (voltage < 3.6)
  {
    voltage = 3.6;
  }
  if (voltage > 4.2)
  {
    voltage = 4.2;
  }

  int j = 0;
  for (float i = 3.6; i <= 4.2; i += 1)
  {
    if (i != voltage)
    {
      i += 1;
      j += 1;
    }
  }

  float pwmZ_ox_1 = mag_softiron_coefs[j][0];
  float pwmZ_ox_2 = mag_softiron_coefs[j][1];
  float pwmZ_ox_3 = mag_softiron_coefs[j][2];
  float pwmZ_oy_1 = mag_softiron_coefs[j][3];
  float pwmZ_oy_2 = mag_softiron_coefs[j][4];
  float pwmZ_oy_3 = mag_softiron_coefs[j][5];
  float pwmZ_oz_1 = mag_softiron_coefs[j][6];
  float pwmZ_oz_2 = mag_softiron_coefs[j][7];
  float pwmZ_oz_3 = mag_softiron_coefs[j][8];

  pwmZ_ox = (pwmZ_ox_1 * pwm_num) + (pwmZ_ox_2 * pow(pwm_num, 2)) + (pwmZ_ox_3 * pow(pwm_num, 3));
  pwmZ_oy = (pwmZ_oy_1 * pwm_num) + (pwmZ_oy_2 * pow(pwm_num, 2)) + (pwmZ_oy_3 * pow(pwm_num, 3));
  pwmZ_oz = (pwmZ_oz_1 * pwm_num) + (pwmZ_oz_2 * pow(pwm_num, 2)) + (pwmZ_oz_3 * pow(pwm_num, 3));
}

void setup()
{
  Serial.begin(9600);

  DataLogSetup(file_name);
  pinMode(LED, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  digitalWrite(STBY, HIGH);
  digitalWrite(LED, HIGH);
  starshotObj.initialize(step_size_input, A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);

  analogWrite(PWMA, 0);

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
  imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS); // Applying 5V on the large coils will make the magnetometer max out at around 1000 uT
  imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  delay(1000);

  // universal_start = millis();
}

void loop()
{

  unsigned long start = millis();

  sensors_event_t accel, mag, gyro, temp;
  imu.getEvent(&accel, &mag, &gyro, &temp);

  // Getting battery voltage
  int voltage_value_pin = 23;
  float voltage_ref = 3.3;
  int resolution = 1023;
  int r1 = 4700;
  int r2 = 10000;

  float voltage = analogRead(voltage_value_pin) * voltage_ref / resolution * (r1 + r2) / r2;

  // Calibrating and assigning magnetometer values
  calc_softirons(prev_pwm, voltage);
  starshotObj.rtU.Bfield_body[0] = mag.magnetic.x - mag_hardiron_x - pwmZ_ox;
  starshotObj.rtU.Bfield_body[1] = mag.magnetic.y - mag_hardiron_z - pwmZ_oz;
  starshotObj.rtU.Bfield_body[2] = mag.magnetic.z - mag_hardiron_y - pwmZ_oy;

  // Calibrating and assigning gyro values
  starshotObj.rtU.w[0] = gyro.gyro.x - gyro_hardiron_x;
  starshotObj.rtU.w[1] = gyro.gyro.y - gyro_hardiron_z;
  starshotObj.rtU.w[2] = gyro.gyro.z - gyro_hardiron_y;

  starshotObj.step();

  // test bench current adjust due to high B field
  double current_adjust = starshotObj.rtY.point[2] * 5.0;
  ACSWrite(current_adjust);

  // data
  int PWM = current2PWM(current_adjust);
  prev_pwm = PWM; // will be used to calculate softiron offsets in the next loop, previous pwm values must be used for current loop.
  double IMUData[16] = {starshotObj.rtY.pt_error, current_adjust, PWM, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z, mag_hardiron_x, mag_hardiron_y, mag_hardiron_z, pwmZ_ox, pwmZ_oy, pwmZ_oz, gyro_hardiron_x, gyro_hardiron_y, gyro_hardiron_z, voltage};
  DataLog(IMUData, 16, file_name);
  Serial.println("data logged");

  unsigned long end = millis();
  delay(100 - (end - start));
}
