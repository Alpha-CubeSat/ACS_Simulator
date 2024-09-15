#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>

#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"

#include "DataLogging.hpp"

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins, 2.29.24 Note that some of the pins were re-arranged to make a cleaner setup
// #define AIN1 28
#define AIN1 37
// #define AIN2 27
#define AIN2 36
// #define PWMA 30
#define PWMA 35
#define LED 13
#define file_name "test19_sep2024"

static StarshotACS starshotObj;
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
  Serial.begin(9600);

  DataLogSetup(file_name);
  pinMode(LED, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  digitalWrite(LED, HIGH);
  starshotObj.initialize(step_size_input, A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);

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
  imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS); // Applying 5V on the large coils will make the magnetometer max out at around 1000 uT
  imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  delay(1000);
}

void loop()
{

  unsigned long start = millis();

  sensors_event_t accel, mag, gyro, temp;
  imu.getEvent(&accel, &mag, &gyro, &temp);

  int pwm_y = 0;

  // Remap axis(rotate around x-axis by 90 deg)

  // Calibrating and assigning magnetometer values
  float mag_hardiron_x = -4.9547000000000025;
  float mag_hardiron_y = 49.75155;
  float mag_hardiron_z = -13.855600000000003;

  float pwmY_ox_1 = 8.47256206e-04;
  float pwmY_ox_2 = -2.39011680e-07;
  float pwmY_ox_3 = -1.81875731e-09;
  float pwmY_oy_1 = -3.20921615e-02;
  float pwmY_oy_2 = -2.93994540e-07;
  float pwmY_oy_3 = 1.20748209e-08;
  float pwmY_oz_1 = -2.05710501e-03;
  float pwmY_oz_2 = -3.26514033e-07;
  float pwmY_oz_3 = -5.07562084e-09;

  // Change all naming to oop-flight-code naming
  float pwmY_ox = (pwmY_ox_1 * pwm_y) + (pwmY_ox_2 * pow(pwm_y, 2)) + (pwmY_ox_3 * pow(pwm_y, 3));
  float pwmY_oy = (pwmY_oy_1 * pwm_y) + (pwmY_oy_2 * pow(pwm_y, 2)) + (pwmY_oy_3 * pow(pwm_y, 3));
  float pwmY_oz = (pwmY_oz_1 * pwm_y) + (pwmY_oz_2 * pow(pwm_y, 2)) + (pwmY_oz_3 * pow(pwm_y, 3));

  starshotObj.rtU.Bfield_body[0] = mag.magnetic.x - mag_hardiron_x - pwmY_ox;
  starshotObj.rtU.Bfield_body[1] = mag.magnetic.z - mag_hardiron_z - pwmY_oz;
  starshotObj.rtU.Bfield_body[2] = -mag.magnetic.y - mag_hardiron_y - pwmY_oy;

  // Calibrating and assigning gyro values
  float gyro_hardiron_x = 0.07280736884261114;
  float gyro_hardiron_y = 0.020224269122947534;
  float gyro_hardiron_z = 0.016019223067681217;

  starshotObj.rtU.w[0] = gyro.gyro.x - gyro_hardiron_x;
  starshotObj.rtU.w[1] = gyro.gyro.z - gyro_hardiron_z;
  starshotObj.rtU.w[2] = -gyro.gyro.y - gyro_hardiron_y;

  // Getting battery voltage
  int voltage_value_pin = 32;
  float voltage_ref = 3.3;
  int resolution = 1023;
  int r1 = 4700;
  int r2 = 10000;

  float voltage = analogRead(voltage_value_pin) * voltage_ref * (r1 + r2) / resolution / r2;

  // starshotObj.rtU.w[0] = gyro.gyro.x;
  // starshotObj.rtU.w[1] = gyro.gyro.z;
  // starshotObj.rtU.w[2] = -gyro.gyro.y;

  // starshotObj.rtU.Bfield_body[0] = mag.magnetic.x;
  // starshotObj.rtU.Bfield_body[1] = mag.magnetic.z;
  // starshotObj.rtU.Bfield_body[2] = -mag.magnetic.y;
  starshotObj.step();

  // test bench current adjust due to high B field
  double current_adjust = starshotObj.rtY.point[2] * 5.0;
  ACSWrite(current_adjust);

  // data
  int PWM = current2PWM(current_adjust);
  pwm_y = PWM;
  double IMUData[7] = {starshotObj.rtY.pt_error, current_adjust, PWM, mag.magnetic.x, mag.magnetic.y, mag.magnetic.z, voltage};
  DataLog(IMUData, 7, file_name);
  Serial.println("data logged");

  unsigned long end = millis();
  delay(100 - (end - start));
}
