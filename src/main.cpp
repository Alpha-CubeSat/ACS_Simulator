#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>

#include <SD.h>
#include <SPI.h>
#include "../lib/ekf/ekf.hpp"
#include <errno.h>
#include <sys/stat.h>

#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"

#include "DataLogging.hpp"


float readBatteryVoltage();

// Pins for inputs
#define AIN1 31
#define AIN2 32
#define PWMA 30
#define STBY 29
#define LED 13
#define file_name "test"

static StarshotACS starshotObj;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
EKF ekf; // Implement EKF

int current2PWM(float current) {
    if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) < 8.062)
        return 0;
    else if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) > 255)
        return 255;
    else
        return int(633.5 * pow(fabs(current), 0.6043) + 8.062);
}

// Starshot parameters
double A_input = 4.0E-5;
double Id_input = 0.0021;

double Kd_input = 0.0001;
double Kp_input = 5;

double c_input = 0.004;
double i_max_input = 0.25;
double k_input = 13.5;
double n_input = 500.0;
double step_size_input = 0.2; // sec

void ACSWrite(float current) {
    int PWM = current2PWM(current);
    if (PWM == 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    } else if (current < 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, PWM);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, PWM);
    }
}

void setup() {
    Serial.begin(9600);

    DataLogSetup(file_name);
    pinMode(LED, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);

    digitalWrite(LED, HIGH);

    starshotObj.initialize(step_size_input, A_input, Id_input, Kd_input, Kp_input, c_input, i_max_input, k_input, n_input);

    if (!imu.begin()) {
        while (1) {
            Serial.println("IMU initialization failed.");
            delay(100);
        }
    }

    Serial.println("Setting up LSM9DS1 9DOF IMU...");
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

     // Initialize EKF
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6);  // [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z]
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(6, 6) * 0.05;

    ekf.initialize(step_size_input, initial_state, initial_covariance,
                  process_noise, measurement_noise, 3.8f);

    // Load calibration files for the ekf
    ekf.loadCalibrationData(
        "../csv/softirons_x.csv",
        "../csv/softirons_y.csv",
        "../csv/softirons_z.csv"
    );

    delay(1000);
}

void loop() {
    unsigned long start = millis();

    sensors_event_t accel, mag, gyro, temp;
    imu.getEvent(&accel, &mag, &gyro, &temp);

    // Hard iron offsets
    float mag_hardiron_x = -4.9547;
    float mag_hardiron_y = 49.75155;
    float mag_hardiron_z = -13.8556;

    // Gyro hard iron offsets
    float gyro_hardiron_x = 0.0728;
    float gyro_hardiron_y = 0.0202;
    float gyro_hardiron_z = 0.0160;

    // Update voltage in EKF
    float voltage = readBatteryVoltage();
    ekf.setVoltage(voltage);

    // Prepare measurement vector
    Eigen::VectorXd measurement(6);
    measurement << mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
                  gyro.gyro.x, gyro.gyro.y, gyro.gyro.z;

    // Prepare control input vector
    Eigen::VectorXd control = Eigen::VectorXd::Zero(6);

    // EKF prediction step
    ekf.predict(control);

    // Get current PWM value for offset calculation
    double current_adjust = starshotObj.rtY.point[2] * 5.0;
    int PWM = current2PWM(current_adjust);

    // EKF update step with measurement and PWM
    ekf.update(measurement, PWM);

     // Get filtered state and offset
    Eigen::VectorXd filtered_state = ekf.getState();
    Eigen::VectorXd offset = ekf.getLatestOffset();

    // Apply hard iron offsets and assign filtered values to Starshot ACS
    starshotObj.rtU.Bfield_body[0] = filtered_state(0) - mag_hardiron_x;
    starshotObj.rtU.Bfield_body[1] = filtered_state(2) - mag_hardiron_z;
    starshotObj.rtU.Bfield_body[2] = -filtered_state(1) - mag_hardiron_y;

    // Apply gyro values with offset correction
    starshotObj.rtU.w[0] = filtered_state(3) - gyro_hardiron_x - offset(3);
    starshotObj.rtU.w[1] = filtered_state(5) - gyro_hardiron_z - offset(5);
    starshotObj.rtU.w[2] = -filtered_state(4) - gyro_hardiron_y - offset(4);
    starshotObj.step();

    // Apply ACS output
    ACSWrite(current_adjust);

    // Log data
    double IMUData[7] = {
        starshotObj.rtY.pt_error,
        current_adjust,
        (double)PWM,
        starshotObj.rtU.Bfield_body[0],
        starshotObj.rtU.Bfield_body[2],
        starshotObj.rtU.Bfield_body[1],
        voltage
    };
    DataLog(IMUData, 7, file_name);
    Serial.println("Data logged.");

    unsigned long end = millis();
    delay(100 - (end - start));

}

// Helper function to read battery voltage
float readBatteryVoltage() {
    int voltage_value_pin = 23;
    float voltage_ref = 3.3;
    int resolution = 1023;
    int r1 = 4700;
    int r2 = 10000;
    return analogRead(voltage_value_pin) * voltage_ref / resolution * (r1 + r2) / r2;
}