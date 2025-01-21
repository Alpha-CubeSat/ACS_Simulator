#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <ArduinoEigen.h>
#include "CalibratedEKF.hpp"  // Include our new EKF class
#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"
#include "DataLogging.hpp"

// Pin definitions
#define AIN1 31
#define AIN2 32
#define PWMA 30
#define STBY 29
#define LED 13
#define FILE_NAME "test"
#define VOLTAGE_PIN 23

// Global objects
static StarshotACS starshotObj;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
CalibratedEKF ekf;

// Parameters
const double step_size = 0.2;  // seconds
const int DELAY_MS = 100;

// Convert current to PWM value
int current2PWM(float current) {
    if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) < 8.062)
        return 0;
    else if (int(633.5 * pow(fabs(current), 0.6043) + 8.062) > 255)
        return 255;
    else
        return int(633.5 * pow(fabs(current), 0.6043) + 8.062);
}

// Write current to ACS
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

float readBatteryVoltage() {
    const float voltage_ref = 3.3;
    const int resolution = 1023;
    const int r1 = 4700;
    const int r2 = 10000;
    
    return analogRead(VOLTAGE_PIN) * voltage_ref / resolution * (r1 + r2) / r2;
}

void setup() {
    Serial.begin(9600);
    DataLogSetup(FILE_NAME);

    // Pin setup
    pinMode(LED, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    digitalWrite(LED, HIGH);

    // Initialize Starshot
    starshotObj.initialize(step_size, 4.0E-5, 0.0021, 0.0001, 5, 0.004, 0.25, 13.5, 500.0);

    // Initialize IMU
    if (!imu.begin()) {
        while (1) {
            Serial.println("IMU initialization failed");
            delay(100);
        }
    }

    Serial.println("Initializing IMU and EKF...");Serial.println("Setting up LSM9DS1 9DOF IMU...");
    // Configure IMU
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

    // Initialize EKF
    Eigen::VectorXd initial_state(6);
    initial_state.setZero();

    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(6, 6) * 0.1;
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);  // Measurement matrix

    ekf.initialize(step_size, initial_state, initial_covariance, 
                  process_noise, measurement_noise, H, readBatteryVoltage());

    delay(1000);
}

void loop() {
    unsigned long start = millis();

    // Read sensor data
    sensors_event_t accel, mag, gyro, temp;
    imu.getEvent(&accel, &mag, &gyro, &temp);

    // Read battery voltage and update EKF calibration
    float voltage = readBatteryVoltage();
    ekf.updateVoltage(voltage);

    // Create measurement vector
    Eigen::VectorXd measurement(6);
    
    // Hard iron corrections
    float mag_x = mag.magnetic.x - (-4.9547000000000025);
    float mag_y = mag.magnetic.z - (-13.855600000000003);
    float mag_z = -mag.magnetic.y - (49.75155);

    float gyro_x = gyro.gyro.x - 0.07280736884261114;
    float gyro_y = gyro.gyro.z - 0.016019223067681217;
    float gyro_z = -gyro.gyro.y - 0.020224269122947534;

    // Update EKF measurement
    measurement << mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z;
    ekf.Z = measurement;

    // Perform EKF step
    ekf.step();

    // Get corrected state for control
    Eigen::VectorXd corrected_state = ekf.state;
    
    // Update Starshot with corrected measurements
    starshotObj.rtU.Bfield_body[0] = corrected_state(0);
    starshotObj.rtU.Bfield_body[1] = corrected_state(1);
    starshotObj.rtU.Bfield_body[2] = corrected_state(2);
    starshotObj.rtU.w[0] = corrected_state(3);
    starshotObj.rtU.w[1] = corrected_state(4);
    starshotObj.rtU.w[2] = corrected_state(5);

    // Run Starshot control step
    starshotObj.step();

    // Apply control output
    double current_adjust = starshotObj.rtY.point[2] * 5.0;
    ACSWrite(current_adjust);

    // Log data
    int PWM = current2PWM(current_adjust);
    double IMUData[7] = {
        starshotObj.rtY.pt_error,
        current_adjust,
        static_cast<double>(PWM),
        corrected_state(0),  // Corrected magnetic field measurements
        corrected_state(1),
        corrected_state(2),
        voltage
    };
    DataLog(IMUData, 7, FILE_NAME);

    // Maintain loop timing
    unsigned long end = millis();
    if (end - start < DELAY_MS) {
        delay(DELAY_MS - (end - start));
    }
}
