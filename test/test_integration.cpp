#include <unity.h>

#ifndef EMBEDDED_BUILD
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_LSM9DS1.h>
#else
#include "../NativeMocks/MockArduino.h"
#include "../NativeMocks/MockWire.h"
#include "../NativeMocks/MockSPI.h"
#include "../NativeMocks/MockSD.h"
#include "../NativeMocks/MockAdafruit_LSM9DS1.h"
#endif
#include <ekf.hpp>
#include "../lib/ACS_libs/StarshotACS_ert_rtw/StarshotACS.h"

// Hardware objects
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();
StarshotACS starshotObj;
EKF ekf;

// Starshot ACS parameters
const double A_input = 4.0E-5;
const double Id_input = 0.0021;
const double Kd_input = 0.0001;
const double Kp_input = 5;
const double c_input = 0.004;
const double i_max_input = 0.25;
const double k_input = 13.5;
const double n_input = 500.0;
const double step_size_input = 0.2;

// Test setup
void setUp(void) {
    Wire.begin();
    SPI.begin();

    // Initialize IMU
    TEST_ASSERT_TRUE(imu.begin());
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

    // Initialize ACS
    starshotObj.initialize(step_size_input, A_input, Id_input, Kd_input, Kp_input, 
                           c_input, i_max_input, k_input, n_input);

    // Initialize EKF
    Eigen::VectorXd initial_state = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd initial_covariance = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(6, 6) * 0.01;
    Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(6, 6) * 0.05;

    ekf.initialize(step_size_input, initial_state, initial_covariance, 
                   process_noise, measurement_noise, 3.8f);
}

void tearDown(void) {
    Wire.end();
    SPI.end();
}

// IMU Tests
void test_imu_initialization(void) {
    TEST_ASSERT_TRUE(imu.begin());
}

void test_imu_readings(void) {
    sensors_event_t accel, mag, gyro, temp;
    imu.getEvent(&accel, &mag, &gyro, &temp);

    // Check if readings are within reasonable ranges
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, mag.magnetic.x);
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, mag.magnetic.y);
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, mag.magnetic.z);

    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, gyro.gyro.x);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, gyro.gyro.y);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, gyro.gyro.z);
}

// SD Card Tests
void test_sd_card(void) {
    TEST_ASSERT_TRUE(SD.begin(BUILTIN_SDCARD));

    File testFile = SD.open("test.txt", FILE_WRITE);
    TEST_ASSERT_NOT_NULL(testFile);
    if (testFile) {
        testFile.println("Test data");
        testFile.close();
    }

    testFile = SD.open("test.txt");
    TEST_ASSERT_NOT_NULL(testFile);
    if (testFile) {
        char buffer[10];
        size_t bytesRead = testFile.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
        buffer[bytesRead] = '\0';
        TEST_ASSERT_EQUAL_STRING("Test data", buffer);
        testFile.close();
    }

    SD.remove("test.txt");
}

// EKF Tests
void test_ekf_prediction(void) {
    sensors_event_t accel, mag, gyro, temp;
    imu.getEvent(&accel, &mag, &gyro, &temp);

    Eigen::VectorXd initial_state = ekf.getState();
    ekf.predict(Eigen::VectorXd::Zero(6));

    Eigen::VectorXd new_state = ekf.getState();
    TEST_ASSERT_NOT_EQUAL(initial_state(0), new_state(0));
}

void test_full_integration(void) {
    sensors_event_t accel, mag, gyro, temp;
    imu.getEvent(&accel, &mag, &gyro, &temp);

    Eigen::VectorXd measurement(6);
    measurement << mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
                   gyro.gyro.x, gyro.gyro.y, gyro.gyro.z;

    ekf.predict(Eigen::VectorXd::Zero(6));
    ekf.update(measurement, 128);

    Eigen::VectorXd state = ekf.getState();
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, static_cast<float>(state(0)));
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, static_cast<float>(state(1)));
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 0.0f, static_cast<float>(state(2)));
}