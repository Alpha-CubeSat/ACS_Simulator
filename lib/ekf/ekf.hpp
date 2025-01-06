#ifndef EKF_H
#define EKF_H

#pragma once

#include <ArduinoEigen.h>
#include <SD.h>
#include <vector>

class EKF {
public:
    EKF() = default;

    void initialize(double delta_t,
                   const Eigen::VectorXd& initial_state,
                   const Eigen::MatrixXd& initial_covariance,
                   const Eigen::MatrixXd& process_noise_covariance,
                   const Eigen::MatrixXd& measurement_noise_covariance,
                   float initial_voltage = 3.8f) {
        dt = delta_t;
        state = initial_state;
        covariance = initial_covariance;
        Q = process_noise_covariance;
        R = measurement_noise_covariance;
        voltage = initial_voltage;

        // Initialize measurement matrix H
        H = Eigen::MatrixXd::Identity(state.size(), state.size());

        // Initialize history vectors
        prediction_history.clear();
        actual_history.clear();
        offset_history.clear();
    }

    void loadCalibrationData(const std::string& xFile,
                           const std::string& yFile,
                           const std::string& zFile) {
        loadCSV(xFile, calibration_data_x);
        loadCSV(yFile, calibration_data_y);
        loadCSV(zFile, calibration_data_z);
    }

    void setVoltage(float new_voltage) {
        voltage = std::max(3.6f, std::min(new_voltage, 4.2f));
    }

    void predict(const Eigen::VectorXd& control_input) {
        // Store current state for history
        prediction_history.push_back(state);

        // Compute state transition matrix F using Jacobian
        Eigen::MatrixXd F = computeStateTransitionJacobian();

        // Predict state
        state = propagateState(state, control_input);

        // Update covariance
        covariance = F * covariance * F.transpose() + Q;
    }

    void update(const Eigen::VectorXd& measurement, float pwm_value) {
        // Store actual measurement for history
        actual_history.push_back(measurement);

        // Apply magnetic field corrections based on calibration data
        Eigen::VectorXd corrected_measurement = correctMeasurement(measurement, pwm_value);

        // Compute Kalman gain
        Eigen::MatrixXd K = covariance * H.transpose() *
                           (H * covariance * H.transpose() + R).inverse();

        // Update state
        Eigen::VectorXd innovation = corrected_measurement - H * state;
        state = state + K * innovation;

        // Update covariance
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state.size(), state.size());
        covariance = (I - K * H) * covariance;

        // Calculate and store offset
        if (!prediction_history.empty() && !actual_history.empty()) {
            Eigen::VectorXd offset = actual_history.back() - prediction_history.back();
            offset_history.push_back(offset);
        }
    }

    Eigen::VectorXd getState() const { return state; }
    Eigen::VectorXd getLatestOffset() const {
        return offset_history.empty() ? Eigen::VectorXd::Zero(state.size())
                                    : offset_history.back();
    }

private:
    double dt;
    float voltage;
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
    Eigen::MatrixXd Q;  // Process noise covariance
    Eigen::MatrixXd R;  // Measurement noise covariance
    Eigen::MatrixXd H;  // Measurement matrix

    std::vector<std::vector<float>> calibration_data_x;
    std::vector<std::vector<float>> calibration_data_y;
    std::vector<std::vector<float>> calibration_data_z;

    std::vector<Eigen::VectorXd> prediction_history;
    std::vector<Eigen::VectorXd> actual_history;
    std::vector<Eigen::VectorXd> offset_history;

    void loadCSV(const std::string& filePath, std::vector<std::vector<float>>& data) {
        File file = SD.open(filePath.c_str());
        if (!file) {
            //throw std::runtime_error("Failed to open file: " + filePath);
            return;
        }
        data.clear(); // Ensure vector is empty before loading new data
        while (file.available()) {
            String line = file.readStringUntil('\n');
            if (line.length() == 0) continue; // Skip empty lines
            std::vector<float> row;
            char* end;
            const char* cLine = line.c_str();
            while (*cLine) {
                float value = strtof(cLine, &end);
                if (cLine == end) break;
                row.push_back(value);
                cLine = end;
                while (*cLine == ',' || *cLine == ' ') cLine++; // Skip delimiter and whitespace
            }
            if (!row.empty()) {
                data.push_back(row);
            }
        }
        file.close();
    }

    Eigen::MatrixXd computeStateTransitionJacobian() {
        // Compute Jacobian of state transition function
        Eigen::MatrixXd transition_matrix = Eigen::MatrixXd::Identity(state.size(), state.size());

        // Add time-dependent terms
        const int half_size = state.size() / 2;
        for (int idx = 0; idx < half_size; ++idx) {
            transition_matrix(idx, idx + half_size) = dt;
        }

        return transition_matrix;
    }

    Eigen::VectorXd propagateState(const Eigen::VectorXd& current_state,
                                 const Eigen::VectorXd& control_input) {
        Eigen::VectorXd new_state = current_state;

        // Simple linear prediction as example
        for (int i = 0; i < current_state.size() / 2; ++i) {
            new_state(i) += current_state(i + current_state.size() / 2) * dt;
            new_state(i + current_state.size() / 2) += control_input(i) * dt;
        }

        return new_state;
    }

    Eigen::VectorXd correctMeasurement(const Eigen::VectorXd& measurement, float pwm) {
        // Get voltage index for calibration lookup
        int voltage_idx = static_cast<int>((voltage - 3.6f) * 10.0f);
        voltage_idx = std::max(0, std::min(voltage_idx, 6));

        // Calculate offsets for each axis
        std::vector<float> offsets = calculateOffsets(pwm, voltage_idx);

        Eigen::VectorXd corrected = measurement;
        // Apply offsets
        for (int i = 0; i < 3 && i < measurement.size(); ++i) {
            corrected(i) -= offsets[i];
        }

        return corrected;
    }

    std::vector<float> calculateOffsets(float pwm, int voltage_idx) {
        std::vector<float> offsets(3);

        // Calculate for X axis
        if (!calibration_data_x.empty()) {
            offsets[0] = calibration_data_x[0][voltage_idx] * pwm +
                        calibration_data_x[1][voltage_idx] * std::pow(pwm, 2) +
                        calibration_data_x[2][voltage_idx] * std::pow(pwm, 3);
        }

        // Calculate for Y axis
        if (!calibration_data_y.empty()) {
            offsets[1] = calibration_data_y[0][voltage_idx] * pwm +
                        calibration_data_y[1][voltage_idx] * std::pow(pwm, 2) +
                        calibration_data_y[2][voltage_idx] * std::pow(pwm, 3);
        }

        // Calculate for Z axis
        if (!calibration_data_z.empty()) {
            offsets[2] = calibration_data_z[0][voltage_idx] * pwm +
                        calibration_data_z[1][voltage_idx] * std::pow(pwm, 2) +
                        calibration_data_z[2][voltage_idx] * std::pow(pwm, 3);
        }

        return offsets;
    }
};

#endif