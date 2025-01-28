#ifndef CALIBRATED_EKF_HPP
#define CALIBRATED_EKF_HPP

#pragma once

#include "constants.hpp"
#include "ekf.h"
#include <ArduinoEigen.h>
#include <SD.h>
#include <map>


typedef std::array<std::array<float, 3>, 7> OffsetArray;

using namespace constants::imu;
// Error codes
enum class EKFError {
  OK = 0,
  VOLTAGE_OUT_OF_BOUNDS = 1,
  INSUFFICIENT_CALIBRATION_DATA = 2,
  INTERPOLATION_ERROR = 3,
  OFFSET_CALCULATION_ERROR = 4,
  STATE_BOUNDS_EXCEEDED = 5,
  INNOVATION_TOO_LARGE = 6
};

class CalibratedEKF : public EKF {
public:
  CalibratedEKF()
      : state_covariance(Eigen::MatrixXd::Identity(6, 6) * 0.1),
        current_voltage(3.6),
        last_error(EKFError::OK), x_offsets{0.0f, 0.0f, 0.0f},
        y_offsets{0.0f, 0.0f, 0.0f}, z_offsets{0.0f, 0.0f, 0.0f} {
    // Initialize the voltage coefficients map with the provided calibration
    // data
  }

private:
  std::map<float, VoltageCoefficients> voltage_coefficients;
  // Constants for voltage lookup
  static constexpr size_t VOLTAGE_LEVELS = 7; // 3.6V to 4.2V
  static constexpr float BASE_VOLTAGE = 3.6f;
  static constexpr float MAX_VOLTAGE = 4.2f;
  static constexpr float VOLTAGE_STEP = 0.1f;
  Eigen::MatrixXd state_covariance;
  float last_innovation_magnitude;
  float last_prediction_error;
  Eigen::VectorXd last_valid_state;
  // Class member variables
  float current_voltage;
  EKFError last_error;
  // Pre-allocated arrays for offset storage
  std::array<float, 3> x_offsets;
  std::array<float, 3> y_offsets;
  std::array<float, 3> z_offsets;

  // Efficient voltage index calculation
  static constexpr size_t getVoltageIndex(float voltage) {
    if (voltage <= BASE_VOLTAGE)
      return 0;
    if (voltage >= BASE_VOLTAGE + (VOLTAGE_LEVELS - 1) * VOLTAGE_STEP) {
      return VOLTAGE_LEVELS - 1;
    }
    return static_cast<size_t>((voltage - BASE_VOLTAGE) / VOLTAGE_STEP);
  }

  // Get interpolated offsets for a given voltage and axis
  // Returns interpolated values directly in the provided array
  bool getInterpolatedOffsets(
      double voltage, PWMCoefficients &coeffs,
      const std::function<const PWMCoefficients &(const VoltageCoefficients &)>
          &coeffSelector) {
    // Validate voltage range
    if (voltage < 3.6 || voltage > 4.2) {
      return false;
    }

    // Find the surrounding voltage levels
    auto upper_it =
        voltage_coefficients.upper_bound(static_cast<float>(voltage));
    if (upper_it == voltage_coefficients.begin() ||
        upper_it == voltage_coefficients.end()) {
      return false;
    }

    auto lower_it = std::prev(upper_it);

    // Calculate interpolation factor
    float lower_voltage = lower_it->first;
    float upper_voltage = upper_it->first;
    float factor = static_cast<float>((voltage - lower_voltage) /
                                      (upper_voltage - lower_voltage));

    // Get coefficients for both voltage levels
    const PWMCoefficients &lower_coeffs = coeffSelector(lower_it->second);
    const PWMCoefficients &upper_coeffs = coeffSelector(upper_it->second);

    // Interpolate coefficients
    coeffs.coeff_1 = lower_coeffs.coeff_1 +
                     factor * (upper_coeffs.coeff_1 - lower_coeffs.coeff_1);
    coeffs.coeff_2 = lower_coeffs.coeff_2 +
                     factor * (upper_coeffs.coeff_2 - lower_coeffs.coeff_2);
    coeffs.coeff_3 = lower_coeffs.coeff_3 +
                     factor * (upper_coeffs.coeff_3 - lower_coeffs.coeff_3);

    return true;
  }

  bool calculatePWMValues(double voltage, float pwm_x, float pwm_y, float pwm_z,
                          float &pwmX_ox, float &pwmX_oy, float &pwmX_oz,
                          float &pwmY_ox, float &pwmY_oy, float &pwmY_oz,
                          float &pwmZ_ox, float &pwmZ_oy, float &pwmZ_oz) {
    PWMCoefficients x_ox_coeffs, x_oy_coeffs, x_oz_coeffs;
    PWMCoefficients y_ox_coeffs, y_oy_coeffs, y_oz_coeffs;
    PWMCoefficients z_ox_coeffs, z_oy_coeffs, z_oz_coeffs;

    bool success = true;

    // Get interpolated coefficients for PWM X
    success &= getInterpolatedOffsets(
        voltage, x_ox_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmX.ox;
        });
    success &= getInterpolatedOffsets(
        voltage, x_oy_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmX.oy;
        });
    success &= getInterpolatedOffsets(
        voltage, x_oz_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmX.oz;
        });

    // Get interpolated coefficients for PWM Y
    success &= getInterpolatedOffsets(
        voltage, y_ox_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmY.ox;
        });
    success &= getInterpolatedOffsets(
        voltage, y_oy_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmY.oy;
        });
    success &= getInterpolatedOffsets(
        voltage, y_oz_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmY.oz;
        });

    // Get interpolated coefficients for PWM Z
    success &= getInterpolatedOffsets(
        voltage, z_ox_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmZ.ox;
        });
    success &= getInterpolatedOffsets(
        voltage, z_oy_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmZ.oy;
        });
    success &= getInterpolatedOffsets(
        voltage, z_oz_coeffs,
        [](const VoltageCoefficients &vc) -> const PWMCoefficients & {
          return vc.pwmZ.oz;
        });

    if (!success) {
      last_error = EKFError::INTERPOLATION_ERROR;
      return false;
    }

    // Calculate PWM X values using interpolated coefficients
    pwmX_ox = x_ox_coeffs.coeff_1 * pwm_x +
              x_ox_coeffs.coeff_2 * pow(pwm_x, 2) +
              x_ox_coeffs.coeff_3 * pow(pwm_x, 3);
    pwmX_oy = x_oy_coeffs.coeff_1 * pwm_x +
              x_oy_coeffs.coeff_2 * pow(pwm_x, 2) +
              x_oy_coeffs.coeff_3 * pow(pwm_x, 3);
    pwmX_oz = x_oz_coeffs.coeff_1 * pwm_x +
              x_oz_coeffs.coeff_2 * pow(pwm_x, 2) +
              x_oz_coeffs.coeff_3 * pow(pwm_x, 3);

    // Calculate PWM Y values using interpolated coefficients
    pwmY_ox = y_ox_coeffs.coeff_1 * pwm_y +
              y_ox_coeffs.coeff_2 * pow(pwm_y, 2) +
              y_ox_coeffs.coeff_3 * pow(pwm_y, 3);
    pwmY_oy = y_oy_coeffs.coeff_1 * pwm_y +
              y_oy_coeffs.coeff_2 * pow(pwm_y, 2) +
              y_oy_coeffs.coeff_3 * pow(pwm_y, 3);
    pwmY_oz = y_oz_coeffs.coeff_1 * pwm_y +
              y_oz_coeffs.coeff_2 * pow(pwm_y, 2) +
              y_oz_coeffs.coeff_3 * pow(pwm_y, 3);

    // Calculate PWM Z values using interpolated coefficients
    pwmZ_ox = z_ox_coeffs.coeff_1 * pwm_z +
              z_ox_coeffs.coeff_2 * pow(pwm_z, 2) +
              z_ox_coeffs.coeff_3 * pow(pwm_z, 3);
    pwmZ_oy = z_oy_coeffs.coeff_1 * pwm_z +
              z_oy_coeffs.coeff_2 * pow(pwm_z, 2) +
              z_oy_coeffs.coeff_3 * pow(pwm_z, 3);
    pwmZ_oz = z_oz_coeffs.coeff_1 * pwm_z +
              z_oz_coeffs.coeff_2 * pow(pwm_z, 2) +
              z_oz_coeffs.coeff_3 * pow(pwm_z, 3);

    last_error = EKFError::OK;
    return true;
  }

public:
  EKFError getLastError() const { return last_error; }

  double getLastInnovationMagnitude() const {
    return last_innovation_magnitude;
  }

  double getLastPredictionError() const { return last_prediction_error; }

  void initialize(double delta_t, const Eigen::VectorXd &initial_state,
                  const Eigen::MatrixXd &initial_covariance,
                  const Eigen::MatrixXd &process_noise_covariance,
                  const Eigen::MatrixXd &noise_covariance,
                  const Eigen::MatrixXd &Hd, double initial_voltage) {
    EKF::initialize(delta_t, initial_state, initial_covariance,
                    process_noise_covariance, noise_covariance, Hd);
    this->updateVoltage(initial_voltage);
  }

  bool updateVoltage(float voltage) {
    if (voltage < BASE_VOLTAGE ||
        voltage > BASE_VOLTAGE + (VOLTAGE_LEVELS - 1) * VOLTAGE_STEP) {
      last_error = EKFError::VOLTAGE_OUT_OF_BOUNDS;
      Serial.print("Warning: Voltage ");
      Serial.print(voltage);
      Serial.println(
          "V is out of bounds (3.6V-4.2V). Using nearest acceptable voltage.");
      current_voltage = constrain(voltage, BASE_VOLTAGE, MAX_VOLTAGE);
      last_error = EKFError::VOLTAGE_OUT_OF_BOUNDS;
    } else {
      current_voltage = voltage;
      last_error = EKFError::OK;
    }
    return last_error == EKFError::OK;
  }

  bool step() {
    // Store current valid state
    Eigen::VectorXd prev_state = state;

    last_error = EKFError::OK;
    // PWM values from measurement vector
    float pwm_x = Z(0), pwm_y = Z(1), pwm_z = Z(2);

    // Apply calibration
    float calibrated_values[9];
    bool calibration_success = calculatePWMValues(
        current_voltage, pwm_x, pwm_y, pwm_z, calibrated_values[0],
        calibrated_values[1], calibrated_values[2], calibrated_values[3],
        calibrated_values[4], calibrated_values[5], calibrated_values[6],
        calibrated_values[7], calibrated_values[8]);

    if (!calibration_success) {
      Serial.println("Calibration failed - using raw values");
      last_error = EKFError::INSUFFICIENT_CALIBRATION_DATA;
      return false;
    }
    // Store the original measurement.
    Eigen::VectorXd original_Z = Z;

    // Apply calibrated values
    for (int i = 0; i < 6; i++) {
      Z(i) += calibrated_values[i];
    }

    // Calculate predicted state
    Eigen::VectorXd predicted_state = rk4(state, dt, 0.0, dt);

    // Innovation check with adaptive thresholding
    Eigen::VectorXd innovation = Z - H_d * predicted_state;
    double innovation_magnitude = innovation.norm();
    static double avg_innovation = innovation_magnitude;
    avg_innovation = 0.95 * avg_innovation + 0.05 * innovation_magnitude;

    static const double MAX_INNOVATION_RATIO = 3.0;
    if (innovation_magnitude > MAX_INNOVATION_RATIO * avg_innovation) {
      R_d *= 2.0;
      Serial.println("Large innovation - increasing measurement noise");
    } else {
      R_d *= 0.95; // Gradually reduce if measurements are good
    }

    // Perform EKF step
    Eigen::MatrixXd J = CalculateJacobian();
    predict(J);
    correct();

    // State validation
    bool state_valid = true;
    static const double MAX_MAG = 10.0;
    static const double MAX_GYRO = 1.0;

    for (int i = 0; i < 3; i++) {
      if (std::abs(state(i)) > MAX_MAG || std::abs(state(i + 3)) > MAX_GYRO) {
        state_valid = false;
        break;
      }
    }

    // Handle invalid states
    if (!state_valid || state.hasNaN()) {
      state = prev_state;
      state_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.1;
      last_error = EKFError::STATE_BOUNDS_EXCEEDED;
      return false;
    }

    last_error = EKFError::OK;
    return true;
  }
};

#endif // CALIBRATED_EKF_H