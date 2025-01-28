#ifndef IMU_MONITOR_HPP_
#define IMU_MONITOR_HPP_
#include "../ekf/CalibratedEKF.hpp"
#include "Adafruit_LSM9DS1.h"
#include "constants.hpp"
#include <Arduino.h>
#include <cmath>
#include <map>

#define VOLTAGE_PIN 23

typedef std::array<std::array<float, 3>, 7> OffsetArray;

using namespace constants::imu;

class IMUMonitor {
public:
  CalibratedEKF ekfObj;

  IMUMonitor();
  void execute();
  void printLastError() const;
  const Adafruit_LSM9DS1 &getIMU();

private:
  //   // Define constants surrounding interpolated data for function usage.
  //   static constexpr size_t VOLTAGE_LEVELS = 7; // 3.6V to 4.2V
  //   static constexpr float BASE_VOLTAGE = 3.6;
  //   static constexpr float VOLTAGE_STEP = 0.1;
  //   static constexpr float MAX_VOLTAGE = 4.2;
  EKFError lastError = EKFError::OK;
  // Global map to store all coefficients indexed by voltage

  // Takes the precision beyond the tenths place of voltage and turns that into
  // a percentage that can then be factored and used to interpolate the values
  // that are between the values in the multidimensional arrays that have been
  // generated from the values in the csv files.
  bool getInterpolatedOffsets(
      float voltage, constants::imu::PWMCoefficients &coeffs,
      const std::map<float, constants::imu::VoltageCoefficients>
          &voltage_coeffs,
      const std::function<const constants::imu::PWMCoefficients &(
          const constants::imu::VoltageCoefficients &)> &coeffSelector);
  bool calculatePWMValues(float voltage, float pwm_x, float pwm_y, float pwm_z,
                          float &pwmX_ox, float &pwmX_oy, float &pwmX_oz,
                          float &pwmY_ox, float &pwmY_oy, float &pwmY_oz,
                          float &pwmZ_ox, float &pwmZ_oy, float &pwmZ_oz);
  void IMU_init();
  void invalidate_data();
  void transition_to_normal();
  void transition_to_abnormal_init();
  void capture_imu_values();
  void imu_offset();
  float temp;
  float voltage;
  float readVoltage();
  float mag_x;
  float mag_y;
  float mag_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  // Pre-allocated arrays for offset storage
  std::array<float, 3> x_offsets;
  std::array<float, 3> y_offsets;
  std::array<float, 3> z_offsets;

  Adafruit_LSM9DS1 imu;

  bool first = true;
};

#endif