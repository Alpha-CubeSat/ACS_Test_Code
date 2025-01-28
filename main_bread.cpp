#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>

#include "DataLogging.hpp"

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins, 2.29.24 Note that some of the pins were re-arranged to make a cleaner setup
// For breadboard
// #define AIN1 31
// #define AIN2 32
// #define PWMA 30
// #define STBY 29
// #define LED 13

// For updated breadboard (12.03.2024)
#define STBY 32
#define AIN1 31
#define AIN2 30
#define PWMA 29
#define LED 13

// For CubeSat
// For x-magnetorquer
// #define AIN1 25
// #define AIN2 24
// #define PWMA 10
// #define STBY 9

// For y-magnetorquer
// #define AIN1 8
// #define AIN2 7
// #define PWMA 6
// #define STBY 9

// For z-magnetorquer
// #define AIN1 28
// #define AIN2 29
// #define PWMA 30
// #define STBY 27

// #define CSAG 21
// #define CSM 20
// #define SHDN 31
// #define RBSLEEP 19
// #define BURN1 14
// #define BURN2 15
#define file_name "test"

// Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(CSAG, CSM);
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();

void setup()
{
  // Use light_test, magnetorquer, gyro, magnetometer
  String test_name = "softiron_testing";

  // Begin
  Serial.begin(9600);
  Serial.flush();
  Serial.println("begin task");

  // Setup pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Extra pins for CubeSat
  // pinMode(SHDN, OUTPUT);
  // pinMode(RBSLEEP, OUTPUT);
  // pinMode(BURN1, OUTPUT);
  // pinMode(BURN2, OUTPUT);

  // Extra pins for breadboard
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  // analogWrite(PWMA, 0);

  // Extra set pins for CubeSat
  // digitalWrite(SHDN, LOW);
  // digitalWrite(RBSLEEP, LOW);
  // digitalWrite(BURN1, LOW);
  // digitalWrite(BURN2, LOW);
  // analogWrite(PWMA, 0);
  // delay(60000);

  if (test_name == "nothing")
  {
    Serial.println("Do nothing");
    delay(10000);
  }

  if (test_name == "softiron_testing")
  {

    DataLogSetup(file_name);

    if (!imu.begin())
    {
      while (1)
      {
        Serial.println("wrong");
        delay(100);
      };
    }

    // Values pre - 22 Jan 2025
    // float mag_hardiron_x = -4.9547000000000025;
    // float mag_hardiron_y = 49.75155;
    // float mag_hardiron_z = -13.855600000000003;

    // float pwmY_ox_1 = 8.83096680e-03;
    // float pwmY_ox_2 = 4.26409072e-07;
    // float pwmY_ox_3 = -6.69370023e-09;
    // float pwmY_oy_1 = -2.64514092e-01;
    // float pwmY_oy_2 = -9.82458813e-06;
    // float pwmY_oy_3 = 9.11136691e-08;
    // float pwmY_oz_1 = -1.90567242e-02;
    // float pwmY_oz_2 = -5.99945842e-06;
    // float pwmY_oz_3 = 7.85718685e-10;

    // Value post - 22 Jan 2025
    // float mag_hardiron_x = -12.10905;
    // float mag_hardiron_y = 35.2236;
    // float mag_hardiron_z = -0.7527499999999989;

    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
    imu.setupMag(imu.LSM9DS1_MAGGAIN_8GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
    sensors_event_t accel, mag, gyro, temp;

    Serial.println("Beginning to iterate through");

    int prev_pwm = 0;

    delay(1000);

    // for (int pwm_num = 255; pwm_num > -256; pwm_num -= 10)
    for (int i = 0; i < 510; i += 1)
    {
      // Generating random integer between -255 and 255
      randomSeed(analogRead(0));
      int pwm_num = random(-255, 256);

      // int pwm_num = 200;

      // digitalWrite(AIN1, HIGH);
      // digitalWrite(AIN2, LOW);
      // analogWrite(PWMA, pwm_num);

      if (pwm_num < 0)
      {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        analogWrite(PWMA, -pwm_num);
      }
      else
      {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, pwm_num);
      }

      int voltage_value_pin = 23; // For the Breadboard
      // int voltage_value_pin = 32; // For the CubeSat
      float voltage_ref = 3.3;
      int resolution = 1023;
      int r1 = 4700;
      int r2 = 10000;

      float voltage = analogRead(voltage_value_pin) * voltage_ref / resolution * (r1 + r2) / r2;

      float sum_x = 0;
      float sum_y = 0;
      float sum_z = 0;
      int reps = 10;

      for (int i = 0; i < reps; i++)
      {
        imu.getEvent(&accel, &mag, &gyro, &temp);

        sum_x += mag.magnetic.x;
        sum_y += mag.magnetic.y;
        sum_z += mag.magnetic.z;

        delay(10);
      }

      // Serial.println("loop print:");
      // Serial.println(pwm_num);
      // Serial.println(sum_x / reps);
      // Serial.println(sum_y / reps);
      // Serial.println(sum_z / reps);
      // Serial.println(voltage);

      double data[5] = {pwm_num, sum_x / reps, sum_y / reps, sum_z / reps, voltage};
      prev_pwm = pwm_num;
      DataLog(data, 5, file_name);
      delay(50);
    }

    Serial.println("Finished");

    // for (int pwm_num = -255; pwm_num <= 255; pwm_num++)
    // {
    //   // Serial.println(pwm_num);

    //   if (pwm_num < 0)
    //   {
    //     digitalWrite(AIN1, LOW);
    //     digitalWrite(AIN2, HIGH);
    //     analogWrite(PWMA, -pwm_num);
    //   }
    //   else
    //   {
    //     digitalWrite(AIN1, HIGH);
    //     digitalWrite(AIN2, LOW);
    //     analogWrite(PWMA, pwm_num);
    //   }

    //   float sum_x = 0;
    //   float sum_y = 0;
    //   float sum_z = 0;
    //   int reps = 10;

    //   float pwmY_ox = (pwmY_ox_1 * pwm_num) + (pwmY_ox_2 * pow(pwm_num, 2)) + (pwmY_ox_3 * pow(pwm_num, 3));
    //   float pwmY_oy = (pwmY_oy_1 * pwm_num) + (pwmY_oy_2 * pow(pwm_num, 2)) + (pwmY_oy_3 * pow(pwm_num, 3));
    //   float pwmY_oz = (pwmY_oz_1 * pwm_num) + (pwmY_oz_2 * pow(pwm_num, 2)) + (pwmY_oz_3 * pow(pwm_num, 3));

    //   float offset_x = mag_hardiron_x + pwmY_ox;
    //   float offset_y = mag_hardiron_y + pwmY_oy;
    //   float offset_z = mag_hardiron_z + pwmY_oz;

    //   for (int i = 0; i < reps; i++)
    //   {
    //     imu.getEvent(&accel, &mag, &gyro, &temp);

    //     sum_x += mag.magnetic.x;
    //     sum_y += mag.magnetic.y;
    //     sum_z += mag.magnetic.z;

    //     delay(0.01);
    //   }

    //   double data[7] = {pwm_num, sum_x / reps, sum_y / reps, sum_z / reps, offset_x, offset_y, offset_z};
    //   DataLog(data, 7, file_name);
    // }

    digitalWrite(LED, LOW);
  }

  if (test_name == "magnetorquer")
  {
    Serial.println("turned on");
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    // for (int i = 0; i <= 255; i++){
    //   analogWrite(PWMA, i);
    //   delay(100);
    // }
    analogWrite(PWMA, 255);

    delay(4000);

    analogWrite(PWMA, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    Serial.println("turned off");
  }

  if (test_name == "magnetometer")
  {
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
    imu.setupMag(imu.LSM9DS1_MAGGAIN_12GAUSS);
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

    sensors_event_t accel, mag, gyro, temp;

    // float mag_hardiron_x = -4.9547000000000025;
    // float mag_hardiron_y = 49.75155;
    // float mag_hardiron_z = -13.855600000000003;

    // float gyro_hardiron_x = 0.07280736884261114;
    // float gyro_hardiron_y = 0.020224269122947534;
    // float gyro_hardiron_z = 0.016019223067681217;

    // float mag_x = mag.magnetic.x - mag_hardiron_x;
    // float mag_y = mag.magnetic.y - mag_hardiron_y;
    // float mag_z = mag.magnetic.z - mag_hardiron_z;

    // float gyro_x = gyro.magnetic.x - gyro_hardiron_x;
    // float gyro_y = gyro.magnetic.y - gyro_hardiron_y;
    // float gyro_z = gyro.magnetic.z - gyro_hardiron_z;

    // for (int i = 0; i < 50; i++)
    // {
    //   imu.getEvent(&accel, &mag, &gyro, &temp);
    //   Serial.println(mag.magnetic.z);
    //   delay(100);
    // }

    for (int i = 0; i < 25; i++)
    {
      imu.getEvent(&accel, &mag, &gyro, &temp);
      Serial.println(mag.magnetic.z);
      delay(100);
    }

    Serial.println("turned on");

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    analogWrite(PWMA, 255);

    for (int i = 25; i < 75; i++)
    {
      imu.getEvent(&accel, &mag, &gyro, &temp);
      Serial.println(mag.magnetic.z);
      delay(100);
    }

    Serial.println("turned off");
    analogWrite(PWMA, 0);

    for (int i = 75; i < 100; i++)
    {
      imu.getEvent(&accel, &mag, &gyro, &temp);
      Serial.println(mag.magnetic.z);
      delay(100);
    }

    digitalWrite(LED, LOW);
  }

  if (test_name == "gyro")
  {
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

    sensors_event_t accel, mag, gyro, temp;

    for (int i = 0; i < 50; i++)
    {
      imu.getEvent(&accel, &mag, &gyro, &temp);
      Serial.println(gyro.gyro.z);
      delay(100);
    }
  }

  if (test_name == "light_test")
  {
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(LED, LOW);
      delay(1000);
      digitalWrite(LED, HIGH);
      delay(1000);
    }
  }

  digitalWrite(STBY, LOW);
  digitalWrite(LED, LOW);
}

void loop()
{
}