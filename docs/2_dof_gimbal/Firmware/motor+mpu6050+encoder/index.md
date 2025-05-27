# Single motor (integrated system) attempt

Before diving into the gyro control logic, it was important to develop a solid understanding of each individual component—namely, the MPU6050 IMU, the GM3506 brushless motor, the AS5048A magnetic encoder, and the SimpleFOC Mini v1.0 driver. Learning to configure and operate each part independently helped build confidence in the system’s behavior. This modular approach allowed for focused troubleshooting and ensured that the sensor data, motor control, and position feedback were all functioning correctly before combining them into a closed-loop gyro control algorithm. This step invloves learning to work with a few different motor modes and understanding the diffrance between them.

### Hardware list

this system includes the following hardware:

| Type            | Name                |
| --------------- | ------------------- |
| Brushless motor | GM3506              |
| Driver          | SimpleFOC mini v1.0 |
| IMU             | MPU6050             |
| Microcontroller | Arduino UNO         |

## MPU6050 roll reading and motor moving based on these readings (Angle Open Loop)

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <math.h>
#include <MPU6050.h>

// Define motor driver pins for Arduino Uno
#define PWM_A 6
#define PWM_B 5
#define PWM_C 3
#define ENABLE_PIN 2

// Create the driver instance
BLDCDriver3PWM driver(PWM_A, PWM_B, PWM_C, ENABLE_PIN);
// Motor and sensor objects
BLDCMotor motor = BLDCMotor(11); // 7 is the number of pole pairs
MPU6050 mpu6050;
// Define the sensor
MagneticSensorSPI as5048u = MagneticSensorSPI(10, 14, 0x3FFF); // CS pin for AS5048A
// Define lowpass filter
LowPassFilter filter = LowPassFilter(0.1); // Tf = 10ms

void setup() {
  Serial.begin(115200);
  delay(1000);
  as5048u.init();
  Serial.println("AS5048U sensor initialized");
  // Driver config
  driver.voltage_power_supply = 12;
  driver.init();
  // Link driver to motor
  motor.linkDriver(&driver);
  // Initialize MPU6050
  Wire.begin();
  mpu6050.initialize();
  // Initialize motor
  motor.linkSensor(&as5048u); // Link the sensor to the motor
  motor.init(); // Initialize the motor
  motor.initFOC(); // Initialize FOC
  motor.controller = MotionControlType::angle_openloop; // Angle openloop approuch
  motor.voltage_limit = 6;  // Half of supply for safety
  // Motor parameters
  motor.velocity_limit = 50; // Set a velocity limit (possible to adjust)
  motor.current_limit = 1.0; // Set a current limit (possible to adjust)
  motor.move(0); // Move to the initial position
}

void loop() {
  // Read accelerometer data
  int16_t ax, ay, az;
  mpu6050.getAcceleration(&ax, &ay, &az);
  // Calculate roll angle
  float roll = atan2(ay, az);
  float filtered_roll = filter(roll);
  // Control motor based on filterd roll
  motor.move(filtered_roll);
  // Printing debug information to Serial Monitor
  Serial.print("Roll Angle: ");
  Serial.println(filtered_roll);
  delay(100);
}
```

## MPU6050 roll reading and motor moving base on these readings (Angle Closed Loop)

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <math.h>
#include <MPU6050.h>

// ====== Motor Driver Pins ======
#define PWM_A 6
#define PWM_B 5
#define PWM_C 3
#define ENABLE_PIN 2

// ====== Motor & Sensor Setup ======
BLDCDriver3PWM driver(PWM_A, PWM_B, PWM_C, ENABLE_PIN);
BLDCMotor motor = BLDCMotor(11);  // 11 pole pairs
MagneticSensorSPI as5048u = MagneticSensorSPI(10, 14, 0x3FFF);  // CS pin, bit resolution, register

// ====== MPU6050 Setup ======
MPU6050 mpu6050;

// ====== Low-pass Filter ======
LowPassFilter filter = LowPassFilter(0.1);  // Smoother response

// ====== Control Parameters ======
const float rollToAngleGain = 1.0;  // Gain for roll-to-motor angle mapping
float initial_roll = 0.0;

// ====== Timing Setup ======
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 5;  // X Hz update
unsigned long lastPrint = 0;
const unsigned long printInterval = 9000;  // Debug print

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize AS5048A encoder
  as5048u.init();
  Serial.println("AS5048A initialized");

  // Initialize motor driver
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Motor setup
  motor.init();
  motor.linkSensor(&as5048u);
  motor.initFOC();
  motor.voltage_limit = 6;
  motor.current_limit = 1.0;
  motor.velocity_limit = 50;
  motor.controller = MotionControlType::angle;
  motor.P_angle.P = 5;
  motor.P_angle.I = 0.1;
  motor.P_angle.D = 0.5;
  motor.LPF_angle.Tf = 0.01;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 5.0;
  motor.LPF_velocity.Tf = 1.0;

  // Initialize MPU6050
  Wire.begin();
  mpu6050.initialize();
  if (mpu6050.testConnection()) {
    Serial.println("MPU6050 connected");

    // Get initial roll for relative reference
    int16_t ax, ay, az;
    mpu6050.getAcceleration(&ax, &ay, &az);
    initial_roll = atan2(ay, az);
    Serial.print("Initial roll angle [rad]: ");
    Serial.println(initial_roll, 4);
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  motor.move(0);  // Hold initial position
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime >= loopInterval) {
    lastLoopTime = now;
    motor.loopFOC();

    // ====== Read MPU and calculate roll ======
    int16_t ax, ay, az;
    mpu6050.getAcceleration(&ax, &ay, &az);
    float roll = atan2(ay, az);
    float relative_roll = roll - initial_roll;
    float filtered_roll = filter(relative_roll);

    // ====== Move motor based on filtered roll ======
    float target_angle = filtered_roll * rollToAngleGain;
    motor.move(target_angle);

    // ====== Debug Print ======
    if (now - lastPrint >= printInterval) {
      Serial.print("Filtered Roll [rad]: ");
      Serial.print(filtered_roll, 4);
      Serial.print(" | Motor Target Angle [rad]: ");
      Serial.println(target_angle, 4);
      lastPrint = now;
    }
  }
}
```

## MPU6050 roll reading and motor moving base on these readings (Velocity Closed Loop)

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <math.h>
#include <MPU6050.h>

// ====== Motor Driver Pins ======
#define PWM_A 6
#define PWM_B 5
#define PWM_C 3
#define ENABLE_PIN 2

// ====== Motor & Sensor Setup ======
BLDCDriver3PWM driver(PWM_A, PWM_B, PWM_C, ENABLE_PIN);
BLDCMotor motor = BLDCMotor(11);  // 11 pole pairs
MagneticSensorSPI as5048u = MagneticSensorSPI(10, 14, 0x3FFF);  // CS pin, bit resolution, register

// ====== MPU6050 Setup ======
MPU6050 mpu6050;

// ====== Low-pass Filter ======
LowPassFilter filter = LowPassFilter(0.05);  // Smoother response

// ====== Control Parameters ======
const float rollToAngleGain = 1.0;  // Gain for converting filtered roll to desired angle (radians)
float initial_roll = 0.0;

// Gain for cascaded velocity control: converts angle error to velocity command (rad/s per rad)
const float velGain = 2.0;

// ====== Timing Setup ======
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 5;  // 200 Hz update
unsigned long lastPrint = 0;
const unsigned long printInterval = 1000;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Initialize AS5048A encoder
  as5048u.init();
  Serial.println("AS5048A initialized");

  // Initialize motor driver
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&as5048u);

  // Set to velocity control mode
  motor.controller = MotionControlType::velocity;

  // Motor parameters
  motor.voltage_limit = 6;
  motor.current_limit = 1.0;
  motor.velocity_limit = 400;  // Increase if needed

  // PID for velocity controller
  motor.PID_velocity.P = 2;
  motor.PID_velocity.I = 5;
  motor.LPF_velocity.Tf = 1;

  // Initialize motor and FOC (order is important)
  motor.init();
  motor.initFOC();

  // Initialize MPU6050
  Wire.begin();
  mpu6050.initialize();
  if (mpu6050.testConnection()) {
    Serial.println("MPU6050 connected");

    // Read initial roll to use as reference
    int16_t ax, ay, az;
    mpu6050.getAcceleration(&ax, &ay, &az);
    initial_roll = atan2(ay, az);
    Serial.print("Initial roll angle [rad]: ");
    Serial.println(initial_roll, 4);
  } else {
    Serial.println("MPU6050 connection failed!");
  }

  motor.move(0);  // Hold initial state
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime >= loopInterval) {
    lastLoopTime = now;

    motor.loopFOC();

    // ====== Read MPU6050 and compute desired angle ======
    int16_t ax, ay, az;
    mpu6050.getAcceleration(&ax, &ay, &az);
    float roll = atan2(ay, az);
    float relative_roll = roll - initial_roll;
    float filtered_roll = filter(relative_roll);

    // Map filtered roll to a desired motor angle.
    // With rollToAngleGain = 1.0, the desired angle equals the filtered roll.
    float desired_angle = filtered_roll * rollToAngleGain;

    // ====== Compute angle error and derive a target velocity ======
    // motor.shaft_angle is provided by the encoder (in radians)
    float angle_error = desired_angle - motor.shaft_angle;
    float target_velocity = velGain * angle_error; // velocity command (rad/s)

    // Pass the velocity command to the motor
    motor.move(target_velocity);

    // ====== Debug Print ======
    if (now - lastPrint >= printInterval) {
      Serial.print("Filtered Roll [rad]: ");
      Serial.print(filtered_roll, 4);
      Serial.print(" | Desired Angle [rad]: ");
      Serial.print(desired_angle, 4);
      Serial.print(" | Motor Angle [rad]: ");
      Serial.print(motor.shaft_angle, 4);
      Serial.print(" | Angle Error [rad]: ");
      Serial.print(angle_error, 4);
      Serial.print(" | Target Velocity [rad/s]: ");
      Serial.println(target_velocity, 4);
      lastPrint = now;
    }
  }
}
```
