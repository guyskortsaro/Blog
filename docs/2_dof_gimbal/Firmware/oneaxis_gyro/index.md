# One AXIS Gyro (Pitch)

The main goal is to test the control logic of a two-axis gimbal on a single axis first. This approach simplifies the hardware setup and allows for quicker and more focused debugging, helping validate the core functionality before scaling up to full two-axis control.

### Initial attempt

```cpp
#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>

// Define PITCH Motor Driver PINS
#define PITCH_PWM_A 2
#define PITCH_PWM_B 3
#define PITCH_PWM_C 4
#define PITCH_ENABLE_PIN 8

// Define Driver
BLDCDriver3PWM PITCH_driver(PITCH_PWM_A, PITCH_PWM_B, PITCH_PWM_C, PITCH_ENABLE_PIN);

// Define Motors
BLDCMotor PITCH_motor(11);

// Define Encoders
MagneticSensorSPI PITCH_encoder = MagneticSensorSPI(12, 14, 0x3FFF); // SCN pin is 12

// Define IMU
MPU6050 mpu6050;

// Define Time Variables
unsigned long lastTime = 0, currentTime = 0;
float         dt = 0.0;

// Define IMU Variables
float gyroBiasX = 0.0, gyroBiasY = 0.0, pitch = 0.0, roll = 0.0;

// Define IMU Complimentary Filter Variables
const float pitch_alpha   = 0.96;
const float roll_alpha    = 0.96;
float       gyroPitchRate = 0.0, gyroRollRate = 0.0;
float       prevPitchRate = 0.0;

// Define debugger index counter
int debugger_index = 0;

// Pitch velocity low-pass-filter
float       pitch_target_velocity_filtered = 0.0;
const float velocity_filter_alpha          = 0.2;

// Define PITCH variables for motor movment and PID control
float pitch_target_velocity = 0.0;
float pitch_Kp = 1, pitch_Ki = 0.0, pitch_Kd = 0.0, pitch_P = 0.0, pitch_I = 0.0, pitch_D = 0.0, pitch_angle_Error = 0.0, pitch_angle_Error_DOT = 0.0;
float pitch_angle_Error_SUM = 0.0, pitch_angle_Error_prev = 0.0, pitch_U = 0.0;

// Steady state variables
bool        pitch_in_steady_state              = false;
const float pitch_steady_state_enter_threshold = 0.01;
const float pitch_steady_state_exit_threshold  = 0.03;

// Setup Functions
void Initialize_Encoders()
{
    PITCH_encoder.init();
    Serial.println("Pitch Encoder is initialized");
}
void Initialize_Drivers()
{
    PITCH_driver.voltage_power_supply = 12;
    PITCH_driver.init();
}
void Linking_Drivers_And_Encoders_To_Motors()
{
    PITCH_motor.linkDriver(&PITCH_driver);
    PITCH_motor.linkSensor(&PITCH_encoder);
}
void Setting_Motors_Modes()
{
    PITCH_motor.controller = MotionControlType::velocity_openloop;
}
void Initialize_Motors()
{
    PITCH_motor.init();
    PITCH_motor.initFOC();
}
void Initialize_IMU()
{
    Wire.begin();
    mpu6050.initialize();
    if (!mpu6050.testConnection())
    {
        Serial.println("MPU6050 connection failed!");
        while (1)
            ;
    }
    Serial.println("MPU6050 initialized.");
    delay(1000);
}
void Gyro_Bias_Calibration()
{
    Serial.println("Calibrating gyroscope... Keep the sensor still.");
    long      gxSum = 0, gySum = 0;
    const int numSamples = 500;
    for (int i = 0; i < numSamples; i++)
    {
        gxSum += mpu6050.getRotationX();
        gySum += mpu6050.getRotationY();
        delay(2); // Small delay for stability
    }
    gyroBiasX = -gxSum / (float)numSamples;
    gyroBiasY = -gySum / (float)numSamples;
    Serial.print("Gyro bias X: ");
    Serial.println(gyroBiasX);
    Serial.print("Gyro bias Y: ");
    Serial.println(gyroBiasY);
    lastTime = millis(); // Initialize timer
}

// loop Functions
void Pitch_Calculation()
{
    // Read raw data
    int16_t ax = mpu6050.getAccelerationX();
    int16_t ay = mpu6050.getAccelerationY();
    int16_t az = mpu6050.getAccelerationZ();
    int16_t gx = mpu6050.getRotationX();
    int16_t gy = mpu6050.getRotationY();
    int16_t gz = mpu6050.getRotationZ();

    // Time difference in seconds
    currentTime = millis();
    dt          = (currentTime - lastTime) / 1000.0;
    lastTime    = currentTime;

    // Convert accel to float
    float axf = (float)ax;
    float ayf = (float)ay;
    float azf = (float)az;

    // Accel-derived angles (in radians)
    float denom_pitch = sqrt(axf * axf + azf * azf);
    if (denom_pitch == 0)
        denom_pitch = 0.0001;
    float accelPitch = atan2(ayf, denom_pitch);

    // Apply gyro bias correction and scale to rad/s
    gyroPitchRate = (gy + gyroBiasY) / 131.0 * DEG_TO_RAD;

    // Complementary filter
    pitch = pitch_alpha * (pitch + gyroPitchRate * dt) + (1 - pitch_alpha) * accelPitch;
}
void Pitch_PID_Calculations()
{
    pitch_angle_Error      = (0 - pitch);
    pitch_angle_Error_DOT  = (pitch_angle_Error - pitch_angle_Error_prev) / dt;
    pitch_angle_Error_prev = pitch_angle_Error;
    pitch_angle_Error_SUM  = pitch_angle_Error * dt;
    pitch_P                = pitch_Kp * pitch_angle_Error;
    pitch_I += pitch_Ki * pitch_angle_Error_SUM;
    pitch_D               = pitch_Kd * pitch_angle_Error_DOT;
    pitch_U               = pitch_P + pitch_I + pitch_D;
    pitch_target_velocity = pitch_U / dt;
}
void Steady_State()
{
    if (pitch_in_steady_state)
    {
        if (abs(pitch_angle_Error) > pitch_steady_state_exit_threshold)
        {
            pitch_in_steady_state = false;
        }
    }
    else
    {
        if (abs(pitch_angle_Error) < pitch_steady_state_enter_threshold)
        {
            pitch_in_steady_state = true;
        }
    }

    if (pitch_in_steady_state)
    {
        pitch_target_velocity = 0.0;
    }
    else
    {
        pitch_target_velocity = pitch_U;
    }
}
void Debbuging_Print()
{
    debugger_index++;
    if (debugger_index == 200)
    {
        Serial.print("Pitch: ");
        Serial.print(pitch, 2);
        Serial.print(" Rad         Pitch Rate: ");
        Serial.print(gyroPitchRate, 2);
        Serial.print(" Rad/s       Target Velocity: ");
        Serial.print(pitch_target_velocity, 2);
        Serial.println(" Rad/s");
        debugger_index = 0;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Initialize_Encoders();
    Initialize_Drivers();
    Linking_Drivers_And_Encoders_To_Motors();
    Setting_Motors_Modes();
    Initialize_Motors();
    Initialize_IMU();
    Gyro_Bias_Calibration();
}

void loop()
{
    Pitch_Calculation();
    Pitch_PID_Calculations();
    Steady_State();
    Debbuging_Print();

    // Moving motor
    PITCH_motor.move(pitch_target_velocity);
}

```

### Final Version (notice header files)

```cpp
#include "Roll_init.h"
#include "myPID.h"
#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>

////////////////////////////////////////////
// -------- PID Control Variables -------- //
////////////////////////////////////////////
myPID rollPID(4.0, 1.0, 2.5);
float RollPID_output = 0.0;
float Roll_target_velocity  = 0.0;

////////////////////////////////////////////
// ---------- IMU Configuration ---------- //
////////////////////////////////////////////
MPU6050 mpu;

float gyroBiasX = 0.0, gyroBiasY = 0.0;
float roll = 0.0;
float gyroRollRate = 0.0;
float prevPitchRate = 0.0;
const float roll_alpha  = 0.98;
const float GYRO_SCALE = 131.0;
const float GYRO_LPF_ALPHA = 0.95;

////////////////////////////////////////////
// --------- Timing Variables ------------ //
////////////////////////////////////////////
unsigned long lastTime = 0, currentTime = 0;
float dt = 0.0;
unsigned long lastTime_PID = 0, currentTime_PID = 0;
float dt_PID = 0.0;

////////////////////////////////////////////
// ----------- Debugging Vars ----------- //
////////////////////////////////////////////
int debugger_index = 0;

void Initialize_Drivers() {
  ROLL_driver.voltage_power_supply = 12;
  ROLL_driver.init();
}

void Linking_Drivers_And_Encoders_To_Motors() {
  ROLL_motor.linkDriver(&ROLL_driver);
}

void Setting_Motors_Modes() {
  ROLL_motor.controller  = MotionControlType::velocity_openloop;
}

void Initialize_Motors() {
  ROLL_motor.init();
  ROLL_motor.initFOC();
}

void Initialize_IMU() {
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    Serial.println("MPU6050 initialized.");
    delay(1000);
}

void Gyro_Bias_Calibration(int samples = 360) {
    long gxSum = 0, gySum = 0;

    Serial.println("Calibrating gyroscope... Keep the sensor still.");

    for (int i = 0; i < samples; i++) {
        gxSum += mpu.getRotationX();
        gySum += mpu.getRotationY();
        delay(2);
    }

    gyroBiasX = gxSum / (float)samples;
    gyroBiasY = gySum / (float)samples;

    Serial.print("Gyro bias X: ");
    Serial.println(gyroBiasX);
    Serial.print("Gyro bias Y: ");
    Serial.println(gyroBiasY);

    lastTime = millis(); // Initialize timer
}

////////////////////////////////////////////
// ---------- Sensor Processing ---------- //
////////////////////////////////////////////
void Pitch_And_Roll_Calculation() {
    // Read raw data
    int16_t ax = mpu.getAccelerationX();
    int16_t ay = mpu.getAccelerationY();
    int16_t az = mpu.getAccelerationZ();
    int16_t gx = mpu.getRotationX();
    int16_t gy = mpu.getRotationY();

    // Time delta in seconds
    currentTime = millis();
    dt = (float)(currentTime - lastTime) / 1000.0f;
    if (dt <= 0.0) dt = 0.001;
    lastTime = currentTime;

    // Convert accelerometer to float
    float axf = (float)ax;
    float ayf = (float)ay;
    float azf = (float)az;

    // Accel-derived angles
    float accelRoll  = atan2(ayf, azf);                          // around X

    // --- Filter time check ---
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f)  dt = 0.05f;

    // --- Raw gyro conversion ---
    float rawGyroRoll  = (gx - gyroBiasX) / GYRO_SCALE * DEG_TO_RAD;

    // --- LPF smoothing ---
    gyroRollRate  = GYRO_LPF_ALPHA * gyroRollRate  + (1 - GYRO_LPF_ALPHA) * rawGyroRoll;

    // --- Complementary filter ---
    roll  = roll_alpha * (roll + gyroRollRate * dt) + (1 - roll_alpha) * accelRoll;
}

////////////////////////////////////////////
// ---------- Motor Control ------------- //
////////////////////////////////////////////
void PID_calculation_Motor_move() {
  currentTime_PID = millis();
  dt_PID = (float)(currentTime_PID - lastTime_PID) / 1000.0f;
  if (dt_PID <= 0.01) dt_PID = 0.001;
  lastTime_PID = currentTime_PID;
  RollPID_output        = rollPID.compute(0.0, roll, dt_PID)/dt_PID;
  Roll_target_velocity  = constrain(RollPID_output, -15.0, 15.0);
  ROLL_motor.move(Roll_target_velocity);
}

////////////////////////////////////////////
// ------------- Debug Output ------------ //
////////////////////////////////////////////
void Debbuging_Print() {
  debugger_index++;
  if (debugger_index == 1) {
      Serial.print(millis());              Serial.print(",");
      Serial.print(roll, 4);               Serial.print(",");
      Serial.println(gyroRollRate, 4);
      debugger_index = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Initialize_Drivers();
  Linking_Drivers_And_Encoders_To_Motors();
  Setting_Motors_Modes();
  Initialize_Motors();
  Initialize_IMU();
  Gyro_Bias_Calibration();
}

void loop() {
  Pitch_And_Roll_Calculation();
  PID_calculation_Motor_move();
  Debbuging_Print();
}
```
