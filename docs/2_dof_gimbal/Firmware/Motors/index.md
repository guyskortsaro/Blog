# GM3506 Brushless motor using SimpleFOC mini v1.0 driver and AS5048A encoder

Before implementing any gyroscopic control, it was important to first understand how to operate the GM3506 brushless motor using the SimpleFOC Mini v1.0 driver. This stage focused on getting the motor to spin reliably in both directions, tuning basic motion parameters, and verifying correct feedback from the magnetic encoder. Establishing this foundation ensured stable motor performance, which is critical for accurate and responsive gimbal control later on.

## 1. Encoder + motor velocity open-loop operation

```cpp
#include <Arduino.h>
#include <SimpleFOC.h>

// Define motor driver pins for Arduino Uno
#define PWM_A 6
#define PWM_B 5
#define PWM_C 3
#define ENABLE_PIN 2

// Create the driver instance
BLDCDriver3PWM driver(PWM_A, PWM_B, PWM_C, ENABLE_PIN);

// Motor instance (11 pole pairs for GM3506, or adjust as needed)
BLDCMotor motor = BLDCMotor(11);

// Sensor instance (not used in open-loop, but declared)
MagneticSensorSPI as5048u = MagneticSensorSPI(10, 14, 0x3FFF);  // CS pin, bit resolution, angle register

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Sensor init (not required for open-loop)
  as5048u.init();
  Serial.println("AS5048U sensor initialized");

  // Driver config
  driver.voltage_power_supply = 12;
  driver.init();

  // Link driver to motor
  motor.linkDriver(&driver);

  // Open-loop velocity control mode
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 6;  // Half of supply for safety

  // Motor init
  motor.init();
  Serial.println("Motor Ready!");
}

void loop()
{
  // Set a constant target velocity (e.g., 5 rad/s)
  motor.move(5);  // Change value to reverse or adjust speed
}
```
## 2. Encoder + motor position/angle motion control

```cpp
/**
* Position/angle motion control
* Steps:
* 1) Configure the motor and magnetic sensor
* 2) Run the code
* 3) Set the target angle (in radians) from serial terminal
*/
#include <Arduino.h>
#include <SimpleFOC.h>

// Define motor driver pins
#define PWM_A 9
#define PWM_B 6
#define PWM_C 5
#define ENABLE_PIN 4

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver(PWM_A, PWM_B, PWM_C, ENABLE_PIN);

// angle set point variable
float target_angle = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup() {

 // use monitoring with serial
 Serial.begin(115200);
 // enable more verbose output for debugging
 // comment out if not needed
 SimpleFOCDebug::enable(&Serial);

 // initialise magnetic sensor hardware
 sensor.init();
 // link the motor to the sensor
 motor.linkSensor(&sensor);

 // driver config
 // power supply voltage [V]
 driver.voltage_power_supply = 12;
 driver.init();
 // link the motor and the driver
 motor.linkDriver(&driver);

 // choose FOC modulation (optional)
 motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

 // set motion control loop to be used
 motor.controller = MotionControlType::angle;

 // contoller configuration
 // default parameters in defaults.h

 // velocity PI controller parameters
 motor.PID_velocity.P = 0.5;
 motor.PID_velocity.I = 20;
 motor.PID_velocity.D = 0.001;
 // maximal voltage to be set to the motor
 motor.voltage_limit = 6;

 // velocity low pass filtering time constant
 // the lower the less filtered
 motor.LPF_velocity.Tf = 0.1;

 // angle P controller
 motor.P_angle.P = 20;
 // maximal velocity of the position control
 motor.velocity_limit = 10;
  // comment out if not needed
 motor.useMonitoring(Serial);

 // initialize motor
 motor.init();
 // align sensor and start FOC
 motor.initFOC();

 // add target command T
 command.add('T', doTarget, "target angle");

 Serial.println(F("Motor ready."));
 Serial.println(F("Set the target angle using serial terminal:"));
}

void loop() {

 // main FOC algorithm function
 // the faster you run this function the better
 // Arduino UNO loop  ~1kHz
 // Bluepill loop ~10kHz
 motor.loopFOC();

 // Motion control function
 // velocity, position or voltage (defined in motor.controller)
 // this function can be run at much lower frequency than loopFOC() function
 // You can also use motor.move() and set the motor.target in the code
 motor.move(target_angle);

 // function intended to be used with serial plotter to monitor motor variables
 // significantly slowing the execution down!!!!
 // motor.monitor();

 // user communication
 command.run();
}
```

