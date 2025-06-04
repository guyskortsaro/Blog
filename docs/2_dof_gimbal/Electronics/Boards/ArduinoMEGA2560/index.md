# Arduino MEGA 2560

- Arduino MEGA 2560 got selected due to the number of PWM pins available in addition to SPI and I2C that are required by the sensors and actuators.

## Arduino MEGA 2560 wiring scheme:

<img src="images/arduino_mega_pinset.png" alt="Arduino mega pinset" width="1000">

[Arduino MEGA 2560 detailed specs](https://devboards.info/boards/arduino-mega2560-rev3)

### General connections

| Sensor / Actuactor | Pin | Description | Arduino MEGA 2560 pin |
| ------------------ | --- | ----------- | --------------------- |
| MPU6050            | SDA | SDA         | D20 (SDA)             |
| MPU6050            | SCL | SCL         | D21 (SCL)             |
| MPU6050            | Vcc | Vcc         | 3V3                   |
| MPU6050            | GND | GND         | GND                   |

### Motor 1 PITCH

| Sensor / Actuactor    | Pin | Description   | Arduino MEGA 2560 pin |
| --------------------- | --- | ------------- | --------------------- |
| SimpleFOC mini driver | EN  | EN            | D8 (PWM)              |
| SimpleFOC mini driver | IN1 | Motor Phase 1 | D2 (PWM)              |
| SimpleFOC mini driver | IN2 | Motor Phase 2 | D3 (PWM)              |
| SimpleFOC mini driver | IN3 | Motor Phase 3 | D4 (PWM)              |
| SimpleFOC mini driver | GND | GND           | GND                   |

### UART connections

| Slave Board  | Pin | Description | Arduino MEGA 2560 pin |
| ------------ | --- | ----------- | --------------------- |
| Arduino NANO | D6  | Rx -> Tx    | 10                    |
| Arduino NANO | D5  | Tx -> Rx    | 11                    |


