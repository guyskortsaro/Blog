# Arduino NANO

- As a result of needing to operate one motor with a separate board, the Arduino Nano was selected due to its compact size, sufficient GPIO pins, and adequate processing capabilities.

### Motor 2 ROLL

| Sensor / Actuactor    | Pin | Description   | Arduino NANO pin |
| --------------------- | --- | ------------- | ---------------- |
| SimpleFOC mini driver | EN  | EN            | D3 (PWM)         |
| SimpleFOC mini driver | IN1 | Motor Phase 1 | D9 (PWM)         |
| SimpleFOC mini driver | IN2 | Motor Phase 2 | D10 (PWM)        |
| SimpleFOC mini driver | IN3 | Motor Phase 3 | D11 (PWM)        |
| SimpleFOC mini driver | GND | GND           | GND              |

### UART connections

| Slave Board  | Pin | Description | Arduino MEGA 2560 pin |
| ------------ | --- | ----------- | --------------------- |
| Arduino NANO | D6  | Rx -> Tx    | 10                    |
| Arduino NANO | D5  | Tx -> Rx    | 11                    |
