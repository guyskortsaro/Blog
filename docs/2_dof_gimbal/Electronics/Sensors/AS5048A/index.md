# AS5048A 

The AS5048A is a high-resolution 14-bit rotary position sensor based on magnetic Hall effect technology. Hall effect sensors measure magnetic fields to determine position, speed, or rotation, and are commonly used in contactless sensing applications. The AS5048A is designed for precise angular position measurements of a rotating magnet, making it well-suited for motor control, robotics, and industrial automation systems. It supports both SPI and PWM interfaces, allowing flexible integration with a variety of microcontrollers. Its non-contact nature ensures durability and longevity, even in harsh environments. For a complete overview of the sensor’s capabilities and electrical characteristics, please refer to the attached datasheet.

[MPU6050 Datasheet link](<files/AS5048Adatasheet.pdf>)

## AS5408A Magnetic Encoder to Arduino UNO connection:

- Keep in mind that this is a "mirror image" to the encoder connection side

| AS5408A pin  | Description  | Arduino UNO pin  |
|-----------|-----------|-----------|
| green  | 5v  | 5V  |
| purple  | GND  | GND  |
| white  | CSN  | D10 |
| black  | MOSI  | D11  |
| yellow  | MISO | D12  |
| red  |  CLK | D13  |
