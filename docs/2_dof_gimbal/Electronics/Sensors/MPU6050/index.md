# MPU6050 

The MPU6050 is a widely used 6-axis Inertial Measurement Unit (IMU) that integrates a 3-axis gyroscope and a 3-axis accelerometer into a single chip. IMUs are sensors that measure linear acceleration and angular velocity, providing essential data for applications such as motion tracking, orientation sensing, and control systems. The MPU6050 combines motion sensing capabilities with an onboard Digital Motion Processor (DMP), allowing it to process complex motion algorithms internally and offload computation from the main microcontroller. Its compact form factor, I²C interface, and real-time motion data output make it a popular choice for embedded systems, robotics, and consumer electronics. For detailed specifications, please refer to the attached datasheet.

[MPU6050 Datasheet link](<files/mpu6050datasheet.pdf>)

## Calculating Pitch and Roll from MPU6050 Data

The MPU6050 provides **accelerometer** and **gyroscope** data, which can be used to estimate the orientation of a device in space. Specifically, we calculate the **pitch** and **roll** angles. Here's how each is derived:

### 1. Using Accelerometer Only

The accelerometer provides information about the direction of gravity. From this, we can estimate pitch and roll:

$$
\text{roll} = \arctan\left(\frac{a_y}{a_z}\right)
$$

$$
\text{pitch} = \arctan\left(\frac{-a_x}{\sqrt{a_y^2 + a_z^2}}\right)
$$

- \( a_x, a_y, a_z \) are the accelerometer readings along the X, Y, and Z axes.
- These estimates are stable but noisy and sensitive to linear acceleration.

### 2. Using Gyroscope Only

The gyroscope provides angular velocity in rad/s or deg/s around each axis. To estimate the angle over time:

$$
\theta(t) = \theta(t - \Delta t) + \omega \cdot \Delta t
$$

Where:
- \( \theta(t) \) is the current angle (pitch or roll),
- \( \omega \) is the angular rate from the gyroscope,
- \( \Delta t \) is the time interval between readings.

This method provides smooth results but suffers from **drift over time**.

### 3. Sensor Fusion with Complementary Filter

To combine the advantages of both sensors, a **complementary filter** is used:

$$
\text{angle} = \alpha \cdot (\text{angle} + \omega \cdot \Delta t) + (1 - \alpha) \cdot \text{accelAngle}
$$

Where:
- \( \alpha \) is a tuning constant between 0 and 1 (e.g., 0.96),
- \( \omega \cdot \Delta t \) is the integrated gyroscope angle,
- \( \text{accelAngle} \) is the angle derived from the accelerometer.

This approach balances **smoothness** (from gyro) and **long-term accuracy** (from accelerometer), resulting in a reliable and efficient orientation estimation method.

## Reducing Measurement Overshoot with a Low-Pass Filter

When working with sensor data such as angular rates or accelerometer outputs from the MPU6050, it's common to experience **noise and overshoot**, especially during rapid motion. To address this, we can apply a **low-pass filter (LPF)** to smooth out sudden spikes and stabilize the output.

### 1. Purpose of a Low-Pass Filter

A **low-pass filter** allows **low-frequency (slow-changing)** signals to pass through while attenuating **high-frequency (fast-changing)** noise. This helps reduce overshoot and jitter in angle calculations or rate measurements.

### 2. Filter Equation (Exponential Smoothing)

The most common and simple form of LPF used in embedded systems is the **exponential moving average**, calculated as:

$$
y_t = \alpha \cdot y_{t-1} + (1 - \alpha) \cdot x_t
$$

Where:
- \( y_t \) is the **filtered output** at time \( t \),
- \( y_{t-1} \) is the **previous output**,
- \( x_t \) is the **current raw input** (e.g., gyro reading),
- \( \alpha \) is the **filter coefficient** between 0 and 1.

### 3. Choosing the Filter Coefficient \( \alpha \)

- A **higher \( \alpha \)** (e.g., 0.95) results in **smoother output**, but reacts slowly to changes.
- A **lower \( \alpha \)** (e.g., 0.7) makes the filter more responsive, but lets more noise through.



