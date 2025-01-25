# Bipedal Robot Leg Movement Control Using Inverse Kinematics and PCA9685

This project demonstrates controlling a bipedal robot's leg movements using inverse kinematics. The system calculates servo angles for precise movements of each joint, enabling the robot to move its left and right legs to specified target coordinates. The servo motor angles are controlled using the PCA9685 PWM driver, ensuring smooth and accurate motion.

---

## Features
1. **Inverse Kinematics (IK):**
   - Calculates joint angles for both the left and right legs to move to target coordinates.
   
2. **Dual Leg Control:**
   - Supports independent control of both legs, with separate inverse kinematics calculations for each leg.

3. **Smooth Servo Motor Control:**
   - Uses the Adafruit PCA9685 PWM driver to control multiple servos at a specified frequency.

4. **Targeted Movements:**
   - Allows specifying a series of coordinates for both legs, enabling complex motion patterns.

---

## How It Works

### **1. PCA9685 Servo Control**
- **PWM Configuration:**
  - The PCA9685 module is initialized with an I2C address (`0x40`) and set to operate at a frequency of 50 Hz.
  - Servo pulse widths are mapped between `600 µs` and `2400 µs`, corresponding to angles of `0°` and `180°`.

- **Servo Mapping:**
  - Each servo's angle is mapped to a PWM pulse width:
    ```cpp
    pwm.setPWM(channel, 0, map(angle, 0, 180, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH));
    ```
  - Channels `0-4` control the left leg servos, and channels `5-9` control the right leg servos.

---

### **2. Inverse Kinematics (IK)**
- **Purpose:**
  - Calculates the angles for each joint (hip, knee, and foot) based on the desired end effector position (target coordinates).

- **Input Parameters:**
  - `targetX` and `targetY`: The desired position of the foot relative to the hip joint.

- **Output Parameters:**
  - `hipAngle`, `joint2Angle`, `joint3Angle`, `joint4Angle`, `joint5Angle`: The angles for each servo controlling the leg.

- **Separate IK Functions:**
  - `calculateIKLeft`: Calculates angles for the left leg.
  - `calculateIKRight`: Calculates angles for the right leg.

---

### **3. Target Coordinates**
- Predefined lists of target coordinates for both legs are stored as arrays:
  ```cpp
  float coordinatesLeft[][2] = {
    {2.0, 3.0},
    {5.0, 6.0},
    {8.0, 7.0}
  };

  float coordinatesRight[][2] = {
    {2.0, 3.0},
    {5.0, 6.0},
    {8.0, 7.0}
  };
- The program iterates through these coordinates, moving each leg to the specified positions sequentially.

### 4. Servo Movement
- **For Each Target Coordinate:**
  1. The `calculateIKLeft` or `calculateIKRight` function is called to determine the joint angles based on the target coordinates.
  2. The calculated angles are mapped to PWM signals using the `map()` function.
  3. The PWM signals are sent to the corresponding PCA9685 channels controlling the servos.
  4. A delay of `1000 ms` is added after each movement to ensure smooth transitions between positions.

### 5. Serial Output
- The program prints the current target coordinates for debugging and monitoring:
    Moving left leg to (2.0, 3.0)
    Moving right leg to (2.0, 3.0)

### Components Required

1. **Adafruit PCA9685 PWM Driver:**
   - Used to control multiple servo motors with precise PWM signals.

2. **Servo Motors:**
   - Controls the joints of the robot's legs for precise movement.

3. **Arduino Board:**
   - Executes the code for inverse kinematics and sends PWM signals to the PCA9685.

4. **Power Supply:**
   - External power source to power the servos and the PCA9685 module.

5. **Connecting Wires:**
   - For establishing connections between the Arduino, PCA9685, and servo motors.

### Installation and Usage

#### 1. Install Required Libraries
- Ensure the following libraries are installed in your Arduino IDE:
  - **Adafruit PWMServoDriver Library:**
    ```bash
    Arduino Library Manager > Search "Adafruit PWMServoDriver" > Install
    ```

#### 2. Wiring
- Connect the **PCA9685 module** to the I2C pins on the Arduino:
  - **SDA** to Arduino SDA.
  - **SCL** to Arduino SCL.
- Attach servo motors to the PCA9685 module's channels.
- Provide an external power supply to the PCA9685 module if required.

#### 3. Upload Code
- Copy the provided code into the Arduino IDE.
- Connect the Arduino to your computer via USB.
- Select the correct **COM Port** and **Board Type** in the Arduino IDE.
- Upload the code to the Arduino.

#### 4. Running the Program
- Once the code is uploaded, the robot will begin executing the predefined movements.
- Monitor the serial output for debugging and to track the robot's movements in real-time.

### Applications

1. **Bipedal Robot Development:**
   - Enables precise control of robot leg movements for walking or complex motions.
   
2. **Robotics Education:**
   - Demonstrates the principles of inverse kinematics and servo control for learning and experimentation.

3. **Automation Projects:**
   - Can be adapted for other robotic applications requiring multi-joint servo control.

---

### Limitations

1. **Accuracy of IK Calculations:**
   - Requires precise implementation of inverse kinematics for accurate joint movements.

2. **Physical Constraints:**
   - Servo range of motion and joint lengths must match the calculated values.

3. **Power Requirements:**
   - Multiple servos may demand a dedicated external power source to avoid power drop-offs.

---

### Future Enhancements

1. **Real-Time Control:**
   - Implement real-time IK calculations based on sensor inputs or user commands.

2. **Gait Generation:**
   - Develop algorithms to generate walking patterns for smooth bipedal motion.

3. **Feedback Integration:**
   - Add sensors to provide closed-loop control for improved accuracy and stability.
