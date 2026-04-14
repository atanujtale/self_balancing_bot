# 🤖 LQR-Based Self-Balancing Robot with Bluetooth Control and Robotic Arm

An advanced two-wheeled self-balancing robot based on the inverted pendulum model, using Linear Quadratic Regulator (LQR) control for optimal stability. The system is integrated with Bluetooth-based mobile control and a 2-DOF robotic arm.

---

## 🚀 Features
- Optimal control using LQR (Linear Quadratic Regulator)
- Real-time balancing of inverted pendulum system
- Bluetooth control via HC-05 module
- Mobile app-based navigation and arm control
- 300 RPM N20 motors for smooth motion
- 2-DOF servo-based robotic arm
- Efficient power management using buck converter

---

## 🛠️ Hardware Components
- Arduino Nano  
- MPU6050 (Gyroscope + Accelerometer)  
- N20 DC Motors (300 RPM)  
- L298N Motor Driver  
- HC-05 Bluetooth Module  
- Servo Motors (2x for robotic arm)  
- Buck Converter  
- Li-ion Battery  

---

## 📱 Bluetooth Control
The robot is controlled using a mobile application via the HC-05 Bluetooth module:
- Forward / Backward motion  
- Direction control  
- Arm manipulation  

---

## 🧠 Working Principle
The system models the robot as an inverted pendulum.

The MPU6050 measures tilt angle and angular velocity.  
These states are fed into an LQR controller, which computes the optimal control input to stabilize the system.

The Arduino Nano applies this control signal to the motors via the L298N driver.

Bluetooth commands allow external control of motion and robotic arm operations.

---

## ⚙️ Control Algorithm (LQR)
The control law is given by:

u = -Kx

Where:
- **x** → state vector (angle, angular velocity, position, velocity)  
- **K** → gain matrix computed using LQR  
- **u** → control input (motor command)

LQR minimizes a cost function to achieve optimal stability and performance.

---

## 📊 System Model
- Based on inverted pendulum dynamics  
- State-space representation used  
- Controller designed using optimal control theory  

---

## 🔋 Power Management
- Buck converter ensures stable voltage supply  
- Protects microcontroller and sensors  

---

## 📸 Demo
Demonstration - https://youtu.be/BZcSurQdnyc

---

## 📌 Future Improvements
- Implement Kalman Filter for better state estimation  
- Replace Arduino with ESP32 for higher computation  
- Autonomous navigation  
- Vision-based object handling  

---

## 👨‍💻 Author
Anuj Tale(Team Leader),
Aditya Tangde,
Prajwal Sontakke.
