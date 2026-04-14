🤖 Self-Balancing Robot using LQR Control
📌 Overview

This project demonstrates a self-balancing robot (inverted pendulum system) stabilized using a Linear Quadratic Regulator (LQR). The robot maintains its upright position by continuously adjusting motor inputs based on real-time sensor feedback.

The system is designed to showcase advanced control theory applied to embedded systems, making it ideal for robotics and control engineering applications.

🚀 Features
🧠 Advanced LQR control algorithm
⚖️ Real-time balancing (inverted pendulum model)
🔄 Closed-loop feedback system
📉 State-space modeling
🎯 Precise angle stabilization using IMU
🔌 Efficient motor control using driver module

🛠️ Hardware Components
Microcontroller: Arduino Nano
Motor Driver: L298N
Motors: N20 DC Gear Motors (300 RPM)
IMU Sensor: MPU6050 (Gyroscope + Accelerometer)
Power Supply: Li-ion Batteries + Buck Converter
Chassis + Wheels

🧮 Control Strategy (LQR)
The system is modeled as an inverted pendulum, and LQR is used to compute optimal control input.

State Variables:
θ → Tilt angle
θ̇ → Angular velocity
x → Position
ẋ → Linear velocity
Objective:

LQR computes gain matrix K, and control input is:
u=−Kx

🔌 Working Principle
MPU6050 measures tilt angle and angular velocity
Sensor data is processed using filtering (e.g., complementary filter)
State vector is calculated
LQR computes optimal control signal
Motor driver adjusts wheel speed
Robot stabilizes itself continuously
🧑‍💻 Software Requirements
Arduino IDE
Embedded C/C++
Libraries:
Wire.h
MPU6050 library

📷 Project Demonstration
https://www.youtube.com/watch?v=BZcSurQdnyc

📊 Results
Stable balancing achieved within small angle deviations
Fast response with minimal oscillations
Improved performance over PID in dynamic conditions

🔮 Future Improvements
Add Kalman Filter for better estimation
Implement wireless control (Bluetooth/WiFi)
Use high torque motors with encoders
Upgrade to STM32 / ESP32 for faster computation

🏆 Achievements
Developed as part of e-Yantra Robotics Competition
Achieved AIR 5 (Team Lead)

👤 Author
Anuj Tale,
Aditya Tangde,
Prajwal Sontakke

Electronics & Telecommunication Engineering
SGGSIE&T, Nanded

📜 License

This project is open-source and available under the MIT License.
