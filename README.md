# 🤖 PID-Based Self-Balancing Robot with Bluetooth Control and Robotic Arm

An advanced two-wheeled self-balancing robot based on the inverted pendulum principle. The system uses PID control for stability and is equipped with Bluetooth-based mobile control and a 2-DOF robotic arm for interaction.

---

## 🚀 Features
- Real-time balancing using PID control
- Bluetooth control via HC-05 module
- Mobile app-based movement and arm control
- 300 RPM N20 motors for precise motion
- 2-DOF servo-based robotic arm
- Stable power system using buck converter

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
- Forward / Backward movement  
- Balance adjustment  
- Arm manipulation  

---

## 🧠 Working Principle
The MPU6050 sensor continuously measures the tilt angle of the robot.

A PID controller processes this data and calculates the required correction.  
The Arduino Nano then adjusts motor speed through the L298N driver to maintain balance.

Simultaneously, Bluetooth commands allow the user to control movement and operate the robotic arm.

---

## ⚙️ Control Algorithm (PID)
- **Proportional (P):** Corrects present error  
- **Integral (I):** Eliminates accumulated error  
- **Derivative (D):** Reduces overshoot and stabilizes motion  

---

## 🔋 Power Management
- Buck converter regulates voltage for stable operation  
- Protects components from voltage fluctuations  

---

## 📌 Future Improvements
- Replace HC-05 with BLE (ESP32-based control)  
- Add camera for vision-based control  
- Implement obstacle detection  
- Auto-tuning PID algorithm  

---

## 👨‍💻 Author
Anuj Tale
