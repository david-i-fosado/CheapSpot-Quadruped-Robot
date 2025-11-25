# 🤖 CheapSpot – Low-Cost Quadruped Robot

CheapSpot is a low-cost quadruped robot inspired by Spot Mini, built using a Raspberry Pi, Arduino, PCA9685, servomotors, 3D-printed parts, and an IMU sensor.
The system supports teleoperation and autonomous navigation with image-based visual servoing (IBVS).

---

## Key Features
✔️ 12-servo control using Arduino + PCA9685
✔️ Main computation on Raspberry Pi (ROS Noetic)
✔️ USB serial communication between Raspberry Pi ↔ Arduino
✔️ Teleoperation using a joystick (PS4)
✔️ Fuzzy PID orientation controller
✔️ IBVS visual control using a camera and Apriltags
✔️ Fully 3D-printable structure

---

## Hardware Used
- Raspberry Pi 4
- Arduino Uno
- PCA9685 16-channel PWM driver
- 12 Servomotors (PDI-HV5523MG)
- IMU (MPU9250)
- LiPo 2s battery
- USB camera (LogitechC920)
- 3D-printed mechanical parts (👉: https://www.printables.com/model/145053-spotmicro-v2)

---

## Software Stack
- ROS Noetic
- Python 3
- Arduino IDE
- OpenCV
- Apriltag ROS

## Documentation
Official document: 

## Video
Project demonstration:
[![Video](https://img.youtube.com/vi/dBfq67AseLw/hqdefault.jpg)](https://youtu.be/dBfq67AseLw)
