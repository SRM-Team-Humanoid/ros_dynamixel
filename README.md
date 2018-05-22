# Pynamixel
This is ROS Package developed by SRM Team Humanoid for controlling **ROBOTIS Dynamixel** smart servos and **ROBOTIS FSR Sensors**.

**Requires Robotis Dynamixel-SDK c version installed.**

Refer to Official Robotis DynamixelSDK page [here](https://github.com/ROBOTIS-GIT/DynamixelSDK#ros-packages-for-dynamixel-sdk
"DynamixelSDK") for installation instructions for DynamixelSDK

---
## How to Install

Clone the ROS Package into your *catkin* workspace *src* directory and run *catkin_make*

  ```bash
  cd ~/catkin_ws/src/
  git clone https://github.com/SRM-Team-Humanoid/pynamixel.git
  cd .. && catkin_make
  ```


## Using Pynamixel package
>To start the writer node in **debug mode**, the values are only printed and not written to the actuators, set the debug parameter *true*.

  `rosrun pynamixel writer.py _debug:=true`

**The writer node subscribes to topic */pynamixel/actuation* that uses Actuation message.**

**To learn about the actuation message format**

  `rosmsg info pynamixel/Actuation`

>fsr_reader node publishes the *left* and *right* foot FSR sensor readings into topic */pynamixel/fsr/left* and */pynamixel/fsr/right* respectively.

`rosrun pynamixel fsr_reader.py`

**These topics use FSR message. To learn about the FSR message format**

`rosmsg info pynamixel/FSR`



**Currently only supports Dynamixel MX series servos with protocol 1.0. Support for other motors and protocol 2.0 coming soon**
