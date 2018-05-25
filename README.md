# ros_dynamixel
This is a ROS Package developed by SRM Team Humanoid for controlling **ROBOTIS Dynamixel** smart servos and **ROBOTIS FSR Sensors**.

**Requires Robotis Dynamixel-SDK c version installed.**

Refer to Official Robotis DynamixelSDK page [here](https://github.com/ROBOTIS-GIT/DynamixelSDK#ros-packages-for-dynamixel-sdk
"DynamixelSDK") for installation instructions for DynamixelSDK


## How to Install

Clone the ROS Package into your *catkin* workspace *src* directory and run *catkin_make*

  ```bash
  cd ~/catkin_ws/src/
  git clone https://github.com/SRM-Team-Humanoid/ros_dynamixel.git
  cd .. && catkin_make
  ```


## Using ros_dynamixel package
* To start the writer node in **debug mode**, the values are only printed and not written to the actuators, set the debug parameter *true*.

  `rosrun pynamixel writer.py _debug:=true`

  **The writer node subscribes to topic */pynamixel/actuation* that uses Actuation message.**

  **To learn about the actuation message format**

  `rosmsg info pynamixel/Actuation`

* fsr_reader node publishes the *left* and *right* foot FSR sensor readings into topic */ros_dynamixel/fsr/left* and */ros_dynamixel/fsr/right* respectively.

  `rosrun ros_dynamixel fsr_reader.py`

  **These topics use FSR message. To learn about the FSR message format**

  `rosmsg info ros_dynamixel/FSR`



**This package currently supports only Dynamixel MX series servos with protocol 1.0. Support for other motors and protocol 2.0 coming soon**
