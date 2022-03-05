# Jetbot Landmark Navigation with ROS

## Objective
Use the on-board camera to drive to specific locations in the environment. 

## File Structure
```
|-- ROOT
  |-- CMakeLists.txt
  |-- init.sh
  |-- jetbot_ros
  |   |-- CMakeLists.txt
  |   |-- package.xml
  |   |-- README.md
  |   |-- gazebo
  |       |-- install_jetbot_model.sh
  |       |-- jetbot_gazebo_0.png
  |       |-- README.md
  |       |-- jetbot
  |           |-- model.config
  |           |-- model.sdf
  |           |-- meshes
  |               |-- JetBot-v3-Chassis.dae
  |               |-- JetBot-v3-Chassis.stl
  |               |-- JetBot-v3-Wheel.stl
  |   |-- scripts
  |       |-- camera_info_publisher.py
  |       |-- camera_intrinsics.yaml
  |       |-- control_test.py
  |       |-- jetbot_control.py
  |       |-- jetbot_motors.py
  |       |-- jetbot_oled.py
  |       |-- teleop_joy.py
  |       |-- teleop_key.py
  |   |-- src
  |       |-- image_converter.cpp
  |       |-- image_converter.h
  |       |-- jetbot_camera.cpp
  |       |-- ros_compat.cpp
  |       |-- ros_compat.h
  
  |-- jetson-inference
  |-- navigation_dev
  |-- ros_deep_learning
```


## Report
[276A_HW2_report .pdf](https://github.com/lmqZach/276_hw2_code/files/8190019/276A_HW2_report.pdf)

