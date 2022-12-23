# hover_diffdrive

ROS2 hardware driver for UART-controlled hoverboard. base on [diffbot system](https://github.com/ros-controls/ros2_control_demos/tree/master/ros2_control_demo_hardware) from [Ros2 Control Demo](https://github.com/ros-controls/ros2_control_demos)
Buit for Ros2 [Humble](https://docs.ros.org/en/humble/index.html)

Hoverboard shoud be using modified [firmware](https://github.com/alex-makarov/hoverboard-firmware-hack-FOC) by [Emanuel Feru](https://github.com/EmanuelFeru), changed to report wheel odometry via serial protocol.


## Usage

1. Hoverboard port and Baudrate can be set in the config.h

## Dependenies
1.  The Comms use [libserial](https://github.com/crayzeewulf/libserial) use 'sudo apt install libserial-dev' to instal.

## DISCLAIMER
I bear **no responsibility** for any damage, direct or indirect, caused by using this project. Hoverboards are powerful and can be dangerous! Make sure you take all safety precautions!
