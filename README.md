# Mecanum Bot
The code for a robot with mecanum wheels and can be control by sending ROS Twist messages.

## Robot Specifications
The robot chasis was build using [this kit](https://robu.in/product/easymech-152-mm-mecanum-wheel-basic-shock-absorber-smart-car-robot-chassis/) from [robu.in](https://robu.in). The robot is controlled using a ESP32-WROOM (38-Pin).

## Firmware
The [firmware](./src/ros_mecanum_robot/) is written in C++ and uses the Arduino framework and should be flashed to the ESP32-WROOM using the Arduino IDE.

## ROS Node
This repo is also a ROS package that contains a ROS node that can be used to bring up the robot. The node is written in Python and uses the [rosserial](http://wiki.ros.org/rosserial) package to communicate with the ESP32-WROOM. The node can be started using the following command:
```bash
roslaunch mecanumbot bringup.launch
```

The robot can be controlled by publishing to the `/cmd_vel` topic. The message type is `geometry_msgs/Twist`. 

To control the robot using a joystick / gamepad, the `joystick` parameter can be set to `true` while launching the node. This then uses the `joy` package to subscribe to the `/joy` topic and convert the joystick messages to `Twist` messages.

## To Do
- [ ] Publish odometry by using reverse kinematics

## Credits
The firmware is based on the [ros_mecanum_robot](https://github.com/ModernOctave/mecanum-bot) by [Reinbert](https://github.com/Reinbert).