# 1 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino"





# 7 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2
# 8 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2
# 9 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2
# 10 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2
# 11 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2
# 12 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2
# 13 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2
# 14 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ros_mecanum_robot.ino" 2



void setup()
{
  // Status LED
  pinMode(2, 0x03);
  digitalWrite(2, 0x0);

  Serial.begin(115200);
  Serial.println();

  setupWifi();
  setupOTA();
  setupROS();
  setupEncoders();
  setupPID();
  setupMotor();
}

void loop()
{
  loopOTA();
  loopROS();
  loopDrive();
  loopEncoders();
  loopPID();
  loopMotor();
  loopOdom();
}
