#line 1 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/drive.h"
extern bool movement, updating;
extern float x, y, z;
extern double LF_setpoint, RF_setpoint, LB_setpoint, RB_setpoint;

float wheel_radius = 0.152/2;
float wheel_separation_width = 0.556/2; // old 0.460
float wheel_separation_length = 0.460/2;

void loopDrive()
{
  if (!movement || updating)
    return;
  
  // Source: https://www.robotsforroboticists.com/drive-kinematics/
  LF_setpoint = (x - y - z * (wheel_separation_width + wheel_separation_length))/ wheel_radius;
  RF_setpoint = (x + y + z * (wheel_separation_width + wheel_separation_length))/ wheel_radius;
  LB_setpoint = (x + y - z * (wheel_separation_width + wheel_separation_length))/ wheel_radius;
  RB_setpoint = (x - y + z * (wheel_separation_width + wheel_separation_length))/ wheel_radius;

  #if DEBUG_DRIVE
    Serial.printf("[DEBUG] (drive) LF: %f, RF: %f, LB: %f, RB: %f\n", LF_setpoint, RF_setpoint, LB_setpoint, RB_setpoint);
  #endif
  movement = false;
}