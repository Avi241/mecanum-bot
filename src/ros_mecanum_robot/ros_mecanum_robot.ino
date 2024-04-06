#define DEBUG_ROS 0
#define DEBUG_ENCODERS 0
#define DEBUG_DRIVE 0
#define DEBUG_ODOM 1

#include "wifi.h"
#include "ota.h"
#include "ros.h"
#include "drive.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "odom.h"

#define LED_BUILTIN 2

void setup()
{
  // Status LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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
