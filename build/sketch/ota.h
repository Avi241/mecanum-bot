#line 1 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/ota.h"
#include <ArduinoOTA.h>


bool updating = false;

extern void stop();
extern void loopDrive();

void setupOTA()
{
  ArduinoOTA.onStart([]() {
    // Stop and handle it immediately, then set updating to true to prevent further commands.
    stop();
    loopDrive();
    updating = true;
  });
  
  ArduinoOTA.onEnd([]() {
    updating = false;
    // digitalWrite(LED_BUILTIN, HIGH);
  });

  // Blink the progress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // digitalWrite(LED_BUILTIN, (100 * progress / total) % 2); // false -> on, true -> off
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();
  Serial.printf("OTA ready\n");
}

void loopOTA()
{
  ArduinoOTA.handle();
}