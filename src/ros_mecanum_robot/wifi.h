#include <WiFi.h>

// The name of the OTA port
#define OTA_HOSTNAME "MECANUM_BOT"

// If access point is defined, a Wifi network with this name will be created.
// Remove if you want to connect to an existing network.
// #define ACCESS_POINT_SSID "MECANUM_BOT"

#ifdef ACCESS_POINT_SSID
  // The ROS serial server address assigned by the NodeMcu access point.
  IPAddress server(192, 168, 4, 2);
#else
  // Override standard settings with user data. Example:
  const char* SSID = "Arvind";
  const char* PASSWORD = "12345678";
  IPAddress server(192, 168, 54, 40);
  // #include "user.h"
#endif


void setupWifi()
{
  // Don't write Wifi config to EEPROM to extend its lifetime. 
  WiFi.persistent(false);
  
#ifdef ACCESS_POINT_SSID

  WiFi.disconnect();
  Serial.println("[INFO] (wifi) Creating Wifi network");
  if (WiFi.softAP(ACCESS_POINT_SSID))
  {
    Serial.println("[INFO] (wifi) Wifi network created");
    Serial.print("[INFO] (wifi) SSID: ");
    Serial.println(WiFi.softAPSSID());
    Serial.print("[INFO] (wifi) IP:   ");
    Serial.println(WiFi.softAPIP());
  }

#else

  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  Serial.println("[INFO] (wifi) Connecting to Wifi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[INFO] (wifi) WiFi connected");
  Serial.print("[INFO] (wifi) SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("[INFO] (wifi) IP:   ");
  Serial.println(WiFi.localIP());

#endif
}