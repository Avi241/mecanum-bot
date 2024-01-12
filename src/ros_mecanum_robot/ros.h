#include <ros.h>
#include <geometry_msgs/Twist.h>

#define LED_BUILTIN 2


bool connected = false;
bool movement = false;
float x = 0, y = 0, z = 0;

void onTwist(const geometry_msgs::Twist &msg);

// ROS serial server
ros::NodeHandle node;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

void setupROS()
{
  // Connect to rosserial socket server and init node. (Using default port of 11411)
  Serial.printf("[INFO] (ros) Connecting to ROS serial server at %s\n", server.toString().c_str());
  node.getHardware()->setConnection(server);
  node.initNode();
  node.subscribe(sub);
}

void loopROS()
{
  // If value changes, notify via LED and console.
  bool conn = node.connected();
  if (connected != conn)
  {
    connected = conn;
    if (!connected)
      stop();
      
    // digitalWrite(LED_BUILTIN, connected? HIGH : LOW);
    Serial.print("[INFO] (ros) ");
    Serial.println(connected ? "ROS connected" : "ROS disconnected");
  }
  
  node.spinOnce();
}

void stop()
{
  x = y = z = 0;
  movement = true;
}

void onTwist(const geometry_msgs::Twist &msg)
{
  if (connected)
  {
    // Cap values at [-1 .. 1]
    x = max(min(msg.linear.x, 1.0f), -1.0f);
    y = max(min(msg.linear.y, 1.0f), -1.0f);
    z = max(min(msg.angular.z, 1.0f), -1.0f);
    movement = true;

  #if DEBUG_ROS
    Serial.printf("[DEBUG] (ros) x: %f, y: %f, z: %f\n", x, y, z);
  #endif
  }
  else
    stop();
}