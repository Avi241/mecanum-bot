#include <WiFi.h>
#include "encoder.h"
#include "motor.h"
#include "pid.h"


int serialTime=0;
double setpoint=0.3;


void setup()
{
	Serial.begin(115200);

	setupEncoders();
	setupMotor();
	setupPID();

	// Wait for enter key to be pressed
	while (Serial.available() <= 0) {
		Serial.println("Press enter to start PID tuning");
		delay(500);
	}

	RF_setpoint = -0.5;
	LF_setpoint = -0.5;
	LB_setpoint = -0.5;
	RB_setpoint = -0.5;
}


void loop()
{
	loopEnc();
	loopMotor();
	loopPID();

	if(millis()>serialTime)
	{
		// Serial.print("setpoint: ");
		// Serial.print(setpoint);
		// Serial.print(" RF_vel: ");
		// Serial.print(RF_vel);
		// Serial.print(" RF: ");
		// Serial.println(RF);

		// Serial.print("setpoint: ");
		// Serial.print(setpoint);
		// Serial.print(" LF_vel: ");
		// Serial.print(LF_vel);
		// Serial.print(" LF: ");
		// Serial.println(LF);

		Serial.print(RF_pos);
		Serial.print(" ");
		Serial.print(LF_pos);
		Serial.print(" ");
		Serial.print(RB_pos);
		Serial.print(" ");
		Serial.println(LB_pos);

		serialTime+=500;
	}
}
