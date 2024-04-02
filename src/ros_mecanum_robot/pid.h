#include <PID_v1.h>


double kp=50,ki=0,kd=0;
double LF_setpoint = 0, RF_setpoint = 0, LB_setpoint = 0, RB_setpoint = 0;

PID *RF_PID, *LF_PID, *LB_PID, *RB_PID;

extern double LF_vel, RF_vel, LB_vel, RB_vel;
extern double LF_pwm, RF_pwm, LB_pwm, RB_pwm;

void setupPID(){
	RF_PID = new PID(&RF_vel, &RF_pwm, &RF_setpoint, kp, ki, kd, DIRECT);
	LF_PID = new PID(&LF_vel, &LF_pwm, &LF_setpoint, kp, ki, kd, DIRECT);
	LB_PID = new PID(&LB_vel, &LB_pwm, &LB_setpoint, kp, ki, kd, DIRECT);
	RB_PID = new PID(&RB_vel, &RB_pwm, &RB_setpoint, kp, ki, kd, DIRECT);

  	RF_PID->SetMode(AUTOMATIC);
	RF_PID->SetOutputLimits(-255,255);

	LF_PID->SetMode(AUTOMATIC);
	LF_PID->SetOutputLimits(-255,255);

	LB_PID->SetMode(AUTOMATIC);
	LB_PID->SetOutputLimits(-255,255);

	RB_PID->SetMode(AUTOMATIC);
	RB_PID->SetOutputLimits(-255,255);
}

void loopPID(){
	RF_PID->Compute();
	LF_PID->Compute();
	LB_PID->Compute();
	RB_PID->Compute();
}