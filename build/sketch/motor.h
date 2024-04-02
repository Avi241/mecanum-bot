#line 1 "/home/indrones/Distroboxes/ubuntu18/catkin_ws/src/mecanum-bot/src/ros_mecanum_robot/motor.h"
const int motor_rate = 10;
const int pwm_deadzone = 5;

const uint8_t LF_DIR = 19;
const uint8_t LF_PWM = 21;

const uint8_t RF_DIR = 22;
const uint8_t RF_PWM = 23;

const uint8_t LB_DIR = 18;
const uint8_t LB_PWM = 17;

const uint8_t RB_DIR = 14;
const uint8_t RB_PWM = 27;

double LF_pwm = 0, RF_pwm = 0, LB_pwm = 0, RB_pwm = 0;
int _LF_pwm = 0, _RF_pwm = 0, _LB_pwm = 0, _RB_pwm = 0;
int LF_last_update = 0, RF_last_update = 0, LB_last_update = 0, RB_last_update = 0;

void setupMotor(){
	pinMode(LF_DIR, OUTPUT);
	pinMode(LF_PWM, OUTPUT);

	pinMode(RF_DIR, OUTPUT);
	pinMode(RF_PWM, OUTPUT);

	pinMode(LB_DIR, OUTPUT);
	pinMode(LB_PWM, OUTPUT);

	pinMode(RB_DIR, OUTPUT);
	pinMode(RB_PWM, OUTPUT);
}

void loopMotor(){
	if (millis() - LF_last_update > 1000/motor_rate && LF_pwm != _LF_pwm)
	{
		LF_last_update = millis();
		_LF_pwm = LF_pwm;
		digitalWrite(LF_DIR, _LF_pwm > 0 ? HIGH : LOW);
		if (abs(_LF_pwm) < pwm_deadzone) {
			analogWrite(LF_PWM, 0);
		}
		else {
			analogWrite(LF_PWM, abs(_LF_pwm));
		}
	}
	if (millis() - RF_last_update > 1000/motor_rate && RF_pwm != _RF_pwm)
	{
		RF_last_update = millis();
		_RF_pwm = RF_pwm;
		digitalWrite(RF_DIR, _RF_pwm > 0 ? HIGH : LOW);
		if (abs(_RF_pwm) < pwm_deadzone) {
			analogWrite(RF_PWM, 0);
		}
		else {
			analogWrite(RF_PWM, abs(_RF_pwm));
		}
	}
	if (millis() - LB_last_update > 1000/motor_rate && LB_pwm != _LB_pwm)
	{
		LB_last_update = millis();
		_LB_pwm = LB_pwm;
		digitalWrite(LB_DIR, _LB_pwm > 0 ? LOW : HIGH);
		if (abs(_LB_pwm) < pwm_deadzone) {
			analogWrite(LB_PWM, 0);
		}
		else {
			analogWrite(LB_PWM, abs(_LB_pwm));
		}
	}
	if (millis() - RB_last_update > 1000/motor_rate && RB_pwm != _RB_pwm)
	{
		RB_last_update = millis();
		_RB_pwm = RB_pwm;
		digitalWrite(RB_DIR, _RB_pwm > 0 ? LOW : HIGH);
		if (abs(_RB_pwm) < pwm_deadzone) {
			analogWrite(RB_PWM, 0);
		}
		else {
			analogWrite(RB_PWM, abs(_RB_pwm));
		}
	}
}