#include <WiFi.h>

const uint8_t LF_DIR = 19;
const uint8_t LF_PWM = 21;

const uint8_t RF_DIR = 22;
const uint8_t RF_PWM = 23;

const uint8_t LB_DIR = 18;
const uint8_t LB_PWM = 17;

const uint8_t RB_DIR = 14;
const uint8_t RB_PWM = 27;

int pwm_val = 150;

void setup()
{
	pinMode(LF_DIR, OUTPUT);
  pinMode(LF_PWM, OUTPUT);

  pinMode(RF_DIR, OUTPUT);
  pinMode(RF_PWM, OUTPUT);

  pinMode(LB_DIR, OUTPUT);
  pinMode(LB_PWM, OUTPUT);

  pinMode(RB_DIR, OUTPUT);
  pinMode(RB_PWM, OUTPUT);
}

void loop()
{
	  digitalWrite(LF_DIR, HIGH);
    analogWrite(LF_PWM, 100);
//    digitalWrite(RF_DIR,HIGH);
//    analogWrite(RF_PWM, abs(_RF_pwm));
//    digitalWrite(LB_DIR, LOW);
//    analogWrite(LB_PWM, abs(_LB_pwm));
//    digitalWrite(RB_DIR, LOW);
//    analogWrite(RB_PWM, abs(_RB_pwm));
}
