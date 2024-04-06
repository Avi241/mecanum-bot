#include <ESPRotary.h>

const int enc_rate = 10;

const uint8_t LF_ENA = 39;
const uint8_t LF_ENB = 36;

const uint8_t RF_ENA = 26;
const uint8_t RF_ENB = 25;

const uint8_t LB_ENA = 33;
const uint8_t LB_ENB = 32;

const uint8_t RB_ENA = 35;
const uint8_t RB_ENB = 34;

ESPRotary LF_ENC, RF_ENC, LB_ENC, RB_ENC;
int LF_pos = 0, RF_pos = 0, LB_pos = 0, RB_pos = 0;
double LF_vel = 0, RF_vel = 0, LB_vel = 0, RB_vel = 0;
int LF_time = 0, RF_time = 0, LB_time = 0, RB_time = 0;

void updateLF(ESPRotary& r)
{
	int time = millis();
	int pos = r.getPosition();
	LF_vel = (float) (pos - LF_pos) / (time - LF_time)*6.283185;
	LF_pos = pos;
	LF_time = time;
}

void updateRF(ESPRotary& r)
{
	int time = millis();
	int pos = r.getPosition();
	RF_vel = (float) (pos - RF_pos) / (time - RF_time)*-6.283185;
	RF_pos = pos;
	RF_time = time;
}

void updateLB(ESPRotary& r)
{
	int time = millis();
	int pos = r.getPosition();
	LB_vel = (float) (pos - LB_pos) / (time - LB_time)*6.283185;
	LB_pos = pos;
	LB_time = time;
}

void updateRB(ESPRotary& r)
{
	int time = millis();
	int pos = r.getPosition();
	RB_vel = (float) (pos - RB_pos) / (time - RB_time)*-6.283185;
	RB_pos = pos;
	RB_time = time;
}


void setupEncoders(){
	LF_ENC.begin(LF_ENA, LF_ENB, 7);
	RF_ENC.begin(RF_ENA, RF_ENB, 7);
	LB_ENC.begin(LB_ENA, LB_ENB, 7);
	RB_ENC.begin(RB_ENA, RB_ENB, 7);

	// LF_ENC.setChangedHandler(updateLF);
	// RF_ENC.setChangedHandler(updateRF);
	// LB_ENC.setChangedHandler(updateLB);
	// RB_ENC.setChangedHandler(updateRB);
}

void loopEnc(){
	LF_ENC.loop();
	RF_ENC.loop();
	LB_ENC.loop();
	RB_ENC.loop();

	if (millis() - LF_time > 1000/enc_rate)
	{
		updateLF(LF_ENC);
	}
	if (millis() - RF_time > 1000/enc_rate)
	{
		updateRF(RF_ENC);
	}
	if (millis() - LB_time > 1000/enc_rate)
	{
		updateLB(LB_ENC);
	}
	if (millis() - RB_time > 1000/enc_rate)
	{
		updateRB(RB_ENC);
	}
}
