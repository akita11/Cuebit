
#include <Arduino.h>
#include "Wire.h"
#include "veml6040.h"
#include <Adafruit_NeoPixel.h>

#include "peripheral.h"

// https://github.com/thewknd/VEML6040/
VEML6040 RGBWSensor;

#define R_B    5 // PD5
#define R_A    6 // PD6
#define L_A    9 // PB1
#define L_B    10 // PB2
#define LEDC   3 // for WS2812
#define LED_EN 2 // PD2
#define PD_R2  A3
#define PD_R1  A2
#define PD_L1  A1
#define PD_L2  A0

// for wide sensor
#define BLACK_PD_R2 36
#define WHITE_PD_R2 360
#define BLACK_PD_R1 42
#define WHITE_PD_R1 342
#define BLACK_PD_L1 45
#define WHITE_PD_L1 400
#define BLACK_PD_L2 35
#define WHITE_PD_L2 350
#define BLACK_COLOR 11000
#define WHITE_COLOR 20000

#define MAX_PWM_L 255
#define MAX_PWM_R 255

Adafruit_NeoPixel pixel(2, LEDC, NEO_GRB + NEO_KHZ800);

void init_peripheral()
{
  pixel.begin(); pixel.clear();
	pinMode(LED_EN, OUTPUT);
	setLED(20, 0, 0);
	setMotorSpeed(0.0, 0.0);
	enableSensorLED(0);
  Wire.begin(); 
	RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  if(!RGBWSensor.begin()) {
    Serial.println("VEML6040 init error");
    while(1){ setLED(20, 0, 0); delay(100); setLED(0, 0, 0); delay(100); }
  }
}

void enableSensorLED(uint8_t f)
{
	if (f == 1) digitalWrite(LED_EN, HIGH);
	else digitalWrite(LED_EN, LOW);
}

uint16_t convMotorPWM(uint8_t max_pwm, float v){
	if (v > 1.0) return(max_pwm);
	else return((uint16_t)(max_pwm * v));
}

// vL/vR : -1 - +1 / +=FWD, -=BWD
void setMotorSpeed(float vL, float vR)
{
	if (vL > 0){
		analogWrite(L_A, convMotorPWM(MAX_PWM_L, vL));	analogWrite(L_B, 0);
	}
	else{
		analogWrite(L_A, 0); analogWrite(L_B, -convMotorPWM(MAX_PWM_L, vL));
	}
	if (vR > 0){
		analogWrite(R_A, convMotorPWM(MAX_PWM_R, vR * LRratio));	analogWrite(R_B, 0);
	}
	else{
		analogWrite(R_A, 0); analogWrite(R_B, -convMotorPWM(MAX_PWM_R, vR * LRratio));
	}
}

void setLED(uint8_t r, uint8_t g, uint8_t b)
{
	pixel.setPixelColor(0,  pixel.Color(r, g, b)); pixel.setPixelColor(1,  pixel.Color(r, g, b)); pixel.show();
}

uint8_t classify(float R, float G, float B, float W) {
	float weights[5][5] = {
		{-0.215193,  0.158631,  0.447248,  0.000700, -24.586008},
		{-0.478091, -0.265669,  0.986931, -0.001539,  21.651707},
		{ 0.355452, -0.928157, -0.255726,  0.000547,  32.428762},
		{-0.102398,  0.391205, -0.465644, -0.000426,  10.497819},
		{ 0.440230,  0.643990, -0.712809,  0.000718, -39.992280}
	};
	float input[4] = {R, G, B, W};
	float scores[5];

	for (int c = 0; c < 5; c++) {
		scores[c] = weights[c][4]; // bias
		for (int i = 0; i < 4; i++) {
			scores[c] += input[i] * weights[c][i];
		}
	}

	int maxIndex = 0;
	for (int c = 1; c < 5; c++) {
		if (scores[c] > scores[maxIndex]) {
			maxIndex = c;
		}
	}

	if (maxIndex == 0) return(COLOR_WHITE);
	else if (maxIndex == 1) return(COLOR_BLACK);
	else if (maxIndex == 2) return(COLOR_RED);
	else if (maxIndex == 3) return(COLOR_GREEN);
	else return(COLOR_BLUE);
}

// BLACK(1.0) - WHITE(0.0)
float lineValue(uint16_t val, uint16_t vBlack, uint16_t vWhite){
	float v;
	if (val < vBlack) v = 1.0;
	else if (val > vWhite) v = 0.0;
	else v = (float)(vWhite - val) / (float)(vWhite - vBlack);
	return(v);
}

SensorData readSensor(SensorData sd)
{
	// return: L2:L1:C:R1:R2:{COLOR[2:0]} (1=line detected)
	uint8_t sensorInfo = COLOR_NONE;
	uint16_t sensorR, sensorG, sensorB, sensorW;

	enableSensorLED(1);
//	delay(2);

	float s = 0.0;
	float v;
	sd.line = 0.0;
	v = lineValue(analogRead(PD_L2), BLACK_PD_L2, WHITE_PD_L2); sd.line += v * (-2); s += v;
	v = lineValue(analogRead(PD_L1), BLACK_PD_L1, WHITE_PD_L1); sd.line += v * (-1); s += v;
	v = lineValue(analogRead(PD_R1), BLACK_PD_R1, WHITE_PD_R1); sd.line += v * (+1); s += v;
	v = lineValue(analogRead(PD_R2), BLACK_PD_R2, WHITE_PD_R2); sd.line += v * (+2); s += v;

	sensorR = RGBWSensor.getRed();
	sensorG = RGBWSensor.getGreen();
	sensorB = RGBWSensor.getBlue();
	sensorW = RGBWSensor.getWhite();
//	enableSensorLED(0);

	// normalize
	float sensorRf, sensorGf, sensorBf;
	sensorRf = (float)sensorR / (float)sensorW * 100.0;
	sensorGf = (float)sensorG / (float)sensorW * 100.0;
	sensorBf = (float)sensorB / (float)sensorW * 100.0;

/*
	if (sensorW > WHITE_COLOR) sensorInfo = COLOR_WHITE;
	else sensorInfo = classify(sensorRf, sensorGf, sensorBf, sensorW);
*/
	sensorInfo = classify(sensorRf, sensorGf, sensorBf, sensorW); // model including W 
	sd.color = sensorInfo;
	if (stLEDmsg == LEDMSG_NONE){
		if (sensorInfo == COLOR_BLACK) setLED(0, 0, 0);
		else if (sensorInfo == COLOR_RED) setLED(10, 0, 0);
		else if (sensorInfo == COLOR_GREEN) setLED(0, 10, 0);
//		else if (sensorInfo == COLOR_BLUE) setLED(0, 0, 10); // blue (old)
		else if (sensorInfo == COLOR_BLUE) setLED(6, 6, 0); // yellow
		else if (sensorInfo == COLOR_WHITE) setLED(10, 10, 10);
	}
	
	if (sensorInfo != COLOR_WHITE) s += lineValue(sensorW, BLACK_COLOR, WHITE_COLOR);	
//	if (s == 0.0) sd.line = -10.0;
	if (s <  0.3) sd.line = -10.0;
	else sd.line = sd.line / s;
	sd.width = s;
	if (fDebug > 0){
	 	Serial.print(sensorRf); Serial.print(",");
	 	Serial.print(sensorGf); Serial.print(",");
	 	Serial.print(sensorBf); Serial.print(",");
	 	Serial.print(sensorW); Serial.print(",");
		if (fDebug == 6){
	 		Serial.print("|,");
		 	Serial.print(analogRead(PD_R2)); Serial.print(',');
		 	Serial.print(analogRead(PD_R1)); Serial.print(',');
	 		Serial.print(analogRead(PD_L1)); Serial.print(',');
	 		Serial.print(analogRead(PD_L2)); Serial.print(",|,");
			Serial.print(lineValue(analogRead(PD_R2), BLACK_PD_R2, WHITE_PD_R2)); Serial.print(',');
			Serial.print(lineValue(analogRead(PD_R1), BLACK_PD_R1, WHITE_PD_R1)); Serial.print(',');
			Serial.print(lineValue(analogRead(PD_L1), BLACK_PD_L1, WHITE_PD_L1)); Serial.print(',');
			Serial.print(lineValue(analogRead(PD_L2), BLACK_PD_L2, WHITE_PD_L2)); Serial.print(',');
			if (sensorInfo != COLOR_WHITE) Serial.print(lineValue(sensorW, BLACK_COLOR, WHITE_COLOR));
			Serial.print(",|,"); Serial.print(sd.line);
			Serial.print(','); Serial.print(sd.width);
			Serial.print('/'); Serial.print(sd.color);
//			Serial.print('|'); Serial.print(vL); Serial.print(','); Serial.println(vR);
			Serial.print(','); Serial.print(BatteryVoltage);
			Serial.println("");
		}
		else if (fDebug >= 1 && fDebug <= 5) Serial.println(fDebug); // with color label
	}
	return(sd);
}
