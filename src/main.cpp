#include <Arduino.h>
#include "Wire.h"
#include "veml6040.h"
#include <Adafruit_NeoPixel.h>
#include <MsTimer2.h>

// https://github.com/thewknd/VEML6040/
VEML6040 RGBWSensor;

#define R_B 5 // PD5
#define R_A 6 // PD6
#define L_A 9 // PB1
#define L_B 10 // PB2
#define LEDC 3 // for WS2812
#define LED_EN 2 // PD2
#define PD_R2 A3
#define PD_R1 A2
#define PD_L1 A1
#define PD_L2 A0

//#define DEBUG_COLOR

#define PD_THRESHOLD 600   // threshold for photo diode

Adafruit_NeoPixel pixel(1, LEDC, NEO_GRB + NEO_KHZ800);

void enableSensorLED(uint8_t f)
{
	if (f == 1) digitalWrite(LED_EN, HIGH);
	else digitalWrite(LED_EN, LOW);
}

#define MAX_PWM_L 255
#define MAX_PWM_R 255

uint16_t convMotorPWM(uint8_t max_pwm, float v){
	return((uint16_t)(max_pwm * v));
}

// vL/vR : -1 - +1 / +=FWD, -=BWD
void setMotorSpeed(float vL, float vR)
{
	if (vL > 0){
		analogWrite(L_A, 0); analogWrite(L_B, convMotorPWM(MAX_PWM_L, vL));
	}
	else{
		analogWrite(L_A, -convMotorPWM(MAX_PWM_L, vL));	analogWrite(L_B, 0);
	}
	if (vR > 0){
		analogWrite(R_A, 0); analogWrite(R_B, convMotorPWM(MAX_PWM_R, vR));
	}
	else{
		analogWrite(R_A, -convMotorPWM(MAX_PWM_R, vR));	analogWrite(R_B, 0);
	}
}

#define COLOR_NONE	0
#define COLOR_WHITE 1
#define COLOR_BLACK 2
#define COLOR_RED	  3
#define COLOR_GREEN 4
#define COLOR_BLUE  5

void setLED(uint8_t r, uint8_t g, uint8_t b)
{
	pixel.setPixelColor(0,  pixel.Color(r, g, b)); pixel.show();
}

uint8_t classify(float R, float G, float B) {
    float coef[4][3] = {
			{1.244, -0.525, -0.113},
			{-0.723, 0.737, -0.366},
			{-0.051, -0.199, 0.314},
			{-0.0012, 0.0968, -0.163}
    };
    
    float intercepts[4] = {-28.665, 7.005, -2.222, 2.364};

    float decision_values[4];
    for (int i = 0; i < 4; i++) {
        decision_values[i] = coef[i][0] * R + coef[i][1] * G + coef[i][2] * B + intercepts[i];
    }

    int max_index = 0;
    for (int i = 1; i < 4; i++) {
        if (decision_values[i] > decision_values[max_index]) {
            max_index = i;
        }
    }
		if (max_index == 0) return(COLOR_RED);
		else if (max_index == 1) return(COLOR_GREEN);
		else if (max_index == 2) return(COLOR_BLUE);
		else return(COLOR_BLACK);
}

uint8_t readSensor()
{
	// return: L2:L1:C:R1:R2:{COLOR[2:0]} (1=line detected)
	uint8_t sensorInfo = COLOR_NONE;
	uint16_t sensorR, sensorG, sensorB, sensorW;

	enableSensorLED(1);
	delay(2);

	uint8_t posLine = 0;
	if (analogRead(PD_L2) < PD_THRESHOLD) posLine |= 0x80;
	if (analogRead(PD_L1) < PD_THRESHOLD) posLine |= 0x40;
	if (analogRead(PD_R1) < PD_THRESHOLD) posLine |= 0x10;
	if (analogRead(PD_R2) < PD_THRESHOLD) posLine |= 0x08;

	sensorR = RGBWSensor.getRed();
	sensorG = RGBWSensor.getGreen();
	sensorB = RGBWSensor.getBlue();
	sensorW = RGBWSensor.getWhite();
	enableSensorLED(0);

	// normalize
	float sensorRf, sensorGf, sensorBf;
	sensorRf = (float)sensorR / (float)sensorW * 100.0;
	sensorGf = (float)sensorG / (float)sensorW * 100.0;
	sensorBf = (float)sensorB / (float)sensorW * 100.0;

	if (sensorW > 29000) sensorInfo = COLOR_WHITE;
	else sensorInfo = classify(sensorRf, sensorGf, sensorBf);

  if ((sensorInfo & 0x07) != COLOR_WHITE) posLine |= 0x20; 
	if (sensorInfo == COLOR_BLACK) setLED(0, 0, 0);
	else if (sensorInfo == COLOR_RED) setLED(20, 0, 0);
	else if (sensorInfo == COLOR_GREEN) setLED(0, 20, 0);
	else if (sensorInfo == COLOR_BLUE) setLED(0, 0, 20);
	else if (sensorInfo == COLOR_WHITE) setLED(20, 20, 20);
#ifdef DEBUG_COLOR
	Serial.print(sensorInfo, HEX); Serial.print(":");
 	Serial.print(sensorRf); Serial.print(",");
 	Serial.print(sensorGf); Serial.print(",");
 	Serial.print(sensorBf); Serial.print(",");
 	Serial.print(sensorW); Serial.print(",");
 	Serial.print(analogRead(PD_R2)); Serial.print(",");
 	Serial.print(analogRead(PD_R1)); Serial.print(",");
 	Serial.print(analogRead(PD_L1)); Serial.print(",");
 	Serial.print(analogRead(PD_L2)); Serial.println(",");
#endif
	return(sensorInfo | posLine);
}

uint8_t detectedColor, detectedLine;

// speed=2cm/s, mark=3mm -> 150ms

uint16_t tm = 0;
uint8_t dir = 0;

// every 10ms
void timerISR()
{
/*
	if (dir == 0) setMotorSpeed(0.2, 0.2);
	else if (dir == 1) setMotorSpeed(0.3, 0.3);
	else if (dir == 3) setMotorSpeed(-0.2, -0.2);
	else if (dir == 4) setMotorSpeed(-0.3, -0.3);
	else setMotorSpeed(0.0, 0.0);
	tm++;
	if (tm == 200){
		tm = 0;
		dir = (dir + 1) % 6;
	}
*/
}

float vL = 0.0, vR = 0.0;
#define NORMAL_V 0.4
#define MAX_V 1.0
#define MIN_V 0.0

void setup() {
	Serial.begin(115200);
  pixel.begin(); pixel.clear();
	setLED(20, 0, 0);
	setMotorSpeed(0.0, 0.0);
	enableSensorLED(0);
  Wire.begin(); 
	RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  if(!RGBWSensor.begin()) {
    Serial.println("VEML6040 init error");
    while(1){ setLED(20, 0, 0); delay(100); setLED(0, 0, 0); delay(100); }
  }
//  MsTimer2::set(10, timerISR); // every 10ms
//  MsTimer2::set(100, timerISR);
//  MsTimer2::start();
	vL = NORMAL_V; vR = NORMAL_V;
}

void loop() {
	uint8_t sensorInfo = readSensor();
	detectedColor = sensorInfo & 0x07;
	detectedLine = sensorInfo >> 3;

	int lineSum = 0;
	uint8_t nLine = 0;
	float lineValue;
	if (detectedLine & B10000){ lineSum -= 2; nLine++;} // L2
	if (detectedLine &  B1000){ lineSum -= 1; nLine++;} // L1
	if (detectedLine &   B100){ nLine++;}
	if (detectedLine &    B10){ lineSum += 1; nLine++;} // R1
	if (detectedLine &     B1){ lineSum += 2; nLine++;} // R2
	if (nLine == 0) lineValue = -1.0; // no line detected
	else lineValue = (float)lineSum / nLine;

	if (lineValue == -1.0){
		// seek for line
	}
	else{
		if (lineValue > 0.0){
			// turn right
			vL = NORMAL_V + 1.5 * lineValue;
			vR = NORMAL_V - 1.5 * lineValue;
		}
		else if (lineValue < 0.0){
			// turn left
			vL = NORMAL_V + 1.5 * lineValue;
			vR = NORMAL_V - 1.5 * lineValue;
		}
		else{
			vL = NORMAL_V;
			vR = NORMAL_V;
		}
	}
	if (vL > MAX_V) vL = MAX_V;
	else if (vL < MIN_V) vL = MIN_V;
	if (vR > MAX_V) vR = MAX_V;
	else if (vR < MIN_V) vR = MIN_V;
	setMotorSpeed(vL, vR);

	Serial.print(lineValue); Serial.print(' ');
	Serial.print(vL); Serial.print(' '); Serial.print(vR); Serial.print(' '); 
	Serial.print(detectedLine, BIN); Serial.print(' '); 
	Serial.println(detectedColor);

}
