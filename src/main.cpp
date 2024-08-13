#include <Arduino.h>
#include "Wire.h"
#include "veml6040.h"
#include <Adafruit_NeoPixel.h>
#include <MsTimer2.h>

// https://github.com/thewknd/VEML6040/
VEML6040 RGBWSensor;

#define L_A1 2 // PD2
#define L_A2 3 // PD3
#define L_B1 4 // PD4
#define L_B2 5 // PD5
#define R_A1 6 // PD6
#define R_A2 7 // PD7
#define R_B1 8 // PB0
#define R_B2 9 // PB1
#define MOTOR_nSLEEP 10
#define LEDC 11 // for WS2812
#define LED_EN 13
#define PD_R2 A0
#define PD_R1 A3
#define PD_L1 A1
#define PD_L2 A2

//#define DEBUG_COLOR

#define PD_THRESHOLD 600   // threshold for photo diode

#define MOTOR_STEP 4
uint8_t motorL, motorR;
// ref: https://techweb.rohm.co.jp/product/motor/motor-types/499/
uint8_t motor_pattern[] = {0x9, 0x5, 0x6, 0xa};
#define MOTOR_DIR_NONE 0
#define MOTOR_DIR_CW   1
#define MOTOR_DIR_CCW  2

Adafruit_NeoPixel pixel(1, LEDC, NEO_GRB + NEO_KHZ800);

void enableMotor(uint8_t f)
{
	if (f == 0) digitalWrite(MOTOR_nSLEEP, LOW);
	else digitalWrite(MOTOR_nSLEEP, HIGH);
}
void enableSensorLED(uint8_t f)
{
	if (f == 1) digitalWrite(LED_EN, HIGH);
	else digitalWrite(LED_EN, LOW);
}

uint8_t changeMotorPattern(uint8_t m, uint8_t dir)
{
	if (dir == MOTOR_DIR_CW) m = (m + 1) % MOTOR_STEP;
	else if (dir == MOTOR_DIR_CCW) m = (m + MOTOR_STEP - 1) % MOTOR_STEP;
	return(m);
}

void setMotor(uint8_t dirL, uint8_t dirR)
{
	// swap direction for right motor	
	if (dirR == MOTOR_DIR_CW) dirR = MOTOR_DIR_CCW;
	else if (dirR == MOTOR_DIR_CCW) dirR = MOTOR_DIR_CW;

	motorL = changeMotorPattern(motorL, dirL);
	motorR = changeMotorPattern(motorR, dirR);

	uint8_t ML, MR;
	ML = motor_pattern[motorL];	
	MR = motor_pattern[motorR];	
	uint8_t pb, pd;
	pb = PORTB & 0xfc; pd = PORTD & 0x03;
	PORTB = pb | (MR >> 2);
	PORTD = pd | ((ML << 2) | (MR << 6));
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

uint8_t readSensor()
{
	// return: L2:L1:C:R1:R2:{COLOR[2:0]} (1=line detected)
	uint8_t sensorInfo = COLOR_NONE;
	uint16_t sensorR, sensorG, sensorB, sensorW;

	enableSensorLED(1);
	delay(2);

	if (analogRead(PD_L2) < PD_THRESHOLD) sensorInfo |= 0x80;
	if (analogRead(PD_L1) < PD_THRESHOLD) sensorInfo |= 0x40;
  if ((sensorInfo & 0x07) != COLOR_WHITE) sensorInfo |= 0x20; 
	if (analogRead(PD_R1) < PD_THRESHOLD) sensorInfo |= 0x10;
	if (analogRead(PD_R2) < PD_THRESHOLD) sensorInfo |= 0x08;

	sensorR = RGBWSensor.getRed();
	sensorG = RGBWSensor.getGreen();
	sensorB = RGBWSensor.getBlue();
	sensorW = RGBWSensor.getWhite();
	// normalize
	float sensorRf, sensorGf, sensorBf;
	sensorRf = (float)sensorR / (float)sensorW * 100.0;
	sensorGf = (float)sensorG / (float)sensorW * 100.0;
	sensorBf = (float)sensorB / (float)sensorW * 100.0;
	//           Rf Gf Bf / W
	// White     59 71 71 / 22000
	// BlackLine 44 51 43
	// Black     43 58 40 / 12000
	// Red       52 43 34
	// Green     43 60 38
	// Blue      49 51 61

	// ToDo: fine tuning for color detect
	if (sensorW > 9000) sensorInfo = COLOR_WHITE;
	else{
		if (sensorBf > 55) sensorInfo = COLOR_BLUE;
		else if (sensorGf > 58 && sensorRf < 45 && sensorBf < 50) sensorInfo = COLOR_GREEN;
		else if (sensorRf > 50 && sensorGf < 58 && sensorBf < 58) sensorInfo = COLOR_RED;
		else sensorInfo = COLOR_BLACK;
	}
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
 	Serial.print(analogRead(PD_L2)); Serial.print("::");
#endif
#ifdef DEBUG_COLOR
	Serial.println(sensorInfo, HEX);
#endif
	enableSensorLED(0);
	return(sensorInfo);
}

uint8_t detectedColor, detectedLine;

// speed=2cm/s, mark=3mm -> 150ms

uint16_t tm = 0;
uint8_t dir = 0;

// every 10ms
void timerISR()
{
	if (dir == 0 || dir == 2) enableMotor(0); else enableMotor(0);
	if (dir == 0) setMotor(MOTOR_DIR_CW, MOTOR_DIR_CW);
	else if (dir == 2) setMotor(MOTOR_DIR_CCW, MOTOR_DIR_CCW);
	tm++;
	if (tm == 200){
		tm = 0;
		dir = (dir + 1) % 4;
	}
}

void setup() {
	Serial.begin(115200);
  pixel.begin(); pixel.clear();
	pinMode(L_A1, OUTPUT); pinMode(L_A1, OUTPUT);
	pinMode(L_B1, OUTPUT); pinMode(L_B1, OUTPUT);
	pinMode(R_A1, OUTPUT); pinMode(R_A1, OUTPUT);
	pinMode(R_B1, OUTPUT); pinMode(R_B1, OUTPUT);
	pinMode(MOTOR_nSLEEP, OUTPUT); pinMode(LED_EN, OUTPUT);
	enableMotor(0); enableSensorLED(0);
	motorL = 0; motorR = 0;
  Wire.begin(); 
	RGBWSensor.setConfiguration(VEML6040_IT_40MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  if(!RGBWSensor.begin()) {
    Serial.println("ERROR: couldn't detect the sensor");
    while(1);
  }
  MsTimer2::set(10, timerISR);
  MsTimer2::start();
}

void loop() {
	uint8_t sensorInfo = readSensor();
	detectedColor = sensorInfo & 0x07;
	detectedLine = sensorInfo >> 3;
	Serial.print(detectedLine, BIN); Serial.print(" "); Serial.println(detectedColor);
	delay(10);
}
