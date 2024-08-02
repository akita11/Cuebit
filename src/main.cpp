#include <Arduino.h>
#include "Wire.h"
#include "veml6040.h"
#include <Adafruit_NeoPixel.h>

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

#define MOTOR_STEP 4
uint8_t motorL, motorR;
// ref: https://techweb.rohm.co.jp/product/motor/motor-types/499/
uint8_t motor_pattern[] = {0x9, 0x5, 0x6, 0xa};
#define MOTOR_DIR_NONE 0
#define MOTOR_DIR_CW   1
#define MOTOR_DIR_CCW  2

#define PD_THRESHOLD 800   // threshold for photo diode

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

uint8_t readSensor()
{
	// return: L2:L1:C:R1:R2:{COLOR[2:0]} (1=line detected)
	enableSensorLED(1);
	delay(1);
	uint8_t sensorInfo = COLOR_NONE;
	uint16_t sensorR, sensorG, sensorB, sensorW;
	sensorR = RGBWSensor.getRed();
	sensorG = RGBWSensor.getGreen();
	sensorB = RGBWSensor.getBlue();
	sensorW = RGBWSensor.getWhite();
	/*
 	Serial.print(sensorR); Serial.print(",");
 	Serial.print(sensorG); Serial.print(",");
 	Serial.print(sensorB); Serial.print(",");
 	Serial.print(sensorW); Serial.println("");
	*/
	// normalize
	float sensorRf, sensorGf, sensorBf;
	sensorRf = (float)sensorR / (float)sensorW * 100.0;
	sensorGf = (float)sensorG / (float)sensorW * 100.0;
	sensorBf = (float)sensorB / (float)sensorW * 100.0;
	//           R     G     B     W      Rf Gf Bf
	// White     10000 13000 13000 18000  59 71 71
	// BlackLine 4600  5300  4500  10300  47 55 49
	// Black     3600  4000  3300  8500   43 49 40
	// Red       6700  6200  6300  12000  53 43 34
	// Green     4800  6500  4300  11000  42 59 37
	// Blue      4800  6000  7300  11000  39 50 62
	if (sensorW > 15000) sensorInfo = COLOR_WHITE;
	else{
		if (sensorBf > 60) sensorInfo = COLOR_BLUE;
		else if (sensorGf > 58 && sensorRf < 45 && sensorBf < 45) sensorInfo = COLOR_GREEN;
		else if (sensorRf > 50 && sensorGf < 58 && sensorBf < 58) sensorInfo = COLOR_RED;
		else sensorInfo = COLOR_BLACK;
	}
/*	
	Serial.print(sensorInfo, HEX); Serial.print(":");
 	Serial.print(sensorRf); Serial.print(",");
 	Serial.print(sensorGf); Serial.print(",");
 	Serial.print(sensorBf); Serial.print(",");
 	Serial.print(sensorW); Serial.println("");
*/
/*
  	Serial.print(analogRead(PD_R2)); Serial.print(",");
  	Serial.print(analogRead(PD_R1)); Serial.print(",");
  	Serial.print(analogRead(PD_L1)); Serial.print(",");
  	Serial.print(analogRead(PD_L2)); Serial.print(",,");
*/
	if (analogRead(PD_L2) < PD_THRESHOLD) sensorInfo |= 0x80;
	if (analogRead(PD_L1) < PD_THRESHOLD) sensorInfo |= 0x40;
  if ((sensorInfo & 0x07) != COLOR_WHITE) sensorInfo |= 0x20; 
	if (analogRead(PD_R1) < PD_THRESHOLD) sensorInfo |= 0x10;
	if (analogRead(PD_R2) < PD_THRESHOLD) sensorInfo |= 0x08;
	enableSensorLED(0);
	return(sensorInfo);
}

void setLED(uint8_t r, uint8_t g, uint8_t b)
{
	pixel.setPixelColor(0,  pixel.Color(r, g, b)); pixel.show();
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
	RGBWSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  if(!RGBWSensor.begin()) {
    Serial.println("ERROR: couldn't detect the sensor");
    while(1){}
  }
}

void loop() {
	enableMotor(1);
	for (uint16_t i = 0; i < 1024; i++){
		setMotor(MOTOR_DIR_CW, MOTOR_DIR_CW);
		delay(4);
	}
	delay(500);
	for (uint16_t i = 0; i < 1024; i++){
		setMotor(MOTOR_DIR_CCW, MOTOR_DIR_CCW);
		delay(4);
	}
	delay(500);
	for (uint16_t i = 0; i < 1024; i++){
		setMotor(MOTOR_DIR_CW, MOTOR_DIR_CCW);
		delay(4);
	}
	delay(500);
	for (uint16_t i = 0; i < 1024; i++){
		setMotor(MOTOR_DIR_CCW, MOTOR_DIR_CW);
		delay(4);
	}
	delay(500);

	enableMotor(0);
	delay(1000);
/*
	for (uint16_t i = 0; i < 5; i++){
		uint8_t sensorInfo = readSensor();
		uint8_t detectedColor = sensorInfo & 0x07;
		uint8_t detectedLine = sensorInfo >> 3;
		Serial.print(detectedLine, BIN); Serial.print(" "); Serial.println(detectedColor);
		delay(50);
	}
*/
}
