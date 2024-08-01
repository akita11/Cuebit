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

#define PD_THRESHOLD 100   // threshold for photo diode
#define RGBW_THRESHOLD 200 // threshold for RGBW sensor
uint8_t pdPattern; // {0,0,0,R2,R1,C,L1,L2}

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

void readLine()
{
	pdPattern = 0;
	enableLED(1);
	if (analogRead(PD_L2) < PD_THRESHOLD) pdPattern |= 0x01;
	if (analogRead(PD_L1) < PD_THRESHOLD) pdPattern |= 0x02;
  if (RGBWSensor.getWhite() < RGBW_THRESHOLD) pdPattern |= 0x04; 
	if (analogRead(PD_R1) < PD_THRESHOLD) pdPattern |= 0x04;
	if (analogRead(PD_R2) < PD_THRESHOLD) pdPattern |= 0x08;
	enableLED(0);
}

void setLED(uint8_t r, uint8_t g, uint8_t b)
{
	pixel.setPixelColor(0,  pixel.Color(r, g, b)); pixel.show();
}

void setup() {
	RGBWSensor.setConfiguration(VEML6040_IT_320MS + VEML6040_AF_AUTO + VEML6040_SD_ENABLE);
  pixel.begin(); pixel.clear();
	Serial.begin(115200);
	pinMode(L_A1, OUTPUT); pinMode(L_A1, OUTPUT);
	pinMode(L_B1, OUTPUT); pinMode(L_B1, OUTPUT);
	pinMode(R_A1, OUTPUT); pinMode(R_A1, OUTPUT);
	pinMode(R_B1, OUTPUT); pinMode(R_B1, OUTPUT);
	pinMode(MOTOR_nSLEEP, OUTPUT); pinMode(LED_EN, OUTPUT);
	enableMotor(0); enableLED(0);
	motorL = 0; motorR = 0;
  Wire.begin(); 
  if(!RGBWSensor.begin()) {
    Serial.println("ERROR: couldn't detect the sensor");
    while(1){}
  }
}

void loop() {

	enableMotor(1);
	for (uint16_t i = 0; i < 1024; i++){
		setMotor(MOTOR_DIR_CW, MOTOR_DIR_CCW);
		delay(3);
	}
	enableMotor(0);
	for (uint16_t i = 0; i < 5; i++){
		enableSensorLED(1);
		delay(1);
  	Serial.print(analogRead(PD_R2)); Serial.print(",");
  	Serial.print(analogRead(PD_R1)); Serial.print(",");
  	Serial.print(analogRead(PD_L1)); Serial.print(",");
  	Serial.print(analogRead(PD_L2)); Serial.print(",,");
  	Serial.print(RGBWSensor.getRed()); Serial.print(",");
  	Serial.print(RGBWSensor.getGreen()); Serial.print(",");
  	Serial.print(RGBWSensor.getBlue()); Serial.print(",");
  	Serial.print(RGBWSensor.getWhite()); Serial.println("");
		enableSensorLED(0);
		delay(100);
	}
}
