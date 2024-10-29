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

#define TYPE_PD 0
#define TYPE_COLOR 1
#define BLACK_PD_R2 70
#define WHITE_PD_R2 490
#define BLACK_PD_R1 45
#define WHITE_PD_R1 320
#define BLACK_PD_L1 70
#define WHITE_PD_L1 450
#define BLACK_PD_L2 45
#define WHITE_PD_L2 320

#define BLACK_COLOR 3200
#define WHITE_COLOR 9500

typedef struct{
	uint8_t color;
	float line;
} SensorData;

Adafruit_NeoPixel pixel(2, LEDC, NEO_GRB + NEO_KHZ800);

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
		analogWrite(L_A, convMotorPWM(MAX_PWM_L, vL));	analogWrite(L_B, 0);
	}
	else{
		analogWrite(L_A, 0); analogWrite(L_B, -convMotorPWM(MAX_PWM_L, vL));
	}
	if (vR > 0){
		analogWrite(R_A, convMotorPWM(MAX_PWM_R, vR));	analogWrite(R_B, 0);
	}
	else{
		analogWrite(R_A, 0); analogWrite(R_B, -convMotorPWM(MAX_PWM_R, vR));
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
	delay(2);

/*
	uint16_t l2, l1, r1, r2;
	l2 = analogRead(PD_L2);
	l1 = analogRead(PD_L1);
	r1 = analogRead(PD_R1);
	r2 = analogRead(PD_R2);
	Serial.print(l2); Serial.print(' ');
	Serial.print(l1); Serial.print(' ');
	Serial.print(r1); Serial.print(' ');
	Serial.print(r2); Serial.print('/');
	Serial.print(lineValue(l2)); Serial.print(' ');
	Serial.print(lineValue(l1)); Serial.print(' ');
	Serial.print(lineValue(r1)); Serial.print(' ');
	Serial.print(lineValue(r2)); Serial.println("");
*/
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
	enableSensorLED(0);

	// normalize
	float sensorRf, sensorGf, sensorBf;
	sensorRf = (float)sensorR / (float)sensorW * 100.0;
	sensorGf = (float)sensorG / (float)sensorW * 100.0;
	sensorBf = (float)sensorB / (float)sensorW * 100.0;

	if (sensorW > WHITE_COLOR) sensorInfo = COLOR_WHITE;
	else sensorInfo = classify(sensorRf, sensorGf, sensorBf);

	sd.color = sensorInfo;
	if (sensorInfo == COLOR_BLACK) setLED(0, 0, 0);
	else if (sensorInfo == COLOR_RED) setLED(20, 0, 0);
	else if (sensorInfo == COLOR_GREEN) setLED(0, 20, 0);
	else if (sensorInfo == COLOR_BLUE) setLED(0, 0, 20);
	else if (sensorInfo == COLOR_WHITE) setLED(20, 20, 20);
#ifdef DEBUG_COLOR
 	Serial.print(sensorRf); Serial.print(",");
 	Serial.print(sensorGf); Serial.print(",");
 	Serial.print(sensorBf); Serial.print(",");
 	Serial.print(sensorW); Serial.print(",");
 	Serial.print(analogRead(PD_R2)); Serial.print(',');
 	Serial.print(analogRead(PD_R1)); Serial.print(',');
 	Serial.print(analogRead(PD_L1)); Serial.print(',');
 	Serial.print(analogRead(PD_L2)); Serial.print(',');
	Serial.print(lineValue(analogRead(PD_R2), BLACK_PD_R2, WHITE_PD_R2)); Serial.print(',');
	Serial.print(lineValue(analogRead(PD_R1), BLACK_PD_R1, WHITE_PD_R1)); Serial.print(',');
	Serial.print(lineValue(analogRead(PD_L1), BLACK_PD_L1, WHITE_PD_L1)); Serial.print(',');
	Serial.print(lineValue(analogRead(PD_L2), BLACK_PD_L2, WHITE_PD_L2)); Serial.print(',');
	if (sensorInfo != COLOR_WHITE) Serial.print(lineValue(sensorW, BLACK_COLOR, WHITE_COLOR));
	Serial.println(' ');
#endif
	if (sensorInfo != COLOR_WHITE) s += lineValue(sensorW, BLACK_COLOR, WHITE_COLOR);	
	if (s == 0.0) sd.line = -10.0;
	else sd.line = sd.line / s;
	return(sd);
}

uint8_t detectedColor, detectedLine;

// speed=2cm/s, mark=3mm -> 150ms

uint16_t tm = 0;

uint8_t dir = 0;
// every 10ms
void timerISR()
{
/*
	if (dir == 0) setMotorSpeed(0.2, 0);
	else if (dir == 1) setMotorSpeed(-0.2, 0);
	else if (dir == 3) setMotorSpeed(0, 0.2);
	else if (dir == 4) setMotorSpeed(0, -0.2);
	else setMotorSpeed(0.0, 0.0);
	tm++;
	if (tm == 100){
		tm = 0;
		dir = (dir + 1) % 6;
	}
*/
}

float vL = 0.0, vR = 0.0;
#define MAX_V 1.0
#define MIN_V 0.0

#define N_BUF 64
char buf[N_BUF];
uint8_t pBuf = 0;
uint8_t fLog = 0;
uint8_t fMotion = 0;
float Kp = 0.3;
float normalV = 0.4;

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
	vL = normalV; vR = normalV;
}

// ToDo: L&R motor calibration using straight move

float line_previous = 0.0;

void loop() {
	SensorData sd;
	sd = readSensor(sd);
	detectedColor = sd.color;
	float line = sd.line;

	// differential value of lineValue
	float d_line = line - line_previous;
	line_previous = line;
	// P control
	if (line < -5.0){
		// seek for line
			vL = normalV;
			vR = normalV;
	}
	else{
#define KP 0.3
		if (line > 0.0){
			// line at right, turn right
			vL = normalV;
			vR = normalV - KP * line;
		}
		else if (line < 0.0){
			// line at left, turn left
			vL = normalV + KP * line;
			vR = normalV;
		}
		else{
			vL = normalV;
			vR = normalV;
		}
	}
	// D control
//	vL += d_line * 0.1;
//	vR += d_line * 0.1;

	if (vL > MAX_V) vL = MAX_V;
	else if (vL < MIN_V) vL = MIN_V;
	if (vR > MAX_V) vR = MAX_V;
	else if (vR < MIN_V) vR = MIN_V;

	if (fLog == 1){
		Serial.print(line); Serial.print(','); Serial.print(vL); Serial.print(','); Serial.println(vR);
	}

	if (fMotion == 1) setMotorSpeed(vL, vR);
	else setMotorSpeed(0, 0);

//	if (detectedColor != COLOR_WHITE && detectedColor != COLOR_BLACK){
//		Serial.println(detectedColor);
//	}
	while(Serial.available()){
		char c = Serial.read();
		if (c == '\r' || c == '\n'){
			buf[pBuf] = '\0';
			pBuf = 0;
			if (buf[0] == 'L') fLog = 1;
			if (buf[0] == 'l') fLog = 0;
			if (buf[0] == 'M') fMotion = 1;
			if (buf[0] == 'm') fMotion = 0;
			if (buf[0] == 'k'){
				Kp = atof(&buf[1]);
				Serial.println(Kp);
			}
			if (buf[0] == 'v'){
				normalV = atof(&buf[1]);
				Serial.println(normalV);
			}
		}
		else{
			buf[pBuf++] = c; if (pBuf == N_BUF) pBuf = 0;
		}
	}
}
