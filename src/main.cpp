/* micro:bit command
$ enter micro:bit command mode
# exit micro:bit command mode
Rxx turn right(+)/left(-) xx degree
Fxx go forward(+)/backward(-) xx cm
Zxx zig-zag xxcm
Sxx skate xxcm
B stop
*/

/* color mark command
*RGB very slow
RGR slow (<- RKR)
BGB fast (<- BKB)
*BGB turbo
*RBR temporal stop
*GKR turn left at cross
*BKR go straight at cross
*BRG turn right at cross
*BRW U-turn
*GRG jump to left
*GBG jump straight
*RGR jump to right
*RGRG hurricane
*BKGR zig-zag
*GRGR spin
*RGKB go backard
*/

#include <Arduino.h>
#include "Wire.h"
#include "veml6040.h"
#include <Adafruit_NeoPixel.h>
#include <MsTimer2.h>

// https://github.com/thewknd/VEML6040/
VEML6040 RGBWSensor;

float Kp = 1.0;
float Kd = 0.0;
uint8_t tm1cm = 100; // [ms]
uint8_t tm10deg = 70; // [ms]
#define vSlow 0.3
#define vFast 1.0
#define vNORMAL 0.4
#define vVerySlow 0.3
#define vVeryFast 0.8
float normalV = vNORMAL;
uint8_t fLineTrace = 1; // Line trace mode at power on
uint8_t fDebug = 0;

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

#define BLACK_PD_R2 35
#define WHITE_PD_R2 313
#define BLACK_PD_R1 37
#define WHITE_PD_R1 343
#define BLACK_PD_L1 38
#define WHITE_PD_L1 337
#define BLACK_PD_L2 39
#define WHITE_PD_L2 351

#define BLACK_COLOR 3200
#define WHITE_COLOR 10000

typedef struct{
	uint8_t color;
	float line;
	float width;
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
#define COLOR_CROSS 6

float vL = 0.0, vR = 0.0;
#define MAX_V 1.0
#define MIN_V 0.0

#define N_BUF 64
char buf[N_BUF];
uint8_t pBuf = 0;

uint8_t nColorCmd = 0;
#define MAX_COLOR_CMD 10
char ColorCmds[MAX_COLOR_CMD];
uint8_t pColorCmd = COLOR_WHITE;
uint8_t pColor = COLOR_WHITE;
uint8_t ColorCmd = COLOR_WHITE;

void setLED(uint8_t r, uint8_t g, uint8_t b)
{
	pixel.setPixelColor(0,  pixel.Color(r, g, b)); pixel.setPixelColor(1,  pixel.Color(r, g, b)); pixel.show();
}

uint8_t classify(float R, float G, float B) {
	// SVM model on 241105
	float coeff[4][3] = {
  	{ 0.1129,  0.0743, -0.0001 }, // K
  	{ 2.9690, -0.9396,  0.4897 }, // R
  	{-1.3199,  1.7235, -0.9846 }, // G
  	{-1.9934,  0.5089,  1.5674 } // B
	};
	float intercepts[4] = { -7.7428, -93.6174, -3.8712, -5.7054 };

	float decision_values[4];
	for (int i = 0; i < 4; i++) {
		decision_values[i] = coeff[i][0] * R + coeff[i][1] * G + coeff[i][2] * B + intercepts[i];
	}

	int max_index = 0;
	for (int i = 1; i < 4; i++) {
  	if (decision_values[i] > decision_values[max_index]) {
    	max_index = i;
  	}
  }
	if (max_index == 0) return(COLOR_BLACK);
	else if (max_index == 1) return(COLOR_RED);
	else if (max_index == 2) return(COLOR_GREEN);
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
	delay(2);

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

	if (sensorInfo != COLOR_WHITE) s += lineValue(sensorW, BLACK_COLOR, WHITE_COLOR);	
	if (s == 0.0) sd.line = -10.0;
	else sd.line = sd.line / s;
	sd.width = s;
	if (fDebug == 1){
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
		Serial.print(':'); Serial.print(sd.line);
		Serial.print(':'); Serial.println(sd.width);
		Serial.print('/'); Serial.print(sd.color);
		Serial.print('|'); Serial.print(vL); Serial.print(','); Serial.println(vR);
	}
	return(sd);
}

uint8_t detectedColor, detectedLine;

// speed=2cm/s, mark=3mm -> 150ms

// every 10ms
void timerISR()
{
}

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
//  MsTimer2::start();
	vL = normalV; vR = normalV;
}

// ToDo: L&R motor calibration using straight move

float line_previous = 0.0;

int16_t getParam(char *s){
	if ((s[0] >= '0' && s[0] <= '9') || s[0] == '-' ) return(atoi(s));
	else return(0);
}

uint8_t nColorContinuous = 0;
uint8_t colorCmd = COLOR_WHITE;

uint8_t fCross = 0;

void loop() {
	if (fLineTrace == 1){
		SensorData sd;
		sd = readSensor(sd);
		detectedColor = sd.color;

		float line = sd.line;
		// differential value of lineValue
		float d_line = line - line_previous;
		line_previous = line;

		// color command detection	
#define COLOR_MARK_TH 10
		if (sd.color == pColor) nColorContinuous++;
		else{
//		if (nColorContinuous > COLOR_MARK_TH){
//			Serial.print(pColor); Serial.print(':'); Serial.println(nColorContinuous);
//		}
			nColorContinuous = 0;
		}
		pColor = sd.color;
		if (nColorContinuous > COLOR_MARK_TH){
			// color mark detected (spike noise removed)
			ColorCmd = sd.color;
			if (ColorCmd != pColorCmd){
				// color mark changed
				if (ColorCmd != COLOR_WHITE && ColorCmd != COLOR_BLACK){
					ColorCmds[nColorCmd] = '0' + ColorCmd;
					nColorCmd++;
					if (nColorCmd == MAX_COLOR_CMD){
						// color command buffer full, ignore buffer
						nColorCmd = 0;
					}
				}
				if ((ColorCmd == COLOR_BLACK || ColorCmd == COLOR_WHITE) && (pColorCmd != COLOR_BLACK && pColorCmd != COLOR_WHITE)){
					ColorCmds[nColorCmd] = '\0';
					// end of color command
//					Serial.print(nColorCmd); Serial.print('*'); Serial.println(ColorCmds);
					if (nColorCmd >= 3){
						// execute motion
						if (strncmp(ColorCmds, "343", 3) == 0){
							Serial.println("CMD:slow");
							normalV = 0.3;
						}
						if (strncmp(ColorCmds, "545", 3) == 0){
							Serial.println("CMD:fast");
							normalV = 0.6;
						}
					}
					nColorCmd = 0;
				}
				pColorCmd = ColorCmd;
			}
		}

		// P control
		if (line < -5.0){
			// seek for line
				vL = normalV; vR = normalV;
		}
		else{
			if (line > 0.0){
				// line at right, turn right
				vL = normalV;
				vR = normalV - Kp * line;
			}
			else if (line < 0.0){
				// line at left, turn left
				vL = normalV + Kp * line;
				vR = normalV;
			}
			else{
				vL = normalV;
				vR = normalV;
			}
		}
		// D control
		vL += d_line * Kd;
		vR += d_line * Kd;
		if (vL > MAX_V) vL = MAX_V;
		else if (vL < MIN_V) vL = MIN_V;
		if (vR > MAX_V) vR = MAX_V;
		else if (vR < MIN_V) vR = MIN_V;
		setMotorSpeed(vL, vR);
/*
		// cross detection
#define LINE_CROSS_TH 3.0
		if (sd.width > LINE_CROSS_TH){
			if (fCross == 0){
				fCross = 1;
				Serial.println("cross");
				fLineTrace = 0;
				setMotorSpeed(0, 0);
			}
		}
		else fCross = 0;
*/
	}
	int16_t param;
	while(Serial.available()){
		char c = Serial.read();
		if (c > 0){ // ignore non-ASCII characters
		if (c == '\r' || c == '\n'){
			buf[pBuf] = '\0';
			pBuf = 0;
			if (buf[0] == 'P'){
				Serial.print("Kp(k)="); Serial.print(Kp);
				Serial.print(" Kd(K)="); Serial.print(Kd); 
				Serial.print(" V(v)="); Serial.print(normalV); 
				Serial.print(" tm1cm(f)="); Serial.print(tm1cm); 
				Serial.print(" tm10deg(g)="); Serial.println(tm10deg);
			}
			if (buf[0] == 'T') {fLineTrace = 1; pColorCmd = COLOR_WHITE; nColorCmd = 0; }
			if (buf[0] == 't') {fLineTrace = 0; setMotorSpeed(0, 0); }
			if (buf[0] == 'D') fDebug = 1;
			if (buf[0] == 'd') fDebug = 0;
			if (buf[0] == 'k'){ Kp = atof(&buf[1]); Serial.println(Kp); }
			if (buf[0] == 'K'){ Kd = atof(&buf[1]); Serial.println(Kd); }
			if (buf[0] == 'v'){ normalV = atof(&buf[1]); Serial.println(normalV); }
			if (buf[0] == 'f'){ tm1cm = atoi(&buf[1]); Serial.println(tm1cm); }
			if (buf[0] == 'g'){ tm10deg = atoi(&buf[1]); Serial.println(tm10deg); }
			// micro:bit command
			if (buf[0] == '$'){
				Serial.println("enter micro:bit command mode");
				fLineTrace = 0;
				setMotorSpeed(0, 0);
				setLED(20, 0, 20); // purple
			}
			if (buf[0] == '#'){
				Serial.println("exit micro:bit command mode");
				fLineTrace = 1;
				setLED(0, 0, 0); // purple
			}
			if (buf[0] == 'R'){
				// Rxx turn right(+) or left(-) xx degree
				fLineTrace = 0;
				param = getParam(buf+1);
				if (param >0){
					setMotorSpeed(vNORMAL, -vNORMAL);
					delay(tm10deg * param / 10);
				}
				else{
					setMotorSpeed(-vNORMAL, vNORMAL);
					delay(tm10deg * -(param) / 10);
				}
				setMotorSpeed(0, 0);
			}
			if (buf[0] == 'B'){
				// B stop
				// ToDo: interrupt other command
				fLineTrace = 0;
				setMotorSpeed(0, 0);
			}
			if (buf[0] == 'F'){
				// Fxx go straight xxcm
				fLineTrace = 0;
				param = getParam(buf+1);
				if (param > 0){
					setMotorSpeed(vNORMAL, vNORMAL);
					delay(tm1cm * param);
				}
				else{
					setMotorSpeed(-vNORMAL, -vNORMAL);
					delay(tm1cm * -(param));
				}
				setMotorSpeed(0, 0);
			}
			if (buf[0] == 'Z'){
				// Zxx zig-zag xxcm
				fLineTrace = 0;
				param = getParam(buf+1);
				// ToDo: implement zig-zag motion
			}
			if (buf[0] == 'S'){
				// Sxx skate xxcm
				fLineTrace = 0;
				param = getParam(buf+1);
				// ToDo: implement skate motion
			}
		}
		else{ buf[pBuf++] = c; if (pBuf == N_BUF) pBuf = 0; }
		}
	}
}
