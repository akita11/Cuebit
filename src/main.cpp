#include <Arduino.h>
#include <MsTimer2.h>
#include "peripheral.h"

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

//uint8_t fLineTrace = 1; // Line trace mode at power on
uint8_t fLineTrace = 0;
uint8_t fDebug = 0;

uint8_t detectedColor;
float vL = 0.0, vR = 0.0;

#define N_BUF 64
char buf[N_BUF];
uint8_t pBuf = 0;

uint8_t nColorCmd = 0;
#define MAX_COLOR_CMD 10
char ColorCmds[MAX_COLOR_CMD];
uint8_t pColorCmd = COLOR_WHITE;
uint8_t pColor = COLOR_WHITE;
uint8_t ColorCmd = COLOR_WHITE;

// every 10ms
void timerISR()
{
}

void setup() {
	Serial.begin(9600);
	init_peripheral();
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
