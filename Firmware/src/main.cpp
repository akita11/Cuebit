#include <Arduino.h>
#include <MsTimer2.h>
#include "peripheral.h"
#include <EEPROM.h>

#define EEPROM_KP 0 // Kp*80
#define EEPROM_KD 1 // Kd*20
#define EEPROM_VN 2 // Vn*100
#define EEPROM_LR 3 // LRratio*100
#define EEPROM_TF 4 // tm1cm
#define EEPROM_TT 5 // tm10deg

// Motion Control Parameters
float Kp; // P gain for Line trace
float Kd; // D gain for Line trace
uint8_t tm1cm = 115; // [ms]
uint8_t tm10deg = 8; // [ms]
float vNormal, vVerySlow, vSlow, vFast, vVeryFast;

#define vNORMAL 0.3 // for turn and step go

float LRratio = 1.4;
float line_previous = 0;

//#define DELAY_AFTER_CROSS 50 // [x10ms]
#define DELAY_AFTER_CROSS 30 // [x10ms]

float vL = 0.0, vR = 0.0; // left/right motor speed (0.0 - 1.0)
uint8_t dirTrace = 0;

// motion types in micro:bit cmd mode
#define MOTION_NONE				0 
#define MOTION_FWD				1
#define MOTION_BWD				2
#define MOTION_TURN_LEFT	3
#define MOTION_TURN_RIGHT	4
#define MOTION_ZIGZAG			5
#define MOTION_SKATE			6

// color mark command (3=R / 4=G / 5=B)
#define COLOR_CMD_VERY_SLOW        "435" // GRB
#define COLOR_CMD_SLOW             "345" // RGB
#define COLOR_CMD_NORMAL           "343" // RGR
#define COLOR_CMD_FAST             "543" // BGR
#define COLOR_CMD_VERY_FAST        "534" // BRG
#define COLOR_CMD_PAUSE            "535" // BRB
#define COLOR_CMD_LEFT_AT_CROSS    "453" // GBR
#define COLOR_CMD_FORWARD_AT_CROSS "545" // BGB 
#define COLOR_CMD_RIGHT_AT_CROSS   "454" // GBG
#define COLOR_CMD_UTURN            "353" // RBR
#define COLOR_CMD_GO_BACK          "434" // GRG
// pending commands
//#define COLOR_CMD_JUMP_LEFT        ""
//#define COLOR_CMD_JUMP_FORWARD     ""
//#define COLOR_CMD_JUMP_RIGHT       ""
//#define COLOR_CMD_JUMP_HURRICANE   ""
//#define COLOR_CMD_ZIGZAG           ""
//#define COLOR_CMD_SPIN             ""

// flag for operation
uint8_t fMotion = MOTION_NONE; // currently operating motion
uint8_t fLineTrace = 1; // Line trace mode at power on
uint8_t fDebug = 0; // debug output of sensor data

uint8_t detectedColor;
int tm10ms = 0;
uint8_t fCross = 0; // "Cross point" detected

#define N_BUF 64 // Serial RX buffer size
char buf[N_BUF];
uint8_t pBuf = 0;

// Variables for color command in line trace mode 
uint8_t nColorCmd = 0;
#define MAX_COLOR_CMD 10
char ColorCmds[MAX_COLOR_CMD];
uint8_t pColorCmd = COLOR_WHITE;
uint8_t pColor = COLOR_WHITE;
uint8_t ColorCmd = COLOR_WHITE;
uint8_t nColorContinuous = 0;

uint8_t stateColorCmd = 0;
#define COLOR_CMD_ST_PAUSE             1
#define COLOR_CMD_ST_CROSS_TO_LEFT     4
#define COLOR_CMD_ST_CROSS_LEFT        3
#define COLOR_CMD_ST_CROSS_FORWARD     4
#define COLOR_CMD_ST_CROSS_TO_RIGHT    5
#define COLOR_CMD_ST_CROSS_RIGHT       6
#define COLOR_CMD_ST_UTURN             7
#define COLOR_CMD_ST_GOING_AFTER_CROSS 8
uint8_t cmdTurnAtCross = 0;

double leftDevi = 1.0, rightDevi = 1.0;
uint8_t tSkate = 0;
#define SKATE_CYCLE 100 // [x 10ms]

uint8_t stateZigzag = 0;
#define ZIGZAG_STEP 100 // [x 10ms] of half cycle
#define ZIGZAG_TURN 50 // [x 10ms] of turn time

uint8_t fLEDmsg = 0;
uint8_t stLEDmsg = 0;
uint8_t cntLEDmsg = 0;
uint8_t cntNoLine = 0;
uint8_t fNoLine = 0;
#define TIME_NOLINE 50 // [x10ms]

// every 10ms
void timerISR()
{
	if (tm10ms > 0) tm10ms--;
	if (fMotion == MOTION_SKATE){
		leftDevi = 1.0 + 0.3 * sin((double)tSkate / (double)SKATE_CYCLE * 2 * 3.14);
		rightDevi = 1.0 - 0.3 * sin((double)tSkate / (double)SKATE_CYCLE * 2 * 3.14);
		tSkate = (tSkate + 1) % SKATE_CYCLE;
	}
	if (fMotion == MOTION_ZIGZAG){
		stateZigzag = (stateZigzag + 1) % (ZIGZAG_STEP * 2);
	}
	if (stLEDmsg != LEDMSG_NONE){
		cntLEDmsg++;
		if (cntLEDmsg > 10){
			cntLEDmsg = 0;
			fLEDmsg = (fLEDmsg + 1) % 10;
			if (stLEDmsg == LEDMSG_LOW_BATTERY){
				if ((fLEDmsg % 2) == 1) setLED(20, 0, 0); // red
				else setLED(0, 0, 0);
			}
			else if (stLEDmsg == LEDMSG_LOST_LINE){
				if ((fLEDmsg % 2) == 1) setLED(20, 20, 0); // yellow
				else setLED(0, 0, 0);
			}
			else if (stLEDmsg == LEDMSG_MICROBIT){
				if (fLEDmsg > 5) setLED(10, 0, 10); // purple
				else setLED(0, 0, 0);
			}
		}
	}
	if (fNoLine == 1){
		if (cntNoLine < TIME_NOLINE) cntNoLine++;
	}
	else cntNoLine = 0;
}

void setSpeedParams()
{
	vVerySlow = vNormal * 0.5;
	vSlow = vNormal * 0.6;
	vFast = vNormal * 1.5;
	vVeryFast = vNormal * 2.0;
}

void setup() {
	Serial.begin(9600);
//	analogWrite(11, 128); while(1); // for OSC calib, 244.14Hz on MOSI

	if (EEPROM.read(EEPROM_KP) < 255) Kp = (float)EEPROM.read(EEPROM_KP) / 80.0;
	else Kp = 0.7;
	if (EEPROM.read(EEPROM_KD) < 255) Kd = (float)EEPROM.read(EEPROM_KD) / 20.0;
	else Kd = 5.0;
	if (EEPROM.read(EEPROM_VN) < 100) vNormal = (float)EEPROM.read(EEPROM_VN) / 100.0;
	else vNormal = 0.3;
	if (EEPROM.read(EEPROM_LR) < 255) LRratio = (float)EEPROM.read(EEPROM_LR) / 100.0;
	else LRratio = 1.0;
	if (EEPROM.read(EEPROM_TF) < 255) tm1cm = EEPROM.read(EEPROM_TF);
	else tm1cm = 90;
	if (EEPROM.read(EEPROM_TT) < 255) tm10deg = EEPROM.read(EEPROM_TT);
	else tm10deg = 5;

	setSpeedParams();

	Serial.print("Kp="); Serial.print(Kp); Serial.print(" Kd="); Serial.print(Kd); Serial.print(" Vn="); Serial.print(vNormal); Serial.print(" LR="); Serial.println(LRratio);
	init_peripheral();
	MsTimer2::set(10, timerISR); // every 10ms
	MsTimer2::start();
}

// ToDo: L&R motor calibration using straight move

int16_t getParam(char *s){
	if ((s[0] >= '0' && s[0] <= '9') || s[0] == '-' ) return(atoi(s));
	else return(0);
}

#define LOW_BATTERY_TH_HL 3.5 // [V], with LDO of Vdrop=0.2V
#define LOW_BATTERY_TH_LH 4.0 // [V], with LDO of Vdrop=0.2V
float sumBatteryVoltage = 0.0;
uint8_t nSumBatteryVolgate = 0;
#define N_SUM_BATTERY_VOLTAGE 50
float BatteryVoltage = 0.0;

float getBatteryVoltage()
{
	float v;
	v = analogRead(A6)  * (3.3 / 0.6) / 1023.0; // Vbat = VDD * 0.6
	return(v);	
}

void loop() {
	sumBatteryVoltage += getBatteryVoltage();
	nSumBatteryVolgate++;
	if (nSumBatteryVolgate == N_SUM_BATTERY_VOLTAGE){
		BatteryVoltage = sumBatteryVoltage / N_SUM_BATTERY_VOLTAGE;
		sumBatteryVoltage = 0;
		nSumBatteryVolgate = 0;
		if (stLEDmsg != LEDMSG_LOW_BATTERY){
			if (BatteryVoltage <= LOW_BATTERY_TH_HL) stLEDmsg = LEDMSG_LOW_BATTERY;
		}
		else if (stLEDmsg == LEDMSG_LOW_BATTERY){
			if (BatteryVoltage >= LOW_BATTERY_TH_LH) stLEDmsg = LEDMSG_NONE;
		}
	}
	
	if (fLineTrace == 1){
		if (stateColorCmd == COLOR_CMD_ST_PAUSE){
			if (tm10ms > 0) setMotorSpeed(0, 0);
			else stateColorCmd = 0;
		}
		else if (stateColorCmd == COLOR_CMD_ST_UTURN){
			if (tm10ms > 0) setMotorSpeed(-vNORMAL, vNORMAL);
			else stateColorCmd = 0;
		}
		else if (cmdTurnAtCross == COLOR_CMD_ST_CROSS_LEFT || cmdTurnAtCross == COLOR_CMD_ST_CROSS_RIGHT){
			// tm10ms
			// - DELAY_AFTER_CROSS+tm10deg*9 - tm10deg*9 : go forward after cross
			// - tm10deg*9 - 0                           : turn left/right
//			Serial.print(cmdTurnAtCross); Serial.print(':'); Serial.println(tm10ms);
			if (tm10ms > tm10deg * 9) setMotorSpeed(vNORMAL, vNORMAL); // go forward after cross
			else if (tm10ms > 0){
				if (cmdTurnAtCross == COLOR_CMD_ST_CROSS_LEFT) fMotion = MOTION_TURN_LEFT;
				else fMotion = MOTION_TURN_RIGHT;
			}
		}

		if (fMotion == MOTION_TURN_RIGHT){
			fNoLine = 0; cntNoLine = 0;
			if (tm10ms > 0) setMotorSpeed(vNORMAL, -vNORMAL);
			else{
				stateColorCmd = 0;
				fMotion = MOTION_NONE;
			}
		}
		else if (fMotion == MOTION_TURN_LEFT){
			fNoLine = 0; cntNoLine = 0;
			if (tm10ms > 0) setMotorSpeed(-vNORMAL, vNORMAL);
			else{
				stateColorCmd = 0;
				fMotion = MOTION_NONE;
			}
		}
		else{
			// line trace mode
			SensorData sd;
			sd = readSensor(sd);
			detectedColor = sd.color;

			// differential value of lineValue
			float d_line = sd.line - line_previous;
			line_previous = sd.line;

			// color command detection	
#define COLOR_MARK_TH 10
			if (sd.color == pColor) nColorContinuous++;
			else{
//			if (nColorContinuous > COLOR_MARK_TH){
//				Serial.print(pColor); Serial.print(':'); Serial.println(nColorContinuous);
//			}
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
						Serial.print(nColorCmd); Serial.print('*'); Serial.println(ColorCmds);
						if (nColorCmd >= 3){
							// execute motion
							if (strncmp(ColorCmds, COLOR_CMD_VERY_SLOW, 3) == 0){
							Serial.println("CMD:veryslow");
							vNormal = vVerySlow;
						}
							if (strncmp(ColorCmds, COLOR_CMD_SLOW, 3) == 0){
							Serial.println("CMD:slow");
							vNormal = vSlow;
						}
							if (strncmp(ColorCmds, COLOR_CMD_NORMAL, 3) == 0){
							Serial.println("CMD:normal");
							vNormal = vNORMAL;
						}
							if (strncmp(ColorCmds, COLOR_CMD_FAST, 3) == 0){
							Serial.println("CMD:fast");
							vNormal = vFast;
						}
							if (strncmp(ColorCmds, COLOR_CMD_VERY_FAST, 3) == 0){
							Serial.println("CMD:veryfast");
							vNormal = vVeryFast;
						}
							if (strncmp(ColorCmds, COLOR_CMD_PAUSE, 3) == 0){
								Serial.println("CMD:pause");
								tm10ms = 300; // pause time = 300ms
								stateColorCmd = COLOR_CMD_ST_PAUSE;
							}
							if (strncmp(ColorCmds, COLOR_CMD_LEFT_AT_CROSS, 3) == 0){
								Serial.println("CMD:left at cross");
								tm10ms = 0;
								cmdTurnAtCross = COLOR_CMD_ST_CROSS_TO_LEFT;
							}
							if (strncmp(ColorCmds, COLOR_CMD_FORWARD_AT_CROSS, 3) == 0){
								Serial.println("CMD:forward at cross");
								tm10ms = 0;
								cmdTurnAtCross = COLOR_CMD_ST_CROSS_FORWARD;
							}
							if (strncmp(ColorCmds, COLOR_CMD_RIGHT_AT_CROSS, 3) == 0){
								Serial.println("CMD:right at cross");
								tm10ms = 0;
								cmdTurnAtCross = COLOR_CMD_ST_CROSS_TO_RIGHT;
							}
							if (strncmp(ColorCmds, COLOR_CMD_UTURN, 3) == 0){
								Serial.println("CMD:u-turn");
								stateColorCmd = COLOR_CMD_ST_UTURN;
								tm10ms = tm10deg * 18; // turn 180deg
							}
							if (strncmp(ColorCmds, COLOR_CMD_GO_BACK, 3) == 0){
								Serial.println("CMD:go back");
								dirTrace = 1 - dirTrace;
							}
						}
						nColorCmd = 0;
					}
					pColorCmd = ColorCmd;
				}
			}

			// P control
			if (sd.line < -5.0 || sd.color == COLOR_WHITE){
				// start count timer at marker/line lost
				fNoLine = 1;
				if (cntNoLine == TIME_NOLINE){
					// line lost for a while, stop
					vL = 0.0; vR = 0.0;
					if (stLEDmsg != LEDMSG_LOW_BATTERY) stLEDmsg = LEDMSG_LOST_LINE;
				}
			}
			else{
				fNoLine = 0;
				if (stLEDmsg != LEDMSG_LOW_BATTERY) stLEDmsg = LEDMSG_NONE;
				if (sd.line > 0.0){
					// line at right, turn right
					vL = vNormal;
					vR = vNormal - Kp * sd.line;
				}
				else if (sd.line < 0.0){
					// line at left, turn left
					vL = vNormal + Kp * sd.line;
					vR = vNormal;
				}
				else{
					vL = vNormal;
					vR = vNormal;
				}
				// D control
				vL += d_line * Kd;
				vR += d_line * Kd;
				if (vL > MAX_V) vL = MAX_V;
				else if (vL < MIN_V) vL = MIN_V;
				if (vR > MAX_V) vR = MAX_V;
				else if (vR < MIN_V) vR = MIN_V;
			}
			if (dirTrace == 0) setMotorSpeed(vL, vR);
			else setMotorSpeed(-vR, -vL);
			// cross detection
			if (stateColorCmd != COLOR_CMD_ST_UTURN){
#define LINE_CROSS_TH 3.0
				if (sd.width > LINE_CROSS_TH){
					if (fCross == 0){
						fCross = 1;
						Serial.print("cross "); Serial.println(cmdTurnAtCross);
						if (cmdTurnAtCross == COLOR_CMD_ST_CROSS_TO_LEFT){
							// turn left at cross
							tm10ms = DELAY_AFTER_CROSS + tm10deg * 9;
							cmdTurnAtCross = COLOR_CMD_ST_CROSS_LEFT;
							Serial.println("turn left at cross");
						}
						else if (cmdTurnAtCross == COLOR_CMD_ST_CROSS_TO_RIGHT){
							// turn right at cross
							tm10ms = DELAY_AFTER_CROSS + tm10deg * 9;
							cmdTurnAtCross = COLOR_CMD_ST_CROSS_RIGHT;
							Serial.println("turn right at cross");
						}
						else if (cmdTurnAtCross == COLOR_CMD_ST_CROSS_FORWARD){
							Serial.println("go forward at cross");
							cmdTurnAtCross = 0;
						}
					}
				}
				else fCross = 0;
			}
		}
	}
	else{
		// micro:bit command motion
		enableSensorLED(0);
		switch(fMotion){
			case MOTION_NONE : setMotorSpeed(0, 0); break;
			case MOTION_TURN_RIGHT:
				if (tm10ms > 0) setMotorSpeed(vNORMAL, -vNORMAL);
				else{ setMotorSpeed(0, 0); fMotion = MOTION_NONE;}
				break;
			case MOTION_TURN_LEFT:
				if (tm10ms > 0) setMotorSpeed(-vNORMAL, vNORMAL);
				else{ setMotorSpeed(0, 0); fMotion = MOTION_NONE;}
				break;
			case MOTION_FWD:
				if (tm10ms > 0) setMotorSpeed(vNORMAL, vNORMAL);
				else{ setMotorSpeed(0, 0); fMotion = MOTION_NONE;}
				break;
			case MOTION_BWD:
				if (tm10ms > 0) setMotorSpeed(-vNORMAL, -vNORMAL);
				else{ setMotorSpeed(0, 0); fMotion = MOTION_NONE;}
				break;
			case MOTION_ZIGZAG:
				if (tm10ms > 0){
					if (stateZigzag >= 0 && stateZigzag < ZIGZAG_TURN) setMotorSpeed(vNORMAL, -vNORMAL); // turn right
					else if (stateZigzag >= ZIGZAG_STEP && stateZigzag < ZIGZAG_STEP + ZIGZAG_TURN) setMotorSpeed(-vNORMAL, vNORMAL); // turn right
					else setMotorSpeed(vNORMAL, vNORMAL); // go straight
				}
				else {setMotorSpeed(0, 0); fMotion = MOTION_NONE;}
				break;
			case MOTION_SKATE:
//				Serial.print(tSkate); Serial.print(' '); Serial.print(vNORMAL*leftDevi); Serial.print(' '); Serial.println(vNORMAL*rightDevi);
				if (tm10ms > 0) setMotorSpeed(vNORMAL * leftDevi, vNORMAL * rightDevi);
				else {setMotorSpeed(0, 0); fMotion = MOTION_NONE;}
				break;
		}
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
					Serial.print(" V(v)="); Serial.print(vNormal); 
					Serial.print(" LRr(r)="); Serial.print(LRratio);
					Serial.print(" tm1cm(f)="); Serial.print(tm1cm); 
					Serial.print(" tm10deg(g)="); Serial.println(tm10deg);
				}
				if (buf[0] == 'T') {fLineTrace = 1; pColorCmd = COLOR_WHITE; nColorCmd = 0; }
				if (buf[0] == 't') {fLineTrace = 0; setMotorSpeed(0, 0); }
				if (buf[0] == 'D'){
					 if (buf[1] == 'k') fDebug = 2;
					 else if (buf[1] == 'r') fDebug = 3;
					 else if (buf[1] == 'g') fDebug = 4;
					 else if (buf[1] == 'y') fDebug = 5;
					 else if (buf[1] == 'w') fDebug = 1;
					 else fDebug = 6;
				}
				if (buf[0] == 'd') fDebug = 0;
				if (buf[0] == 'k'){ Kp = atof(&buf[1]); Serial.println(Kp); }
				if (buf[0] == 'K'){ Kd = atof(&buf[1]); Serial.println(Kd); }
				if (buf[0] == 'r'){ LRratio = atof(&buf[1]); Serial.println(LRratio); }
				if (buf[0] == 'v'){ vNormal = atof(&buf[1]); Serial.println(vNormal); setSpeedParams(); }	
				if (buf[0] == 'f'){ tm1cm = atoi(&buf[1]); Serial.println(tm1cm); }
				if (buf[0] == 'g'){ tm10deg = atoi(&buf[1]); Serial.println(tm10deg); }
				if (buf[0] == '!'){
					EEPROM.write(EEPROM_KP, (uint8_t)(Kp * 80));
					EEPROM.write(EEPROM_KD, (uint8_t)(Kd * 20));
					EEPROM.write(EEPROM_VN, (uint8_t)(vNormal * 100));
					EEPROM.write(EEPROM_LR, (uint8_t)(LRratio * 100));
					EEPROM.write(EEPROM_TF, tm1cm);
					EEPROM.write(EEPROM_TT, tm10deg);
					Serial.println("parameters saved.");
				}
				// micro:bit command
				if (buf[0] == '$'){
					Serial.println("enter micro:bit command mode");
					fLineTrace = 0;
					setMotorSpeed(0, 0);
					stLEDmsg = LEDMSG_MICROBIT;
				}
				if (buf[0] == '#'){
					Serial.println("exit micro:bit command mode");
					fLineTrace = 1;
					stLEDmsg = LEDMSG_NONE;
				}
				if (buf[0] == 'R'){
					// Rxx turn right(+) or left(-) xx degree
					fLineTrace = 0;
					param = getParam(buf+1);
					if (param >0){
						fMotion = MOTION_TURN_RIGHT;
						tm10ms = tm10deg * param / 10;
						Serial.println(tm10ms);
					}
					else{
						fMotion = MOTION_TURN_LEFT;
						tm10ms = tm10deg * (-param) / 10;
					}
				}
				if (buf[0] == 'B'){
					// B stop
					fMotion = MOTION_NONE;
				}
				if (buf[0] == 'F'){
					// Fxx go straight xxcm
					param = getParam(buf+1);
					if (param > 0){
						fMotion = MOTION_FWD;
						tm10ms = tm1cm * param / 10;
					}
					else{
						fMotion = MOTION_BWD;
						tm10ms = tm1cm * (-param) / 10;
					}
				}
				if (buf[0] == 'Z'){
					// Zxx zig-zag xxcm
					param = getParam(buf+1);
					fMotion = MOTION_ZIGZAG;
					stateZigzag = 0;
					tm10ms = param; // tentative
				}
				if (buf[0] == 'S'){
					// Sxx skate xxcm
					param = getParam(buf+1);
					fMotion = MOTION_SKATE;
					tSkate = 0;
					tm10ms = param; // tentative
				}
			}
			else{ buf[pBuf++] = c; if (pBuf == N_BUF) pBuf = 0; }
		}
	}
}
