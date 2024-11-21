#define COLOR_NONE	0
#define COLOR_WHITE 1
#define COLOR_BLACK 2
#define COLOR_RED	  3
#define COLOR_GREEN 4
#define COLOR_BLUE  5
#define COLOR_CROSS 6

#define MAX_V 1.0
#define MIN_V 0.0

typedef struct{
	uint8_t color;
	float line;
	float line_previous;
	float width;
} SensorData;

extern void init_peripheral();
extern void enableSensorLED(uint8_t f);
extern void setMotorSpeed(float vL, float vR);
extern void setLED(uint8_t r, uint8_t g, uint8_t b);
extern SensorData readSensor(SensorData sd);
extern uint8_t fDebug;
extern float vL, vR;



