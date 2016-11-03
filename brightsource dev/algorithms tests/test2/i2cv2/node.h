
	/* this include all node data, configurations and prototypes */
#ifndef node
#define node

	/******************* include part *********************/
#include "math.h"
//#include "servo.h"
#include "headers/basics.h"
#include "headers/i2cdriver.h"
#include "headers/ADC1.h"

/********************* decleration part *****************/
#define VERTICAL_MOTOR_INTIAL 					120 
#define HORIZONTAL_MOTOR_INTIAL 				150
#define MOTOR_ROTATION_STEPS 						2
#define NEGOTIATION_TIMEOUT 						10000
#define SCANING_ANGLES  								8//steps in scanning 
#define MASTER_WAIT_TIME 								20

#define SUN_HEIGHT 											2
#define COORDINATION_UNIT    10//cm
#define HORIZONTAL_MOTOR_HEIGHT											0
#define HORIZONTAL_SHIFT 								-90
#define VERTICAL_SHIFT 									-90

#define RIGTH_DIRECTION 								-1 
#define LEFT_DIRECTION 									 1

#define UP_DIRECTION 								-1 
#define LEFT_DIRECTION 									 1

#define TRUE 														1
#define FALSE													  0

#define NODE_ID													0x3b
#define MAX_VIN													3.3

#define NODE_X_POSITION 									1
#define NODE_Y_POSITION 									1
#define DELTA_POWER					10

#define PI 3.14159265
#define degreesToRadians(angleDegrees) (angleDegrees * PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / PI)

/*
#ifndef I2CMODULE 
	#error "i2c module 0 is enabled by default "
#endif
*/


typedef enum {STARTUP,NEGOTIATION,
SCANNING,NORMAL,RECEIVING,SENDING,
WAITING,TRACKING,SLEEPING}states;



/*holding node current state*/
extern states nodeState;
/*holding horizontal angle*/
extern int8_t hAngle;
/*vertical angle*/
extern int8_t vAngle;
/*current power value*/
extern uint8_t currentPower;
/*for looping in scanning*/
extern uint8_t hIndex,vIndex;
/*master flag*/
extern uint8_t isMaster;
/*right scan flag*/
extern uint8_t isRightFinished;
/*left scan flag*/
extern uint8_t isLeftFinished;
/*max power horizontal position*/
extern uint8_t maxHPosition;
/*max power vertical position*/
extern uint8_t maxVPosition;
/*max power*/
extern uint8_t maxPower;
/*recieving flag*/
extern uint8_t isReceived;

extern uint8_t time;
/*horizontal steps to scan*/
extern uint8_t hSteps;
/*vertical steps to scan*/
extern uint8_t vSteps;
extern uint8_t hDirection;
extern uint8_t vDirection;



//sun position 
extern uint8_t sunPositions[2];/////////

/*node absolute x pos*/
extern uint8_t nodeXPosition;
/*node absolute y pos*/
extern uint8_t nodeYPosition;
/* receved power*/
extern uint8_t receivedPower;
/* data sent flag*/
extern uint8_t isDataSent;
/**/
extern uint8_t originalH;
extern uint8_t originalV;



/********************************* functions prototypes part************************/
void initSystem(void);

//void delay(int ms); // aleady decleared in delay library

uint8_t readLDR(void);

void setState(states state);

void moveStepVertically(uint8_t step,int8_t direction);
void moveStepHorizontally(uint8_t step,int8_t direction);

void moveToVertically(int16_t angle);
void moveToHorizontally(int16_t angle);

void sendData(uint8_t p);
void sendPosition(uint8_t x,uint8_t y);

float getAngle(float height ,float angle,uint8_t shift); // done 
float getLength(float height,float angle,uint8_t shift);

#endif
