#include "Basics.h"
#define STEPSIZE  8.62 
void initServoMotor(void);
void servoGotoAngle(uint8_t angle);
void servoGotoVerticalAngle(uint8_t verticalAngle);
void servoGotoHorizontalAngle(uint8_t horizontalAngle);
void servoGotoVH(uint8_t verticalAngle,uint8_t horizontalAngle);
uint16_t getTime(uint8_t angle) ;
