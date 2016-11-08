#include "headers/Basics.h"
#ifndef _SERVO_
#define _SERVO_

#define STEPSIZE  8.62 
void initServoMotor(void);
void servoGotoAngle(int16_t angle);
void servoGotoVerticalAngle(int16_t verticalAngle);
void servoGotoHorizontalAngle(int16_t horizontalAngle);
void servoGotoVH(int16_t verticalAngle,int16_t horizontalAngle);
uint16_t getTime(int16_t angle) ;
#endif
