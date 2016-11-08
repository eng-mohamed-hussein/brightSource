#include "node.h"
#include "timeouttimer.h"

/*holding node current state*/
states nodeState;
/*holding horizontal angle*/
int16_t hAngle;
/*vertical angle*/
int16_t vAngle;
/*current power value*/
uint8_t currentPower;
/*for looping in scanning*/
uint8_t hIndex,vIndex;
/*master flag*/
uint8_t isMaster;
/*right scan flag*/
uint8_t isRightFinished;
/*left scan flag*/
uint8_t isLeftFinished;
/*max power horizontal position*/
uint16_t maxHPosition;
/*max power vertical position*/
uint16_t maxVPosition;
/*max power*/
uint8_t maxPower;
/*recieving flag*/
uint8_t isReceived;

uint8_t time;
/*horizontal steps to scan*/
uint8_t hSteps;
/*vertical steps to scan*/
uint8_t vSteps;
uint8_t hDirection;
uint8_t vDirection;



//sun position 
uint8_t sunPositions[2];/////////


/*node absolute x pos*/
uint8_t nodeXPosition;
/*node absolute y pos*/
uint8_t nodeYPosition;
/* receved power*/
uint8_t receivedPower;


//receiving queue
int8_t receiveBuffer[2];
uint8_t recBufferIndex;


/* data sent flag*/
uint8_t isDataSent;
/**/
uint16_t originalH;
uint16_t originalV;



/*This function is checked for delay, motor, ADC1 and i2c initializations*/
void initSystem()    
{
	//init hardware modules
	initDelayTimer();
	initADC1();
	initServoMotor();
	i2c1Enable(STANDARD);//---------------------
	initTimeOutTimer();
	initLeds();
	//init system varibales
	nodeState = NEGOTIATION ;//startup state for testint only
//	nodeState = STARTUP ;
	nodeXPosition=NODE_X_POSITION;
	nodeYPosition=NODE_Y_POSITION;
	
	currentPower=readLDR();
	isDataSent=FALSE;
	isReceived=FALSE;
	hIndex=1;
	vIndex=1;
	isMaster=TRUE;
	
	receivedPower=0;
	
	recBufferIndex=0;
	
	vDirection=RIGTH_DIRECTION;
	hDirection =UP_DIRECTION;
	
	isRightFinished=FALSE;
	isLeftFinished=FALSE;
	
	i2c1GotoSlaveMode();//------------------
	
	moveToVertically(VERTICAL_MOTOR_INTIAL);
	moveToHorizontally(HORIZONTAL_MOTOR_INTIAL);
	
	timeOut_ms(NEGOTIATION_TIMEOUT);

}
/*******************************************************************/
/*NOT completed functions. It needs the funtions of ADC module*///---------------------------->need to complete
uint8_t readLDR(void)
{	
	//return ((readADC1()* MAX_VIN)/4096) ;//---------------
	return ((readADC1())/64) ;
}
/*******************************************************************/
/*This function change the state of the node with the passed argument*/
void setState(states state) // done 
{
	nodeState=state;
}
/*******************************************************************/
/*This function move the motor vertically with specific step and direction (up or down)*/
void moveStepVertically(uint8_t step,int8_t direction)
{
	int16_t newAngle = vAngle +(step*direction) ;
	if(newAngle < 0 )
	{
		newAngle = 0 ;
	}
	else if(newAngle > 180 )
	{
		newAngle = 180 ;
	}
	else
	{
		;
	}
	
	servoGotoVerticalAngle(newAngle);
	
	vAngle = newAngle;
}
/********************************************************************/
/*This function move the motor horizontally with specific step and direction (left or right)*/
void moveStepHorizontally(uint8_t step,int8_t direction)
{
	
	int16_t newAngle = hAngle +(step*direction) ;
	if(newAngle < 0 )
	{
		newAngle = 0 ;
	}
	else if(newAngle > 180 )
	{
		newAngle = 180 ;
	}
	else
	{
		;
	}
	
	servoGotoHorizontalAngle(newAngle);
	
	hAngle=newAngle;
}
/*********************************************************************/
/*Move the motor vertically with the passed angle*/
void moveToVertically(int16_t angle)
{
	
	servoGotoVerticalAngle(angle);
	
	vAngle=angle;
}
/*********************************************************************/
/*Move the motor horizontally with the passed angle*/
void moveToHorizontally(int16_t angle)
{
	servoGotoHorizontalAngle(angle);
	
	hAngle=angle;
}
/*********************************************************************/
/*NOT completed function*/
/*Send the power and then ****WHAT******/
void sendData(uint8_t p)
{
	i2c1SendMaster(p,NODE_ID);
	/*use goto slave funtion or not*/
	i2c1GotoSlaveMode();
}
/*********************************************************************/
/*NOT completed function*/
/*Send the power and then ****WHAT******/
//void sendPosition(uint8_t x,uint8_t y)
//{
//	uint8_t dataBuffer[2];
//	dataBuffer[0] = x ;
//	dataBuffer[1]	= y	;
//  i2c1SendMasterMultiBytes(dataBuffer,2,NODE_ID);
//	/*use goto slave funtion or not*/
//	i2c1GotoSlaveMode();
//}
/*********************************************************************/
//float getAngle(float height ,float angle,uint8_t shift)// need revision
//{
//	float tempAngle;
//	tempAngle=atan2(height , angle);
//	return tempAngle+shift;
//	
//}
//float getLength(float height,float angle,uint8_t shift) // need revision
//{

//		angle+=(-1*shift);
//		return (height/tan(angle));
//	
//}

void indicateState(states s)
{
	switch(s)
	{
		case NEGOTIATION :
			ledOn(1);
		break;
			case NORMAL :
			ledOn(3);
		break;
				case WAITING :
			ledOn(2);
		break;
				default:
					allLedsOff();
				break;
	}
}
