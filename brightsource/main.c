#include <tm4c123gh6pm.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
                 
#include "node.h"
#include "timeouttimer.h"

void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
	
}
char *st[]={"STARTUP","NEGOTIATION","SCANNING","NORMAL","RECEIVING","SENDING","WAITING","TRACKING","SLEEPING"};

uint8_t tempValue=0;

int main ()
{
	/********************************************/
	//buffer to hold floating value to print at uart`
	/*************************************************/
	char floatbuffer[10];
	
	int count=800;
	//uint8_t tempValue=0;
	float xLength,yLength,sum;
	float relativeXSun,relativeYSun,newVAngle,newHAngle;
	
	InitConsole();
	UARTprintf("  \n SYSTEM started");
	initSystem();
	
	
	
	UARTprintf("  \n currentPower value:%d\n",currentPower);	//----------------------------------------------
	UARTprintf("  \n SYSTEM INIT DONE");//---------------------------------------------
	UARTprintf("  \n current State : %s\n",st[nodeState]);
	while(count--)
	{
		
		//tempValue=readLDR();
			
	
	//	UARTprintf("  \n temp value:%d\n",tempValue);	
		
		//UARTprintf("  \n current State : %s\n",st[nodeState]);//-------------------------------------------
		indicateState(nodeState);
		
		switch(nodeState)
		{
					//	UARTprintf("  \n current State : %s\n",st[nodeState]);//-------------------------------------------
			
			case NEGOTIATION:		
			{			
				//nego. state
		//	UARTprintf("  \n Entered NEGO  State\n");	//--------------------------------------
	
				UARTprintf("  \n received power %d and receved flag is %d\n",receiveBuffer[0],isReceived);
		
		
			if((isReceived==TRUE) && (receiveBuffer[0]>currentPower))
			{
				isMaster=FALSE;
				isReceived=FALSE;
				isDataSent=TRUE;          //---------------------------
				UARTprintf("\n ####### node is slave\n");	//----------------------------------------------
			}
				//------------------------
			if((isMaster==TRUE) && (isDataSent==FALSE))
			{
				//send current power
				
//				if(sendPower(currentPower)==TRUE)
//				{
//					isDataSent=TRUE;
//				}
						//----------------------------------------------
				sendData(currentPower);
				UARTprintf("\n##### power sent\n");
			//	isDataSent=TRUE;
				
			}
				break;
		}
			/***********************************   SCANIng    *********************************************/
			case SCANNING:
			{
			UARTprintf("\nhIndex:%d ,vIndex:%d\n",hIndex,vIndex);
			UARTprintf("\nis right:%d ,is left:%d\n",isRightFinished,isLeftFinished);
				UARTprintf("\nv angle:%d ,h angle:%d\n",vAngle,hAngle);
				if( hIndex>hSteps && vIndex>vSteps)
				{
					if(isRightFinished & isLeftFinished)//&&
					{
						moveToHorizontally(maxHPosition);
						moveToVertically(maxVPosition);
					//	delay_ms(2000);
						currentPower=readLDR();
		//				currentPower = maxPower;   //for simulation
						if(isMaster)
						{
							setState(SENDING);
						}
						else
						{
							setState(NORMAL);
						}
						break;
					}
					else if(isRightFinished & !isLeftFinished)//&&
					{
						isLeftFinished=TRUE;
						break;
					}
					else
					{
						isRightFinished=TRUE;
						hAngle=originalH;
						vAngle=originalV;
					//	cout<<"\nreset motor rotation \n";
						moveToHorizontally(hAngle);
						moveToVertically(vAngle);
						hIndex=1;
						vIndex=1;
						vDirection=LEFT_DIRECTION;
				
						break;
					}

				}

			
				if(hIndex<= hSteps)
				{
					moveStepHorizontally(MOTOR_ROTATION_STEPS,hDirection);
					hIndex++;
				}
				
				if(vIndex<= vSteps)
				{
				moveStepVertically(MOTOR_ROTATION_STEPS,vDirection);
				vIndex++;
				}
									//	read ldr
	
				tempValue=readLDR();
		
				UARTprintf("\nLDR:%d ,maxPower:%d\n",tempValue,maxPower);
				if(tempValue < (maxPower-4))//ADD EQUAL COND
				{
					
					hIndex=(hSteps+1);
					vIndex=(vSteps+1);
					UARTprintf("\ncancel this direction\n");
				}
				else if(tempValue > (maxPower+4))
				{
				
					maxPower=tempValue;
					maxVPosition=vAngle;
					maxHPosition=hAngle;
					UARTprintf("\ndetect high power in v : %d, h : %d\n",maxVPosition,maxHPosition);
				}
				
				else
				{
					
				}
	
				break;}
			/************************           NORMAL     ************************************************/
			case NORMAL:
			{
			count++; 
				//UARTprintf("  \n Entered Normal state \n");	//----------------------------------------------
			if(isMaster==TRUE)
			{
					UARTprintf("  \n being master \n");	//----------------------------------------------
		
				isMaster=FALSE;
		
			}
			tempValue=readLDR();
			//	UARTprintf("  \n Current power:%d\n",currentPower);	//----------------------------------------------
			//UARTprintf("  \n temp value:%d\n",tempValue);	//----------------------------------------------
			if(tempValue-currentPower>DELTA_POWER || (isReceived==TRUE))
			{
					UARTprintf("  \n checking delta_power with tempvlaue...\n");	//----------------------------------------------
				UARTprintf("  \n Current power:%d\n",currentPower);
				UARTprintf("  \n temp value:%d\n",tempValue);
				currentPower=tempValue;
				setState(NEGOTIATION);
				isMaster=TRUE;
				isReceived==FALSE;//---------------------------------------
				isDataSent=FALSE;//-------------------
			
				count+=1600;
				
				UARTprintf("\njumping to negotiation state\n");
			
				timeOut_ms(NEGOTIATION_TIMEOUT);//----------------
			}
			
	
				break;}
/******************************** RECIVING ************************************/		
			case RECEIVING:
			{
				isReceived=FALSE; 
					//compute xlength ylenght
			UARTprintf("\nsun received positions x: %d , y: %d\n",sunPositions[0],sunPositions[1]);
					 xLength=sunPositions[0]-nodeXPosition;
					 yLength=sunPositions[1]-nodeYPosition;
					 newVAngle=radiansToDegrees(atan2(xLength,yLength));
					//float sqr=sqrt(x*x+y*y);
				//	newHAngle=radiansToDegrees(atan2((SUN_HEIGHT-HORIZONTAL_MOTOR_HEIGHT),yLength));
					newHAngle=radiansToDegrees(atan2(yLength,(SUN_HEIGHT-HORIZONTAL_MOTOR_HEIGHT)));
					
				snprintf(floatbuffer,80,"newVAngle pre :%f\n",newVAngle);
			UARTprintf(floatbuffer);
				snprintf(floatbuffer,80,"newHAngle pre :%f\n",newHAngle);
			UARTprintf(floatbuffer);
				
			newHAngle=fabs(newHAngle);
			newVAngle=fabs(newVAngle);
				
				snprintf(floatbuffer,80,"newVAngle abs :%f\n",newVAngle);
			UARTprintf(floatbuffer);
				snprintf(floatbuffer,80,"newHAngle abs :%f\n",newHAngle);
			UARTprintf(floatbuffer);
			
			//------
			if(sunPositions[0]>nodeXPosition)
			{
				newVAngle*=-1;
			}
			
			if(sunPositions[1]>nodeYPosition)
			{
				newHAngle*=-1;
			}
			
			snprintf(floatbuffer,80,"newVAngle pre sh :%f\n",newVAngle);
			UARTprintf(floatbuffer);
				snprintf(floatbuffer,80,"newHAngle pre sh :%f\n",newHAngle);
			UARTprintf(floatbuffer);
			newVAngle-=VERTICAL_SHIFT;
					 newHAngle-=HORIZONTAL_SHIFT;
			
			snprintf(floatbuffer,80,"newHAngle :%f\n",newHAngle);
			UARTprintf(floatbuffer);//-----------------------------------
				snprintf(floatbuffer,80,"newVAngle :%f\n",newVAngle);
			UARTprintf(floatbuffer);//-----------------------------------	
		
			setState(TRACKING);
				break;
			}
/****************************    SENDING     ***************************************************/			
			case SENDING:
			{	
				//compute relative sun positions
			hAngle+=HORIZONTAL_SHIFT;//-------------------
			vAngle+=VERTICAL_SHIFT;//------------------
		//	relativeYSun=(SUN_HEIGHT-HORIZONTAL_MOTOR_HEIGHT)/(tan(degreesToRadians(hAngle)));
			relativeYSun=(SUN_HEIGHT-HORIZONTAL_MOTOR_HEIGHT)*(tan(degreesToRadians(hAngle)));
	
			relativeXSun=(relativeYSun*tan(degreesToRadians(vAngle)));

			relativeXSun=fabs(relativeXSun);
			relativeYSun=fabs(relativeYSun);
			
			snprintf(floatbuffer,80,"relativeXSun :%f\n",relativeXSun);
			UARTprintf(floatbuffer);//-----------------------------------
			
			snprintf(floatbuffer,80,"relativeYSun :%f\n",relativeYSun);
			UARTprintf(floatbuffer);//------------------------------
			
			
			if(vAngle<0)
			{
				sunPositions[0]=nodeXPosition+relativeXSun;
			}
			else
			{
				sunPositions[0]=nodeXPosition-relativeXSun;
			}
			if(hAngle<0)
			{
				sunPositions[1]=nodeYPosition+relativeYSun;
			}
			else
			{
				sunPositions[1]=nodeYPosition-relativeYSun;
			}
		//	snprintf(floatbuffer,80,"XSun :%f\n",sunPositions[0]);
		//	UARTprintf(floatbuffer);//-----------------------------------
			UARTprintf("\nXSun :%d\n",sunPositions[0]);//-----------------------
			UARTprintf("\nXSun :%d\n",sunPositions[1]);//--------------
		//	snprintf(floatbuffer,80,"YSun :%f\n",sunPositions[1]);
		//	UARTprintf(floatbuffer);//------------------------------
			
			sendData(sunPositions[0]);
			delay_ms(15);
			sendData(sunPositions[1]);
			
			UARTprintf("\n    sun position sent\n");
			UARTprintf("  \n Entered Normal state \n");
			isReceived=FALSE;
			setState(NORMAL);
				break;}
/*******************************  WAITING  *************************/			
			case WAITING:
			{
				
				while(recBufferIndex<=1);//polling on received buffer
				
				sunPositions[0]=receiveBuffer[0];
				sunPositions[1]=receiveBuffer[1];
//				if(isReceived==TRUE)
//			{
//				
//				sunPositions[tempValue]=receivedPower;//tempvalue need to be intialized
//				
//				tempValue++;
//				isReceived=FALSE;
//			}
			
		UARTprintf("\n#########received [1]: %d  ,[2] : %d\n",sunPositions[0],sunPositions[1]);
//			if(tempValue>1)
//			{
//				 setState(RECEIVING);
//			}
				
			 isReceived=FALSE;
			 
			 recBufferIndex=0;
			 
				setState(RECEIVING);
				
				break;
			}
/*****************************          TRACKING  *******************/ 
			case TRACKING:
			{
				moveToHorizontally(newHAngle);
			moveToVertically(newVAngle);
			currentPower=readLDR();
			//init scanning
			hIndex=1;
			vIndex=1;
			hSteps=4;
			vSteps=4;
			maxPower=currentPower;
			maxVPosition=vAngle;
			maxHPosition=hAngle;
			isRightFinished=FALSE;
			isLeftFinished=FALSE;
				vDirection=RIGTH_DIRECTION;
	hDirection =UP_DIRECTION;
				originalH=hAngle;
					originalV=vAngle;
			setState(SCANNING);
				
				UARTprintf("  \n Entered scanning state \n");
				recBufferIndex=0;
			//setState(NORMAL);
				break;
			}
/************************SLEEPING  **************************************/			
			case SLEEPING:
				break;
				
		}
	
	}
	
	return 0;
}

void TIMER1A_Handler(void)
{
	/* clear time out flag */
	setBit(TIMER1 -> ICR,BIT0);
	
	UARTprintf("  \n negotiation time finished");
	
	recBufferIndex=0;//----------------------------
	
	//TBD
	if(isMaster == TRUE)
				{
					setState(SCANNING);
//						setState(NORMAL);//////////for test
					hIndex=1;
					vIndex=1;
					hSteps=SCANING_ANGLES;
					vSteps=SCANING_ANGLES;
					maxPower=currentPower;
					maxVPosition=vAngle;
					maxHPosition=hAngle;
					isLeftFinished=FALSE;
					isRightFinished=FALSE;
					hDirection=UP_DIRECTION;
					vDirection=RIGTH_DIRECTION;
					originalH=hAngle;
					originalV=vAngle;
				}
				else 
				{
						setState(WAITING);
		//			setState(NORMAL);//////////for test
						
				}
				//break;
				isReceived=FALSE;
				receivedPower=0;//--------
	
	//
	
}