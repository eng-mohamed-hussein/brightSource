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
                 
//#include "headers/basics.h"
//#include "headers/i2cdriver.h"
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

int main ()
{
	uint8_t tempValue=0;
	InitConsole();
	UARTprintf("  \n SYSTEM started");
	/*
	double newHAngle,newVAngle;
	float xLength,yLength,sum;
	float relativeXSun,relativeYSun;
	int originalH,originalY;
	*/
	initSystem();
		UARTprintf("  \n currentPower value:%d\n",currentPower);	//----------------------------------------------
	UARTprintf("  \n SYSTEM INIT DONE");//---------------------------------------------
	while(1)
	{
		
		//tempValue=readLDR();
			
	
	//	UARTprintf("  \n temp value:%d\n",tempValue);	
		
		switch(nodeState)
		{
		//	UARTprintf("  \n current State : %s\n",st[nodeState]);//-------------------------------------------
			
			case NEGOTIATION:																					//nego. state
			UARTprintf("  \n Entered NEGO  State\n");	//--------------------------------------
			UARTprintf("  \n received power %d and receved flag is %d\n",receivedPower,isReceived);
			if((isReceived==TRUE) && (receivedPower>currentPower))
			{
				isMaster=FALSE;
				isReceived=FALSE;
				isDataSent=TRUE;          //---------------------------
				UARTprintf("  \n recived > currentpower being slave\n");	//----------------------------------------------
			}
				//------------------------
			if((isMaster==TRUE) && (isDataSent==FALSE))
			{
				//send current power
				//cout<<"\nsending current power "<<id<<" \n";
//				if(sendPower(currentPower)==TRUE)
//				{
//					isDataSent=TRUE;
//				}
						//----------------------------------------------
				sendData(currentPower);
				UARTprintf("\ncurrent power sent\n");
			//	isDataSent=TRUE;
				
			}
/*		
			if(time > NEGOTIATION_TIMEOUT)
			{
			//	isNegotiate=FALSE;

				if(isMaster == TRUE)
				{
//					setState(SCANNING);
						setState(NORMAL);//////////for test
					//cout<<"node "<<id<<" is master\n";
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

					//locker1.lock();
					//m.lock();
				}
				else 
				{
					//	setState(WAITING);
					setState(NORMAL);//////////for test
						//cout<<"node "<<id<<" is slave\n";
				}
				//break;
				isReceived=FALSE;
			}*/
				break;
			/********************************************************************************************/
			//
			
			//
			/********************************************************************************/
			case SCANNING:
				break;
			case NORMAL:
				//cout<<"\nnode "<<id<<"receive state is "<<isReceived<<"\n";
		//	cout<<"\nnode "<<id<<"tempValue "<<tempValue<<"current power "<<getCurrentPower()<<"\n";
		//	cout<<"\nnode "<<id<<"receive state is "<<isReceived<<"\n";
				UARTprintf("  \n Entered Normal state \n");	//----------------------------------------------
			if(isMaster==TRUE)
			{
					UARTprintf("  \n being master \n");	//----------------------------------------------
			//	cout<<"\nnode "<<id<<" is waiting for other nodes\n";
			//	delay(MASTER_WAIT_TIME);
				isMaster=FALSE;
			//	cout<<"\nnode "<<id<<" is running now\n";
			}
			tempValue=readLDR();
				UARTprintf("  \n Current power:%d\n",currentPower);	//----------------------------------------------
			UARTprintf("  \n temp value:%d\n",tempValue);	//----------------------------------------------
			if(tempValue-currentPower>DELTA_POWER || (isReceived==TRUE))
			{
					UARTprintf("  \n checking delta_power with tempvlaue...\n");	//----------------------------------------------
				//cout<<"\nnode "<<id<<" need to send "<<currentPower<<"\n";
				currentPower=tempValue;
				setState(NEGOTIATION);
				isMaster=TRUE;
				isReceived==FALSE;//---------------------------------------
				isDataSent=FALSE;//-------------------
			//	time=0;
				//t=0;
				//startTime=std::chrono::system_clock::now();
				//startTimer();
				UARTprintf("\njumping to negotiation state\n");
			
				timeOut_ms(NEGOTIATION_TIMEOUT);//----------------
			}
			
	
				break;
			case RECEIVING:
				break;
			case SENDING:
				break;
			case WAITING:
				break;
			case TRACKING:
				break;
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
	
	//TBD
	if(isMaster == TRUE)
				{
//					setState(SCANNING);
						setState(NORMAL);//////////for test
					//cout<<"node "<<id<<" is master\n";
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

					//locker1.lock();
					//m.lock();
				}
				else 
				{
					//	setState(WAITING);
					setState(NORMAL);//////////for test
						//cout<<"node "<<id<<" is slave\n";
				}
				//break;
				isReceived=FALSE;
				receivedPower=0;//--------
	
	//
	
}