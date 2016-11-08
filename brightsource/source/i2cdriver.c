
#include "headers/i2cdriver.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "inc/hw_ints.h"
#include "node.h"

/********************* Functions *************************/
//***************************************************************************************/
//
// The interrupt handler for the for          I2C0             data slave interrupt.
//
//******************************************************************************/
void
I2C0_Handler(void)
{
    //
    // Clear the I2C0 interrupt flag.
    //
    I2CSlaveIntClear(I2C0_BASE);

    //
    // Read the data from the slave.
    //
    receivedPower = I2CSlaveDataGet(I2C0_BASE);

		//UARTprintf("  \n Received: %d\n", g_ui32DataRx);
		isReceived = TRUE;           //Recived Flag
}
//*****************************************************************************
//
// The interrupt handler for the for           I2C1               data slave interrupt.
//
//*****************************************************************************
void
I2C1_Handler(void)
{
    //
    // Clear the I2C1 interrupt flag.
    //
    I2CSlaveIntClear(I2C1_BASE);

    //
    // Read the data from the slave.
    //
   // receivedPower = I2CSlaveDataGet(I2C1_BASE);
	receiveBuffer[recBufferIndex]=I2CSlaveDataGet(I2C1_BASE);
	
	if(nodeState==WAITING)
	{
	recBufferIndex++;
	}
	isReceived = TRUE;           //Recived Flag
	
	
}
//*****************************************************************************
//
// The interrupt handler for the for             I2C2              data slave interrupt.
//
//*****************************************************************************
void
I2C2_Handler(void)
{
    //
    // Clear the I2C2 interrupt flag.
    //
    I2CSlaveIntClear(I2C2_BASE);

    //
    // Read the data from the slave.
    //
    receivedPower = I2CSlaveDataGet(I2C2_BASE);
		isReceived = TRUE;           //Recived Flag
}
//*****************************************************************************
//
// The interrupt handler for the for             I2C3              data slave interrupt.
//
//*****************************************************************************
void
I2C3_Handler(void)
{
    //
    // Clear the I2C3 interrupt flag.
    //
    I2CSlaveIntClear(I2C3_BASE);

    //
    // Read the data from the slave.
    //
    receivedPower = I2CSlaveDataGet(I2C3_BASE);

	  isReceived = TRUE;           //Recived Flag
}



/********************* Enable Functions *************************/
void i2c0Enable(i2cState state)													//is checked
{
	
/*************** enable I2C0 configuration ****************/
/* enable I2C0 clock */
	setBit(SYSCTL->RCGCI2C,BIT0);
/* enable portB clock */
	setBit(SYSCTL->RCGCGPIO,BIT1);
/* configure alternative function registser of portB to work as I2C */
	setBit(GPIOB->AFSEL,BIT2);
	setBit(GPIOB->AFSEL,BIT3);
/* enable digital */	
	setBit(GPIOB->DEN,BIT2);
	setBit(GPIOB->DEN,BIT3);
/* enable open drain on pin3 of portB (SDL) */
	setBit(GPIOB->ODR,BIT3);
/* configure pin[2,3] of portB to work as SDL,SCL */
	GPIOB->PCTL |= ( 0x3 << 12 ) | (0x3 << 8) ;  

/*********************************************************/

	/* configure bus speed depending on i2c state*/
	/* all the calculations are depending one the equation */
	/* TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1; */
	/* where SCL_LP and SCL_HP  are fixed to the values 4 and 6 respectivelly */
	/* and the HS bit in I2CMTPR register is 0 for all modes but high speed mode */
	switch(state)
	{
		case STANDARD :
			/* speed is 100kbps */	
			I2C0 -> MTPR = (SystemCoreClock / (2000000))-1 ;
			break;
		case FAST :
			/* speed is 400kbps */
			I2C0 -> MTPR = (SystemCoreClock / (8000000))-1 ;
			break;
		case FASTPLUS :
			/* speed is 1Mbps */
			I2C0 -> MTPR = (SystemCoreClock / (20000000))-1 ;
			break;
		case HIGHSPEED :
			/* speed is 3.33Mbps */
			I2C0 -> MTPR = (SystemCoreClock / (19980000))-1 ;
		 /* setting HS bit in both registers to run in high speed mode */
			I2C0 -> MTPR |= (1 << 7 ) ;
			setBit(I2C0->MCS , BIT4) ; 
			break;
	}
	
	/* led port init (portF)*/
			SYSCTL ->RCGCGPIO |=(1 << 5 );
			GPIOF  ->DEN |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
			GPIOF  ->DIR |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
	
}
/***************************************************************/
void i2c1Enable(i2cState state)													//is checked
{
	
/*************** enable I2C1 configuration ****************/
/* enable I2C1 clock */
	setBit(SYSCTL->RCGCI2C,BIT1);
/* enable portA clock */
	setBit(SYSCTL->RCGCGPIO,BIT0);
/* configure alternative function registser of portA to work as I2C */
	setBit(GPIOA->AFSEL,BIT6);
	setBit(GPIOA->AFSEL,BIT7);
/* enable digital */	
	setBit(GPIOA->DEN,BIT6);
	setBit(GPIOA->DEN,BIT7);
/* enable open drain on pin7 of portA (SDL) */
	setBit(GPIOA->ODR,BIT7);
/* configure pin[2,3] of portA to work as SDL,SCL */
	GPIOA->PCTL |= ( 0x3 << 24 ) | (0x3 << 28) ;  
	
/*********************************************************/

	/* configure bus speed depending on i2c state*/
	/* all the calculations are depending one the equation */
	/* TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1; */
	/* where SCL_LP and SCL_HP  are fixed to the values 4 and 6 respectivelly */
	/* and the HS bit in I2CMTPR register is 0 for all modes but high speed mode */
	switch(state)
	{
		case STANDARD :
			/* speed is 100kbps */	
			I2C1->MTPR = (SystemCoreClock / (2000000))-1 ;
			break;
		case FAST :
			/* speed is 400kbps */
			I2C1->MTPR = (SystemCoreClock / (8000000))-1 ;
			break;
		case FASTPLUS :
			/* speed is 1Mbps */
			I2C1->MTPR = (SystemCoreClock / (20000000))-1 ;
			break;
		case HIGHSPEED :
			/* speed is 3.33Mbps */
			I2C1->MTPR = (SystemCoreClock / (19980000))-1 ;
		 /* setting HS bit in both registers to run in high speed mode */
			I2C1->MTPR |= (1 << 7 ) ;
			setBit(I2C1->MCS , BIT4) ; 
			break;
	}
	
	/* led port init (portF)*/
			SYSCTL ->RCGCGPIO |=(1 << 5 );
			GPIOF  ->DEN |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
			GPIOF  ->DIR |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
	
}
/***************************************************************/
void i2c2Enable(i2cState state)													//is not checked
{
	
/*************** enable I2C2 configuration ****************/
/* enable I2C2 clock */
	setBit(SYSCTL->RCGCI2C,BIT2);
/* enable portE clock */
	setBit(SYSCTL->RCGCGPIO,BIT4);
/* configure alternative function registser of portE to work as I2C */
	setBit(GPIOE->AFSEL,BIT4);
	setBit(GPIOE->AFSEL,BIT5);
/* enable digital */	
	setBit(GPIOE->DEN,BIT4);
	setBit(GPIOE->DEN,BIT5);
/* enable open drain on pin5 of portE (SDL) */
	setBit(GPIOE->ODR,BIT5);
/* configure pin[4,5] of portE to work as SDL,SCL */
	GPIOE->PCTL |= ( 0x3 << 20 ) | (0x3 << 16) ;  

/*********************************************************/

	/* configure bus speed depending on i2c state*/
	/* all the calculations are depending one the equation */
	/* TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1; */
	/* where SCL_LP and SCL_HP  are fixed to the values 4 and 6 respectivelly */
	/* and the HS bit in I2CMTPR register is 0 for all modes but high speed mode */
	switch(state)
	{
		case STANDARD :
			/* speed is 100kbps */	
			I2C0 -> MTPR = (SystemCoreClock / (2000000))-1 ;
			break;
		case FAST :
			/* speed is 400kbps */
			I2C0 -> MTPR = (SystemCoreClock / (8000000))-1 ;
			break;
		case FASTPLUS :
			/* speed is 1Mbps */
			I2C0 -> MTPR = (SystemCoreClock / (20000000))-1 ;
			break;
		case HIGHSPEED :
			/* speed is 3.33Mbps */
			I2C0 -> MTPR = (SystemCoreClock / (19980000))-1 ;
		 /* setting HS bit in both registers to run in high speed mode */
			I2C0 -> MTPR |= (1 << 7 ) ;
			setBit(I2C0->MCS , BIT4) ; 
			break;
	}
	
	/* led port init (portF)*/
			SYSCTL ->RCGCGPIO |=(1 << 5 );
			GPIOF  ->DEN |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
			GPIOF  ->DIR |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
	
}
/***************************************************************/
void i2c3Enable(i2cState state)													//is not checked 
{
	
/*************** enable I2C3 configuration ****************/
/* enable I2C3 clock */
	setBit(SYSCTL->RCGCI2C,BIT3);
/* enable portD clock */
	setBit(SYSCTL->RCGCGPIO,BIT3); 
/* configure alternative function registser of portD to work as I2C */
	setBit(GPIOD->AFSEL,BIT0);
	setBit(GPIOD->AFSEL,BIT1);
/* enable digital */	
	setBit(GPIOD->DEN,BIT0);
	setBit(GPIOD->DEN,BIT1);
/* enable open drain on pin1 of portD (SDL) */
	setBit(GPIOD->ODR,BIT1);
/* configure pin[0,1] of portD to work as SDL,SCL */
	GPIOD->PCTL |= ( 0x3 << 4 ) | (0x3 << 0) ;  

/*********************************************************/

	/* configure bus speed depending on i2c state*/
	/* all the calculations are depending one the equation */
	/* TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1; */
	/* where SCL_LP and SCL_HP  are fixed to the values 4 and 6 respectivelly */
	/* and the HS bit in I2CMTPR register is 0 for all modes but high speed mode */
	switch(state)
	{
		case STANDARD :
			/* speed is 100kbps */	
			I2C0 -> MTPR = (SystemCoreClock / (2000000))-1 ;
			break;
		case FAST :
			/* speed is 400kbps */
			I2C0 -> MTPR = (SystemCoreClock / (8000000))-1 ;
			break;
		case FASTPLUS :
			/* speed is 1Mbps */
			I2C0 -> MTPR = (SystemCoreClock / (20000000))-1 ;
			break;
		case HIGHSPEED :
			/* speed is 3.33Mbps */
			I2C0 -> MTPR = (SystemCoreClock / (19980000))-1 ;
		 /* setting HS bit in both registers to run in high speed mode */
			I2C0 -> MTPR |= (1 << 7 ) ;
			setBit(I2C0->MCS , BIT4) ; 
			break;
	}
	
	/* led port init (portF)*/
			SYSCTL ->RCGCGPIO |=(1 << 5 );
			GPIOF  ->DEN |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
			GPIOF  ->DIR |=((1 << 1) | ( 1 << 2 ) | (1 << 3 ));
	
}

/***************************************************************/
/***************** Send single byte Functions ******************/
void i2c0SendMaster(uint8_t data,uint8_t slaveAddress)	//is checked
{
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C0->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C0 ->MSA |=(slaveAddress << 1 );
	clearBit(I2C0 ->MSA,BIT0);
	I2C0 -> MDR = data ;   
	/*Initiate a single byte transmit of the data from Master to Slave by writing the I2CMCS register
	with a value of 0x0000.0007 (STOP, START, RUN).*/
	I2C0 -> MCS	= 7 ; 
	/* checking busy bit  to check if the transmission is complete*/ 
	while( (I2C0->MCS & (1 << 0)) == 1 ) ;

}
/***************************************************************/
void i2c1SendMaster(uint8_t data,uint8_t slaveAddress)  //is checked
{
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C1->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C1 ->MSA |=(slaveAddress << 1 ); 
	clearBit(I2C1 ->MSA,BIT0);
	I2C1 -> MDR = data ;   
	/*Initiate a single byte transmit of the data from Master to Slave by writing the I2CMCS register
	with a value of 0x0000.0007 (STOP, START, RUN).*/

	//////	I2C1 -> MCS	= 7 ; 
	
	/* checking busy bit  to check if the transmission is complete*/ 
	
	/////while( (I2C1->MCS & (1 << 0)) == 1 ) ;
	if((I2C1->MCS & (1 << 0)) == 0 )//-------------------------------------------------------------------------------------
	{
		I2C1 -> MCS	= 7 ; 
		isDataSent = TRUE;
	}
	
}
/***************************************************************/
void i2c2SendMaster(uint8_t data,uint8_t slaveAddress)  //is not checked
{
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C2->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C2 ->MSA |=(slaveAddress << 1 ); 
	clearBit(I2C2 ->MSA,BIT0);
	I2C2 -> MDR = data ;   
	/*Initiate a single byte transmit of the data from Master to Slave by writing the I2CMCS register
	with a value of 0x0000.0007 (STOP, START, RUN).*/
	I2C2 -> MCS	= 7 ; 
	/* checking busy bit  to check if the transmission is complete*/ 
	while( (I2C2->MCS & (1 << 0)) == 1 ) ;
}
/***************************************************************/
void i2c3SendMaster(uint8_t data,uint8_t slaveAddress)  //is not checked
{
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C3->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C3 ->MSA |=(slaveAddress << 1 ); 
	clearBit(I2C3->MSA,BIT0);
	I2C3 -> MDR = data ;   
	/*Initiate a single byte transmit of the data from Master to Slave by writing the I2CMCS register
	with a value of 0x0000.0007 (STOP, START, RUN).*/
	I2C3 -> MCS	= 7 ; 
	/* checking busy bit  to check if the transmission is complete*/ 
	while( (I2C3->MCS & (1 << 0)) == 1 ) ;
}
/***************************************************************/
uint8_t i2c0SlaveReceive(void)													//is checked 
{
	uint8_t data ;
	/* Initialize the I2C Slave by writing the I2CMCR register with */
  /* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
	I2C0->MCR = 0x20 ;
	/* set slave own address */
	I2C0 ->SOAR = 0x3b ;
	/* enable slave operation */
	setBit(I2C0->SCSR , BIT0 ) ;
	/* check if any valid data received */
	while((I2C0->SRIS & (1 << BIT0 )) != 1);
	/* receive data */
	data = I2C0->SDR ;
	/* clearing received data flag */
	setBit(I2C0 ->SICR , BIT0 );
	/* returning data received */
	return data ;
	
}
/***************************************************************/
uint8_t i2c1SlaveReceive(void)													//is checked
{
	uint8_t data ;
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
	I2C1->MCR = 0x20 ;
	/* set slave own address */
	I2C1 ->SOAR = 0x3b ;
	/* enable slave operation */
	setBit(I2C1->SCSR , BIT0 ) ;
	/* check if any valid data received */
	while((I2C1->SCSR & (1 << BIT0 )) != 1);
	/* receive data */
	data = I2C1->SDR ;
	/* clearing received data flag */
	setBit(I2C1 ->SICR , BIT0 );
	/* returning data received */
	return data ;
	
}
/***************************************************************/
uint8_t i2c2SlaveReceive(void)													//is checked
{
	uint8_t data ;
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
	I2C2->MCR = 0x20 ;
	/* set slave own address */
	I2C2 ->SOAR = 0x3b ;
	/* enable slave operation */
	setBit(I2C2->SCSR , BIT0 ) ;
	/* check if any valid data received */
	while((I2C2->SCSR & (1 << BIT0 )) != 1);
	/* receive data */
	data = I2C2->SDR ;
	/* clearing received data flag */
	setBit(I2C2 ->SICR , BIT0 );
	/* returning data received */
	return data ;
	
}
/***************************************************************/
uint8_t i2c3SlaveReceive(void)													//is checked
{
	uint8_t data ;
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
	I2C3->MCR = 0x20 ;
	/* set slave own address */
	I2C3 ->SOAR = 0x3b ;
	/* enable slave operation */
	setBit(I2C3->SCSR , BIT0 ) ;
	/* check if any valid data received */
	while((I2C3->SCSR & (1 << BIT0 )) != 1);
	/* receive data */
	data = I2C3->SDR ;
	/* clearing received data flag */
	setBit(I2C3 ->SICR , BIT0 );
	/* returning data received */
	return data ;
	
}
/***************************************************************/
/***************** Send multi bytes Functions ******************/
void i2c0SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress) //is checked
{
	int counter ;
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C0->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C0 ->MSA |=(slaveAddress << 1 ); 
	clearBit(I2C0->MSA,BIT0);
	
	/* send first byte */
	I2C0->MDR = *data;
	/* start and run */
	setBit(I2C0-> MCS,BIT0);
	setBit(I2C0-> MCS,BIT1); 
	/* send the rest data except the last byte*/
	for(counter = 1 ; counter < dataSize -2;counter ++)
	{
		/* checking busy bit in I2CMCS to check if the transmission is complete */ 
		while((checkBit(I2C0->MCS,BIT0)) != 0 ) ;
		/* Increment the pointer */
		data ++;
		/* send the data */
		I2C0 -> MDR = *data ; 
		/* set RUN bit only in I2CMCS */
		setBit(I2C0-> MCS,BIT0);
	}
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C0->MCS,BIT0)) != 0 ) ;
	/* Increment the pointer to send the last byte */
	I2C0 -> MDR = *(data+1);
	/* set both RUN and STOP bits in I2CMCS */
	setBit(I2C0->MCS,BIT0);
	setBit(I2C0->MCS,BIT2);
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C0->MCS,BIT0)) != 0 ) ;
		
}
/***************************************************************/
void i2c1SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress) //is checked
{
	int counter ;
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C1->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C1 ->MSA |=(slaveAddress << 1 ); 
	clearBit(I2C1->MSA,BIT0);
	
	/* send first byte */
	I2C1->MDR = *data;
	/* start and run */
	setBit(I2C1-> MCS,BIT0);
	setBit(I2C1-> MCS,BIT1); 
	/* send the rest data except the last byte*/
	for(counter = 1 ; counter < dataSize -2;counter ++)
	{
		/* checking busy bit in I2CMCS to check if the transmission is complete */ 
		while((checkBit(I2C1->MCS,BIT0)) != 0 ) ;
		/* Increment the pointer */
		data ++;
		/* send the data */
		I2C1 -> MDR = *data ; 
		/* set RUN bit only in I2CMCS */
		setBit(I2C1-> MCS,BIT0);
	}
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C1->MCS,BIT0)) != 0 ) ;
	/* Increment the pointer to send the last byte */
	I2C1 -> MDR = *(data+1);
	/* set both RUN and STOP bits in I2CMCS */
	setBit(I2C1->MCS,BIT0);
	setBit(I2C1->MCS,BIT2);
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C1->MCS,BIT0)) != 0 ) ;
		
}
/***************************************************************/
void i2c2SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress) //is checked
{
	int counter ;
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C2->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C2 ->MSA |=(slaveAddress << 1 ); 
	clearBit(I2C2->MSA,BIT0);
	
	/* send first byte */
	I2C2->MDR = *data;
	/* start and run */
	setBit(I2C2-> MCS,BIT0);
	setBit(I2C2-> MCS,BIT1); 
	/* send the rest data except the last byte*/
	for(counter = 1 ; counter < dataSize -2;counter ++)
	{
		/* checking busy bit in I2CMCS to check if the transmission is complete */ 
		while((checkBit(I2C2->MCS,BIT0)) != 0 ) ;
		/* Increment the pointer */
		data ++;
		/* send the data */
		I2C2 -> MDR = *data ; 
		/* set RUN bit only in I2CMCS */
		setBit(I2C2-> MCS,BIT0);
	}
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C2->MCS,BIT0)) != 0 ) ;
	/* Increment the pointer to send the last byte */
	I2C2 -> MDR = *(data+1);
	/* set both RUN and STOP bits in I2CMCS */
	setBit(I2C2->MCS,BIT0);
	setBit(I2C2->MCS,BIT2);
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C2->MCS,BIT0)) != 0 ) ;
		
}
/***************************************************************/
void i2c3SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress) //is checked
{
	int counter ;
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	setBit(I2C3->MCR,BIT4);
	/* applying master slave address with default write operation */
	I2C3 ->MSA |=(slaveAddress << 1 ); 
	clearBit(I2C3->MSA,BIT0);
	
	/* send first byte */
	I2C3->MDR = *data;
	/* start and run */
	setBit(I2C3-> MCS,BIT0);
	setBit(I2C3-> MCS,BIT1); 
	/* send the rest data except the last byte*/
	for(counter = 1 ; counter < dataSize -2;counter ++)
	{
		/* checking busy bit in I2CMCS to check if the transmission is complete */ 
		while((checkBit(I2C3->MCS,BIT0)) != 0 ) ;
		/* Increment the pointer */
		data ++;
		/* send the data */
		I2C3 -> MDR = *data ; 
		/* set RUN bit only in I2CMCS */
		setBit(I2C3-> MCS,BIT0);
	}
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C3->MCS,BIT0)) != 0 ) ;
	/* Increment the pointer to send the last byte */
	I2C3 -> MDR = *(data+1);
	/* set both RUN and STOP bits in I2CMCS */
	setBit(I2C3->MCS,BIT0);
	setBit(I2C3->MCS,BIT2);
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C3->MCS,BIT0)) != 0 ) ;
		
}

/****************************************************************/
void i2c0GotoSlaveMode(void)																		//is checked 
{
	
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
	//I2C0->MCR = 0x20 ;
	/* set slave own address */
	//I2C0 ->SOAR = 0x3b ;
	/* enable slave operation */
	//setBit(I2C0->SCSR , BIT0 ) ;
	/***********************************************************************/
	IntEnable(INT_I2C0);                
    // Configure and turn on the I2C0 slave interrupt.  The I2CSlaveIntEnableEx()
    // gives you the ability to only enable specific interrupts.  For this case
    // we are only interrupting when the slave device receives data.
   
   I2CSlaveIntEnable(I2C0_BASE);
	 
	 I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);//----------------------------------

// Enable the I2C0 slave module.
   
    I2CSlaveEnable(I2C0_BASE);
 
 // Set the slave address to SLAVE_ADDRESS.  
    
    I2CSlaveInit(I2C0_BASE, 0x3B);
}
/****************************************************************/
void i2c1GotoSlaveMode(void)																		//is checked 
{
	
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
//	I2C1->MCR = 0x20 ;
	/* set slave own address */
//	I2C1 ->SOAR = 0x3b ;
	/* enable slave operation */
//	setBit(I2C1->SCSR , BIT0 ) ;
		IntEnable(INT_I2C1);                
    // Configure and turn on the I2C0 slave interrupt.  The I2CSlaveIntEnableEx()
    // gives you the ability to only enable specific interrupts.  For this case
    // we are only interrupting when the slave device receives data.
   
   I2CSlaveIntEnable(I2C1_BASE);
	 
	 I2CSlaveIntEnableEx(I2C1_BASE, I2C_SLAVE_INT_DATA);//----------------------------------

// Enable the I2C0 slave module.
   
    I2CSlaveEnable(I2C1_BASE);
 
 // Set the slave address to SLAVE_ADDRESS.  
    
    I2CSlaveInit(I2C1_BASE, 0x3B);
}
/****************************************************************/
void i2c2GotoSlaveMode(void)																		//is checked 
{
	
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
	//I2C2->MCR = 0x20 ;
	/* set slave own address */
	//I2C2 ->SOAR = 0x3b ;
	/* enable slave operation */
//	setBit(I2C2->SCSR , BIT0 ) ;
		IntEnable(INT_I2C2);                
    // Configure and turn on the I2C0 slave interrupt.  The I2CSlaveIntEnableEx()
    // gives you the ability to only enable specific interrupts.  For this case
    // we are only interrupting when the slave device receives data.
   
   I2CSlaveIntEnable(I2C2_BASE);
	 
	 I2CSlaveIntEnableEx(I2C2_BASE, I2C_SLAVE_INT_DATA);//----------------------------------

// Enable the I2C0 slave module.
   
    I2CSlaveEnable(I2C2_BASE);
 
 // Set the slave address to SLAVE_ADDRESS.  
    
    I2CSlaveInit(I2C2_BASE, 0x3B);
}
/****************************************************************/
void i2c3GotoSlaveMode(void)																		//is checked
{
	
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
//	I2C3->MCR = 0x20 ;
	/* set slave own address */
//	I2C3 ->SOAR = 0x3b ;
	/* enable slave operation */
//	setBit(I2C3->SCSR , BIT0 ) ;
		IntEnable(INT_I2C3);                
    // Configure and turn on the I2C0 slave interrupt.  The I2CSlaveIntEnableEx()
    // gives you the ability to only enable specific interrupts.  For this case
    // we are only interrupting when the slave device receives data.
   
   I2CSlaveIntEnable(I2C3_BASE);
	 
	 I2CSlaveIntEnableEx(I2C3_BASE, I2C_SLAVE_INT_DATA);//----------------------------------

// Enable the I2C0 slave module.
   
    I2CSlaveEnable(I2C3_BASE);
 
 // Set the slave address to SLAVE_ADDRESS.  
    
    I2CSlaveInit(I2C3_BASE, 0x3B);
}
