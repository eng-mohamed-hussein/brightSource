
#include "I2C.h"

/********************* Functions *************************/

void i2c0Enable(i2cState state)													//is checked
{
	
/*************** enable I2C0 configuration ****************/
/* enable I2C clock */
	SYSCTL->RCGCI2C |=(1 << 0 );
/* enable portB clock */
	SYSCTL->RCGCGPIO |= (1 << 1 ); 
	/* configure alternative function registser of portB to work as I2C */
	GPIOB->AFSEL  |=((1 << 2 ) | (1 << 3 )); 
	/* enable digital */	
	GPIOB->DEN |= ((1 << 2 ) | ( 1 << 3 )); 
/* enable open drain on pin3 of portB (SDL) */
	GPIOB->ODR |=(1 << 3) ; 
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
	
/*************** enable I2C configuration ****************/
/* enable I2C clock */
	SYSCTL->RCGCI2C |=(1 << 1 ); 
/* enable portB clock */
	SYSCTL->RCGCGPIO |= (1 << 0 );
/* configure alternative function registser of portB to work as I2C */
	GPIOA->AFSEL  |=((1 << 6 ) | (1 << 7 )); 
/* enable digital */	
	GPIOA->DEN |= ((1 << 6 ) | ( 1 << 7 )); 
/* enable open drain on pin3 of portB (SDL) */
	GPIOA->ODR |=(1 << 7) ; //true
/* configure pin[2,3] of portB to work as SDL,SCL */
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
void i2cSendMasterI2C1(uint8_t data,uint8_t slaveAddress)  //is checked
{
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	I2C1->MTPR = (SystemCoreClock / (2000000))-1 ;
	I2C1->MCR = 0x10  ; 
	/* applying master slave address with default write operation */
	I2C1 ->MSA |=(slaveAddress << 1 ); 
	I2C1 ->MSA &=~(1 << 0 ); 
	I2C1 -> MDR = data ;   
	/*Initiate a single byte transmit of the data from Master to Slave by writing the I2CMCS register
	with a value of 0x0000.0007 (STOP, START, RUN).*/
	I2C1 -> MCS	= 7 ; 
	/* checking busy bit  to check if the transmission is complete*/ 
	while( (I2C1->MCS & (1 << 0)) == 1 ) ;
}
/***************************************************************/
void i2cSendMaster(uint8_t data,uint8_t slaveAddress)			//is checked this for I2C0
{
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	I2C0->MCR |=(1 << 4)  ; 
	/* applying master slave address with default write operation */
	I2C0 ->MSA |=(slaveAddress << 1 ); 
	I2C0 ->MSA &=~(1 << 0 ); 
	I2C0 -> MDR = data ;   
	/*Initiate a single byte transmit of the data from Master to Slave by writing the I2CMCS register
	with a value of 0x0000.0007 (STOP, START, RUN).*/
	I2C0 -> MCS	= 7 ; 
	/* checking busy bit  to check if the transmission is complete*/ 
	while( (I2C0->MCS & (1 << 0)) == 1 ) ;

}
/***************************************************************/
uint8_t i2cSlaveReceive(void)															//is checked this for I2C0
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
uint8_t i2cSlaveReceiveI2C1(void)														//is checked
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
void i2cSendMasterMultiBytesI2C1(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress) //is checked
{
	int counter ;
	/* Initialize the I2C Master by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	I2C1->MCR |=(1 << 4)  ; 
	/* applying master slave address with default write operation */
	I2C1 ->MSA |=(slaveAddress << 1 ); 
	I2C1 ->MSA &=~(1 << 0 ); 
	
	/* send first byte */
	I2C1->MDR = *data;
	/* start and run */
	I2C1 -> MCS	|=((1<<0)|(1<<1)) ; 
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
		I2C1 -> MCS	|= (1<<0) ; 
	}
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while((checkBit(I2C1->MCS,BIT0)) != 0 ) ;
	/* Increment the pointer to send the last byte */
	I2C1 -> MDR = *(data+1);
	/* set both RUN and STOP bits in I2CMCS */
	I2C1->MCS |= ((1<<0) | (1<<2)) ;
	/* checking busy bit in I2CMCS to check if the transmission is complete */
	while( (I2C1->MCS & (1 << 0)) == 1 ) ;
		
}
/***************************************************************/
void gotoSlaveMode(void)																		//is checked
{
	
	/* Initialize the I2C Slave by writing the I2CMCR register with */
	/* a value of 0x0000.0010 */
	/* Enable slave mode by settng SFE bit in I2CMCR */
	I2C1->MCR = 0x20 ;
	/* set slave own address */
	I2C1 ->SOAR = 0x3b ;
	/* enable slave operation */
	setBit(I2C1->SCSR , BIT0 ) ;
}
