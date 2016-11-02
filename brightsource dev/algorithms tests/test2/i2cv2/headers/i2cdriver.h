/* This header file contains the funtions of I2C driver for both I2C0 & I2C1 which are : 
 * enable the module, master send single byte, master send multiple byte ,slave recive and 
 * change the state of the controler to slave mode*/
#ifndef _I2CDRIVER_
#define _I2CDRIVER_

#include <tm4c123gh6pm.h>
#include "Basics.h"

typedef enum {NO_ERROR,ADDRESS_NOT_ACKNOLEDGED,DATA_NOT_ACKNOLEDGED,ARBITRATION_ERROR}i2cErrors;
/*Prototype*/
typedef enum {STANDARD,FAST,FASTPLUS,HIGHSPEED}i2cState;

void i2c0Enable(i2cState state);
void i2c1Enable(i2cState state);
void i2c2Enable(i2cState state);
void i2c3Enable(i2cState state);

/* send single byte */
void i2c0SendMaster(uint8_t data,uint8_t slaveAddress); 
void i2c1SendMaster(uint8_t data,uint8_t slaveAddress);
void i2c2SendMaster(uint8_t data,uint8_t slaveAddress); 
void i2c3SendMaster(uint8_t data,uint8_t slaveAddress);

/* send multiple bytes */
void i2c0SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress); 
void i2c1SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress); 
void i2c2SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress); 
void i2c3SendMasterMultiBytes(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress); 

uint8_t i2c0SlaveReceive(void);
uint8_t i2c1SlaveReceive(void);
uint8_t i2c2SlaveReceive(void);
uint8_t i2c3SlaveReceive(void);

void i2c0GotoSlaveMode(void);
void i2c1GotoSlaveMode(void);
void i2c2GotoSlaveMode(void);
void i2c3GotoSlaveMode(void);

#endif

