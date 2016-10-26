/* This header file contains the funtions of I2C driver for both I2C0 & I2C1 which are : 
 * enable the module, master send single byte, master send multiple byte ,slave recive and 
 * change the state of the controler to slave mode*/

#define TM4C123GH6PM 1
#include "TM4C123.h"
#include "Basics.h"
typedef enum {NO_ERROR,ADDRESS_NOT_ACKNOLEDGED,DATA_NOT_ACKNOLEDGED,ARBITRATION_ERROR}i2cErrors;
/*Prototype*/
typedef enum {STANDARD,FAST,FASTPLUS,HIGHSPEED}i2cState;

void i2c0Enable(i2cState state);
void i2c1Enable(i2cState state);
void i2cSendMasterI2C1(uint8_t data,uint8_t slaveAddress);
void i2cSendMaster(uint8_t data,uint8_t slaveAddress); /* automatic sending of data */
void i2cSendMasterMultiBytesI2C1(uint8_t* data ,uint8_t dataSize , uint8_t slaveAddress); /* send multiple bytes */
uint8_t i2cSlaveReceiveI2C1(void);
uint8_t i2cSlaveReceive(void)	;
void gotoSlaveMode(void);


