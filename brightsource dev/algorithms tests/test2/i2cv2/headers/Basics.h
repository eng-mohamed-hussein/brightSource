/* this library designed to offer the most basic functions and 
 * code essentials like delay function and pins definitions  */

/*************** basic includes and definitions **************/

#ifndef TM4C123GH6PM 
	#define TM4C123GH6PM processor
#endif

#define PIN0  0
#define PIN1  1
#define PIN2  2
#define PIN3  3
#define PIN4  4
#define PIN5  5
#define PIN6  6
#define PIN7  7

#define BIT0  0
#define BIT1  1
#define BIT2  2
#define BIT3  3
#define BIT4  4
#define BIT5  5
#define BIT6  6
#define BIT7  7
#define BIT8  8
#define BIT9  9
#define BIT10 10
#define BIT11 11
#define BIT12 12
#define BIT13 13
#define BIT14 14
#define BIT15 15

/* Bit Manipulations */
#define setBit(REG,PIN)					(REG |=  (1<<PIN))
#define toggleBit(REG,PIN)			(REG ^=  (1<<PIN))
#define clearBit(REG,PIN)				(REG &= ~(1<<PIN))
#define checkBit(REG,PIN)				(REG &   (1<<PIN))

/* Register Manipulations */
#define setAllBits(REG)					(REG |=  (0xFFFFFFFF))
#define toggleAllBits(REG)			(REG ^=  (0xFFFFFFFF))
#define clearAllBits(REG)				(REG &= ~(0xFFFFFFFF))
	
#include "TM4C123.h" 

/******************* functions prototypes *******************/

/*Initializing Timer0
 *BE AWARE not to using timer0 while using delay_ms() function 
 */
void initDelayTimer(void);

/*This function take the required time to be delayed in ms*/
void delay_ms(uint32_t time);
