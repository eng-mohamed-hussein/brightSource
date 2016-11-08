
/*************** basic includes and definitions **************/

#include "headers/Basics.h"

/******************* functions definition *******************/
void toggel_led(uint8_t led)
{
	SYSCTL->RCGCGPIO |= (1 << 5 );
	GPIOF -> DEN |=(1 << led );
	GPIOF ->DIR |= (1 << led );
	toggleBit(GPIOF->DATA,led);
	delay_ms(1000);
	
}
/* Initializing timer0*/

void initDelayTimer(void)
{
	 /* enable timer0  */
		setBit(SYSCTL -> RCGCTIMER,BIT0); 
	 /* clear timer zero control registser  pin TnEN*/
		clearBit(TIMER0 -> CTL,BIT0); 
	 /* cleared to concatenate A,B timer registers */	
		clearAllBits(TIMER0 -> CFG);
	 /* set timer to one-shot mode	 */	 
		setBit(TIMER0 -> TAMR,BIT0);	
	 /* clear TACDIR as count down timer	*/	
		clearBit(TIMER0 -> TAMR,BIT4); 		
	
}
 
void delay_ms(uint32_t time)
{
	 /* setting timer equation to be meassured in MS*/
	  time = time * (SystemCoreClock/1000);
	 /* configure the TIMER0 with the value i want it to count */ 
		TIMER0->TAILR = time;		
		setBit(TIMER0 -> CTL,BIT0);
	 /* checking if TIMER0 finished */
		while(checkBit(TIMER0 -> RIS,BIT0) != 1);
	 /* clear timer finish flag to be able to count back again and that by forcing one */
		setBit(TIMER0 -> ICR,BIT0);
}

void delay_us(uint32_t time)
 {
	 /* setting timer equation to be meassured in microseconds*/
	  time =(uint32_t) time * (SystemCoreClock/1000000);
	 /* configure the TIMER0 with the value i want it to count */ 
		TIMER0->TAILR = time;		
		setBit(TIMER0 -> CTL,BIT0);
	 /* checking if TIMER0 finished */
		while(checkBit(TIMER0 -> RIS,BIT0) != 1);
	 /* clear timer finish flag to be able to count back again and that by forcing one */
		setBit(TIMER0 -> ICR,BIT0);
	 
}
 
void initLeds(void)
{
	
	SYSCTL->RCGCGPIO |= ( 1 << 5 ); // enable portF
	GPIOF->DIR = 0xFF; // make all pins as output 
	GPIOF->DEN = 0xFF ; // configure all portF pins as digital
	GPIOF->DATA = 0x00 ; // init portF with 0  value ( all leds off );

}
void allLedsOff(void)
{
	GPIOF->DATA = 0x00 ;
}

void ledOn(uint8_t b)
{
	allLedsOff();
	GPIOF->DATA |=(1 << b ) ;
}


