
/*************** basic includes and definitions **************/

#include"Basics.h"

/******************* functions definition *******************/

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