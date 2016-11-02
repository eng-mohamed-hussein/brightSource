#include "timeouttimer.h"

void initTimeOutTimer(void)
{
	 /* enable TIMER1  */
		setBit(SYSCTL -> RCGCTIMER,BIT1); 
	 /* clear timer one control registser  pin TnEN*/
		clearBit(TIMER1 -> CTL,BIT0); 
	 /* cleared to concatenate A,B timer registers */	
		clearAllBits(TIMER1 -> CFG);
	 /* set timer to one-shot mode*/	 
		setBit(TIMER1 -> TAMR,BIT0);	
	 /* clear TACDIR as count down timer	*/	
		clearBit(TIMER1 -> TAMR,BIT4); 		
	
}
 
void timeOut_ms(uint32_t time)
{
	 /* setting timer equation to be meassured in MS*/
	  time = time * (SystemCoreClock/1000);
	 /* configure the TIMER1 with the value i want it to count */ 
		TIMER1->TAILR = time;	
		/* enable interrupt */
		setBit(TIMER1 ->IMR , BIT0 );
	/* enable general interrupt in interrupt table */
		setBit(NVIC->ISER[0],21);
		setBit(TIMER1 -> CTL,BIT0);
	 /* checking if TIMER1 finished */
		//while(checkBit(TIMER1 -> RIS,BIT0) != 1);
	 /* clear timer finish flag to be able to count back again and that by forcing one */
		//setBit(TIMER1 -> ICR,BIT0);
}