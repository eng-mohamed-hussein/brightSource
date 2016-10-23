# include <ADC1.h>

void initADC1(void)
{
	SYSCTL->RCGCADC = (1<<1);       	/*use ADC1 2  16 MHZ*/
	SYSCTL->RCGCGPIO = (1<<4);       	/*set PORTE as GPIO */
	GPIOE->DIR &= ~(1<<1);				/*use pin 1 as I/P  */
	GPIOE->AFSEL = (1<<1);              /*pE1 */
	GPIOE->DEN  &=~(1<<1);              /*disable  digital */
	GPIOE->AMSEL = (1<<1);              /* diable circuts with this  pin */
	
	/*sample sequencer SS3 as it  one bit i/p */
	
	ADC1->ACTSS &= ~(1<<3);     		/*disblae sequncer*/
	ADC1->EMUX =(0xf<<12);   			/*tirger will start as soon as the adc enable  "continous sample " */
	ADC1->SSMUX3 = 2;   				/*analog input */
	ADC1->SSCTL3 = 0x6;  				/*enable intterupt */
	ADC1->ACTSS |= (1<<3); 				/* enable the sequencer */
}


uint32_t readADC(void)
{
	volatile static uint32_t adcResult =0;
	adcResult = ADC1->SSFIFO3;
	return adcResult;
}