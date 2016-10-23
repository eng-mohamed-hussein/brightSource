# include <tm4c123gh6pm.h>
/* This driver using  ADC1 module and  with sample sequencer SS3,*/
/* trigger event  is "always sample" and no.of pin inputs is one  bit PE1*/

void initADC1(void);

/*getting  sample will be without any interrupt  so it polling function*/

/*gets the converted I/P */
/*unsigned int32 returned value */

uint32_t readADC1(void); 