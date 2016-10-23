# include <tm4c123gh6pm.h>
/* This driver using  ADC1 module and  with sample sequencer SS3,*/
/* trigger event  is "always sample" and no.of pin inputs is one  bit PE1*/

void Initial_ADC1(void);

/*getting  sample will be without any interrupt  so it polling function*/


uint32_t Get_ADC(void);
