#ifndef _AD_H_
#define _AD_H_

/***************¶¨ÒåADC¶Ë¿Ú**************************/
#define ADL0     ADC0_DM1       // ADC0_DM1  
#define ADL1     ADC1_SE4a     //PTE0
#define ADL2     ADC1_SE6a      // PTE2

#define ADR0     ADC0_SE18    //PTE25
#define ADR1     ADC0_SE17    //PTE24
#define ADR2     ADC0_DM0       // ADC0_DM0 

#define ADM      ADC1_SE7a      //PTE3

extern void ADCDEAL();

#endif