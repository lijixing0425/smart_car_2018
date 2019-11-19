#ifndef _AD_H_
#define _AD_H_

/***************定义ADC端口**************************/
#define ADL0     ADC1_SE5a    // PTE1  
#define ADL1     ADC1_SE4a     //PTE0
#define ADL2     ADC1_SE6a     // PTE2      第二个竖直电感

#define ADR0     ADC0_SE18    //PTE25
#define ADR1     ADC0_SE17    //PTE24
#define ADR2     ADC0_DM0     //ADC0_DM0   第二个竖直电感   

#define ADM      ADC1_SE7a      //PTE3

extern void ADCDEAL();
extern uint16 kalman_filter(uint16 inData);

#endif