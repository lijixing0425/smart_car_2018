#include "common.h"
#include "include.h"
#include "AD.h"

/********************   扫到的电感最大值   *******************/
extern int H[8];
extern int key_deal_flag2;
extern int key_deal_flag3;

// 左 左 右 右 中
int max_100mA[7] = {160, 160, 160 , 160, 160 , 160 , 160}; 
int min_100mA[7] = {  5,   5,   5 ,   5,   5 ,   5 ,   5}; 
int         L[7] = {  5,   5,   5 ,   5,   5 ,   5 ,   5}; 


void ADCDEAL()
{
    int ad_value[8];
    ad_value[0] = 0;   //  左水平
    ad_value[1] = 0;   //  左竖直 1
    ad_value[2] = 0;   //  左竖直 2
    
    ad_value[3] = 0;   //  右水平
    ad_value[4] = 0;   //  右竖直 1
    ad_value[5] = 0;   //  右竖直 2
    
    ad_value[6] = 0;   //  中间
    
    if(key_deal_flag2 == 1)               
    {      
          for(int i = 0;i < 5;i++)             
          {
              ad_valueL0 = adc_once(ADL0, ADC_8bit); 
              ad_value[0] += ad_valueL0;
              
              ad_valueL1 = adc_once(ADL1, ADC_8bit);
              ad_value[1] += ad_valueL1;
              
              ad_valueL2 = adc_once(ADL2, ADC_8bit);
              ad_value[2] += ad_valueL2;
           
              ad_valueR0 = adc_once(ADR0, ADC_8bit);
              ad_value[3] += ad_valueR0;
              
              ad_valueR1 = adc_once(ADR1, ADC_8bit);
              ad_value[4] += ad_valueR1;

              ad_valueR2 = adc_once(ADR2, ADC_8bit);
              ad_value[5] += ad_valueR2;
              
              ad_valueM  = adc_once(ADM, ADC_8bit);
              ad_value[6] += ad_valueM;
          }
          
          ad_value[0] = ad_value[0] / 5;
          ad_value[1] = ad_value[1] / 5;
          ad_value[2] = ad_value[2] / 5;
          ad_value[3] = ad_value[3] / 5;
          ad_value[4] = ad_value[4] / 5;
          ad_value[5] = ad_value[5] / 5;
          ad_value[6] = ad_value[6] / 5;
          
          int convert_ad[7] = {0};
          for(int AD_i = 0;AD_i < 7;AD_i++)
          {
              L[AD_i] = min_100mA[AD_i] * H[AD_i] / 140;
              convert_ad[AD_i] = min_100mA[AD_i] + (ad_value[AD_i] - L[AD_i]) * (max_100mA[AD_i] - min_100mA[AD_i])/ (H[AD_i] + 20 * H[AD_i] / 140 - L[AD_i]);  
          }
          ad_valueL0 = convert_ad[0];
          ad_valueL1 = convert_ad[1];
          ad_valueL2 = convert_ad[2];
          
          ad_valueR0 = convert_ad[3];
          ad_valueR1 = convert_ad[4];
          ad_valueR2 = convert_ad[5];
          
          ad_valueM  = convert_ad[6];    
    }
    
    else if(key_deal_flag2 == 0)            
    { 
          ad_valueL0 = adc_once(ADL0, ADC_8bit);      
          
          ad_value[0]= ad_valueL0;
          
          ad_valueL1 = adc_once(ADL1, ADC_8bit);
          
          ad_value[1]= ad_valueL1;
          
          ad_valueL2 = adc_once(ADL2, ADC_8bit);
          
          ad_value[2]= ad_valueL2;
          
          ad_valueR0 = adc_once(ADR0, ADC_8bit);
          
          ad_value[3]= ad_valueR0;
          
          ad_valueR1 = adc_once(ADR1, ADC_8bit);
          
          ad_value[4]= ad_valueR1;
          
          ad_valueR2 = adc_once(ADR2, ADC_8bit);
          
          ad_value[5]= ad_valueR2;
          
          ad_valueM  = adc_once(ADM, ADC_8bit);
          
          ad_value[6]= ad_valueM;
        
   }
   
    /*****************   归一化   *******************************/
    int sensor_to_oneL0 = (ad_valueL0 - min_v[0]) * 100 / (max_v[0] - min_v[0]);
    int sensor_to_oneL1 = (ad_valueL1 - min_v[1]) * 100 / (max_v[1] - min_v[1]);
    int sensor_to_oneL2 = (ad_valueL2 - min_v[2]) * 100 / (max_v[2] - min_v[2]);

    int sensor_to_oneR0 = (ad_valueR0 - min_v[3]) * 100 / (max_v[3] - min_v[3]);
    int sensor_to_oneR1 = (ad_valueR1 - min_v[4]) * 100 / (max_v[4] - min_v[4]);
    int sensor_to_oneR2 = (ad_valueR2 - min_v[5]) * 100 / (max_v[5] - min_v[5]);
    
    int sensor_to_oneM  = (ad_valueM -  min_v[6]) * 100 / (max_v[6] - min_v[6]);
    
   /******************  限幅   ********************************/
    if(sensor_to_oneL0 <= 0)  sensor_to_oneL0 = 1;
    if(sensor_to_oneL1 <= 0)  sensor_to_oneL1 = 1;
    if(sensor_to_oneL2 <= 0)  sensor_to_oneL2 = 1;
    
    if(sensor_to_oneR0 <= 0)  sensor_to_oneR0 = 1;
    if(sensor_to_oneR1 <= 0)  sensor_to_oneR1 = 1;
    if(sensor_to_oneR2 <= 0)  sensor_to_oneR2 = 1;
    
    if(sensor_to_oneM  <= 0)   sensor_to_oneM  = 1;
    
    if(round_left_flag || round_right_flag)
    {
       if(sensor_to_oneL0 >= 100)  sensor_to_oneL0 = 100;
       if(sensor_to_oneL1 >= 100)  sensor_to_oneL1 = 100;
       if(sensor_to_oneL2 >= 100)  sensor_to_oneL2 = 100;
       
       if(sensor_to_oneR0 >= 100)  sensor_to_oneR0 = 100;
       if(sensor_to_oneR1 >= 100)  sensor_to_oneR1 = 100;
       if(sensor_to_oneR2 >= 100)  sensor_to_oneR2 = 100;
       
       if(sensor_to_oneM >= 100)   sensor_to_oneM = 100;
    }
   
    ADVL0 = sensor_to_oneL0;
    ADVL1 = sensor_to_oneL1;
    ADVL2 = sensor_to_oneL2;
    ADVR0 = sensor_to_oneR0;
    ADVR1 = sensor_to_oneR1;
    ADVR2 = sensor_to_oneR2;
    ADVM  = sensor_to_oneM;
}

uint16 kalman_filter(uint16 inData)
{
    static float last_Data = 0;
    static float p = 10, q = 0.5, r = 0.005, kGain = 0;
   
    p = p + q;
    kGain = p / (p + r);
    
    inData = (uint16)(last_Data + kGain * (inData - last_Data));
    p = (1 - kGain) * p;
   
    last_Data = inData;
    
    return inData;
}