#include "common.h"
#include "include.h"
#include "AD.h"

extern int H[8];
extern int key_deal_flag2;
extern int key_deal_flag3;
// 左 左 右 右 中
int max_100mA[7] = {160, 160, 160, 160, 160, 160, 160}; 
int min_100mA[7] = {5, 5, 5, 5, 5, 5, 5}; 
int L[7] = {5, 5, 5, 5, 5, 5, 5}; 

typedef struct{
	int prevData;
	float p,q,r,kGain;
}Kalman;
 
Kalman L0;
Kalman L1;
Kalman L2;
Kalman R0;
Kalman R1;
Kalman R2;
Kalman M;
int one;

void KalmanInit(Kalman *k){
	k->kGain=0;
	k->p=5;	        //p初值可以随便取，但是不能为0（0的话最优滤波器了）
	k->q=0.8;	//q参数调滤波后的曲线平滑程度，q越小越平滑
	k->r=3;	//r参数调整滤波后的曲线与实测曲线的相近程度，越小越接近
	k->prevData=0;
}

int KalmanFilter(Kalman *k,int data){
	k->p=k->p+k->q;
	k->kGain=k->p/(k->p+k->r);
	data=(int)(k->prevData+k->kGain*(data-k->prevData));
	k->p=(1-k->kGain*k->p);
	k->prevData=data;
	return data;
}

void ADCDEAL()
{ 
    int ad_value[7];
    
    if(key_deal_flag2 == 0)
    {
      ad_valueL0 = adc_once(ADL0, ADC_8bit);
      ad_valueL1 = adc_once(ADL1, ADC_8bit);
      ad_valueL2 = adc_once(ADL2, ADC_8bit);
      ad_valueR0 = adc_once(ADR0, ADC_8bit);
      ad_valueR1 = adc_once(ADR1, ADC_8bit);
      ad_valueR2 = adc_once(ADR2, ADC_8bit);
      ad_valueM  = adc_once(ADM, ADC_8bit);
    }
    else if(key_deal_flag2 == 1)                                                      
    {
      ad_valueL0 = 0;
      ad_valueL1 = 0;
      ad_valueL2 = 0;
      ad_valueR0 = 0;
      ad_valueR1 = 0;
      ad_valueR2 = 0;
      ad_valueM  = 0; 
     
      for(int i = 0; i < 5; i ++)
      {
        ad_valueL0 += adc_once(ADL0, ADC_8bit);
        ad_valueL1 += adc_once(ADL1, ADC_8bit);
        ad_valueL2 += adc_once(ADL2, ADC_8bit);
        ad_valueR0 += adc_once(ADR0, ADC_8bit);
        ad_valueR1 += adc_once(ADR1, ADC_8bit);
        ad_valueR2 += adc_once(ADR2, ADC_8bit);
        ad_valueM  += adc_once(ADM, ADC_8bit);
      }
      ad_valueL0 = ad_valueL0 / 5;
      ad_value[0]= ad_valueL0;
      
      ad_valueL1= ad_valueL1 / 5;
      ad_value[1]= ad_valueL1;
      
      ad_valueL2= ad_valueL2 / 5;
      ad_value[2]= ad_valueL2;
      
      ad_valueR0= ad_valueR0 / 5;
      ad_value[3]= ad_valueR0;
      
      ad_valueR1 = ad_valueR1 / 5;
      ad_value[4]= ad_valueR1;
      
      ad_valueR2 = ad_valueR2 / 5;
      ad_value[5]= ad_valueR2;
      
      ad_valueM = ad_valueM / 5;
      ad_value[6]= ad_valueM;
    
      int convert_ad[7] = {0};
      for(int AD_i = 0;AD_i < 7;AD_i++)
      {
          L[AD_i] = min_100mA[AD_i] * H[AD_i] / 140;
          convert_ad[AD_i] = min_100mA[AD_i] + (ad_value[AD_i] - L[AD_i]) * (max_100mA[AD_i] - min_100mA[AD_i])/ (H[AD_i] + 20 * H[AD_i] / 140 - L[AD_i]);  //转换成 100 mA 的电感初始值
      }
      ad_valueL0 = convert_ad[0];
      ad_valueL1 = convert_ad[1];
      ad_valueL2 = convert_ad[2];
      ad_valueR0 = convert_ad[3];
      ad_valueR1 = convert_ad[4]; 
      ad_valueR2 = convert_ad[5];
      ad_valueM  = convert_ad[6];
    }
    /*****************归一化*******************************/
    int sensor_to_oneL0 = (ad_valueL0 - min_v[0]) * 100 / (max_v[0] - min_v[0]);
    int sensor_to_oneL1 = (ad_valueL1 - min_v[1]) * 100 / (max_v[1] - min_v[1]);
    int sensor_to_oneL2 = (ad_valueL2 - min_v[2]) * 100 / (max_v[2] - min_v[2]);
    int sensor_to_oneR0 = (ad_valueR0 - min_v[3]) * 100 / (max_v[3] - min_v[3]);
    int sensor_to_oneR1 = (ad_valueR1 - min_v[4]) * 100 / (max_v[4] - min_v[4]);
    int sensor_to_oneR2 = (ad_valueR2 - min_v[5]) * 100 / (max_v[5] - min_v[5]);
    int sensor_to_oneM  = (ad_valueM -  min_v[6]) * 100 / (max_v[6] - min_v[6]);
   /******************限幅********************************/
    if(round_left_flag || round_right_flag || out_left_str || out_right_str)
    {
      if(sensor_to_oneL0 >= 100)  sensor_to_oneL0 = 100;
      if(sensor_to_oneL1 >= 100)  sensor_to_oneL1 = 100;
      if(sensor_to_oneL2 >= 100)  sensor_to_oneL2 = 100;
      if(sensor_to_oneR0 >= 100)  sensor_to_oneR0 = 100;
      if(sensor_to_oneR1 >= 100)  sensor_to_oneR1 = 100;
      if(sensor_to_oneR2 >= 100)  sensor_to_oneR2 = 100;
      if(sensor_to_oneM >= 100)   sensor_to_oneM = 100;
    }
    if(sensor_to_oneL0 <= 0)  sensor_to_oneL0 = 1;
    if(sensor_to_oneL1 <= 0)  sensor_to_oneL1 = 1;
    if(sensor_to_oneL2 <= 0)  sensor_to_oneL2 = 1;
    if(sensor_to_oneR0 <= 0)  sensor_to_oneR0 = 1;
    if(sensor_to_oneR1 <= 0)  sensor_to_oneR1 = 1;
    if(sensor_to_oneR2 <= 0)  sensor_to_oneR2 = 1;
    if(sensor_to_oneM  <= 0)  sensor_to_oneM  = 1;
   
    ADVL0 = sensor_to_oneL0;
    ADVL1 = sensor_to_oneL1;
    ADVL2 = sensor_to_oneL2;
    ADVR0 = sensor_to_oneR0;
    ADVR1 = sensor_to_oneR1;
    ADVR2 = sensor_to_oneR2;
    ADVM  = sensor_to_oneM;
}



