#include "common.h"
#include "include.h"
#include "define.h"


int steer_ec;
int steer_e[8];
/***********************����ͷ����**********************************/
uint8 imgbuff[600];                             //����洢����ͼ������飬�൱�ڴ洢��600����λ����������CAMERA_SIZE=600
uint8 img[80*60];                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��60*80=600*8

/*********************    ��������   ***************************/
int yuanhuan_direction;
int round_left_flag;
int round_left_flag1;
int left_max_to_min;
int left_min_to_max;
int out_flag_left1;
int out_flag_left2;
int out_left_str;

int round_right_flag;
int round_right_flag1;
int right_min_to_max;
int right_max_to_min;
int out_flag_right1;
int out_flag_right2;
int out_right_str;
int round_pluse5;

int is_round_pluse;
int round_pluse;            
int round0_pluse;
int out_round_pluse;       
int straight_pluse;      
int podao_pluse;

int HD_flag;
int podao_flag;
int dajiao_left;
int dajiao_right;
int  speed1; 
/***********   �ᳵ����   **********/
int distance1;
int isr_flag;
int meet_area;
int meet_run;
int meet_area_num;
int S_meet_distance;
int meet_flag = 0;
int meet_pluse = 0;
int meet_start = 1;
int spot_flag;

int Max_cha;
int bord[80];
uint32 map_pluse;
/************************Һ������**********************************************/
Site_t site     = {0, 0};                               //��ʾLCDͼ�����Ͻ�λ��
Size_t imgsize  = {CAMERA_W, CAMERA_H};                 //LCDͼ���С��CAMERA_W=80��CAMERA_H=60
Size_t size     ={80,60};                               //LCD��ʾ����ͼ���С

Site_t RIGHT[200];                                       //LCD�ұ߽���ʾ�ṹ��
Site_t angle_line1[82];
Site_t angle_lie[82];


/*****************nrf****************************/
uint8 buff[32];

/***********LCD������ʾ***************************/
Site_t const1   = {0, 60};                            
Site_t const2   = {0, 75};
Site_t const3   = {0, 90};
Site_t const4   = {0, 105};

Site_t const5   = {25, 60};
Site_t const6   = {25, 75};
Site_t const7   = {25, 90};
Site_t const8   = {25, 105};

Site_t const9   = {50, 60};
Site_t const10  = {50, 75};
Site_t const11  = {50, 90};
Site_t const12  = {60, 105};

Site_t const13  = {25, 105};
Site_t const14  = {35, 105};
Site_t const15  = {30, 105};
Site_t const16  = {30, 110};


/*****************ADC����**************************/
uint16 ad_valueL0;
uint16 ad_valueL1;
uint16 ad_valueL2;
uint16 ad_valueR0;
uint16 ad_valueR1;
uint16 ad_valueR2;
uint16 ad_valueM;


int ADVL0;
int ADVL1;
int ADVL2;
int ADVR0;
int ADVR1;
int ADVR2;
int ADVM;


int max_v[7] = {160, 160, 160 , 160,  160, 160 , 160}; 
int min_v[7] = {5,   5,  5 ,  5,   5,  5 ,  5};  

/***************  �������  ***********************/
uint8 lptmr_direction;

/*****************�������*************************/
int compensition_error;
float steer_P;
float steer_D;
int steer_error;
int normal_error;
int normal_error1;
int last_steer_error;
int steer_duty;

/**************  ���ٱ���  *************************/

int stop = 0;

/*************** �������� **********************/
int32 var0 =0, var1 =0;
int32 var2 =0, var3 =0;
int32 var4 =0, var5 =0;
int32 var6 =0, var7 =0;
int32 var8 =0, var9 =0;
int32 var10=0, var11=0;
int32 var12=0, var13=0;
int32 var14=0, var15=0;
int32 var16=0, var17=0;
int32 var18=0, var19=0;
int32 var20=0, var21=0;
int32 var22=0, var23=0;
int32 var24=0, var25=0;
int32 var26=0, var27=0;
int32 var28=0, var29=0;
int32 var30=0, var31=0;
int32 var32=0, var33=0;
int32 var34=0, var35=0;


