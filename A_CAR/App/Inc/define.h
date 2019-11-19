#ifndef _DEFINE_H_
#define _DEFINE_H_

/***********************����ͷ����**********************************/
extern uint8 imgbuff[600];                             //����洢����ͼ������飬�൱�ڴ洢��600����λ����������CAMERA_SIZE=600
extern uint8 img[80*60];                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��60*80=600*8



/*********************    ��������   ***************************/
extern int yuanhuan_direction; 
extern int round_left_flag;
extern int left_min_to_max;
extern int left_max_to_min;
extern int out_flag_left1;
extern int out_flag_left2;
extern int out_left_str;
extern int round_left_flag1;

extern int round_right_flag;
extern int round_right_flag1;
extern int right_min_to_max;
extern int right_max_to_min;
extern int out_flag_right1;
extern int out_flag_right2;
extern int out_right_str;

extern int dajiao_left;
extern int dajiao_right;

extern int is_round_pluse;
extern int podao_pluse;
extern int round_pluse;
extern int round_pluse1;
extern int out_round_pluse;
extern int straight_pluse;
extern int str_Yes_pluse;          //  �����󾭹�һ������ ����ֱ��

extern int HD_OK;
extern int HD_flag;
extern int podao_flag;
/***********  �ᳵ����  *************/

extern uint16 distance1;
extern uint32 map_pluse;

extern int isr_flag;
extern int meet_flag;
extern int meet_start;

extern int  meet_pluse;
extern int  this_car;
extern int spot_flag;

/************************Һ������**********************************************/
extern Site_t site;                               //��ʾLCDͼ�����Ͻ�λ��
extern Size_t imgsize;                 //LCDͼ���С��CAMERA_W=80��CAMERA_H=60
extern Size_t size;                               //LCD��ʾ����ͼ���С

extern Site_t MID[60];                                         //LCD������ʾ�ṹ�壬��Ա1��ʾ�У���Ա2��ʾ��
extern Site_t LEFT[60];                                        //LCD��߽���ʾ�ṹ��
extern Site_t RIGHT[200];                                       //LCD�ұ߽���ʾ�ṹ��
extern Site_t angle_line1[82];
extern Site_t angle_line[82];
extern Site_t angle_line2[82];
extern Site_t angle_line3[82];

extern uint8 buff[32];
extern int NRF_B;

/***********LCD������ʾ***************************/
extern Site_t const1;                            
extern Site_t const2;
extern Site_t const3;
extern Site_t const4;

extern Site_t const5;
extern Site_t const6;
extern Site_t const7;
extern Site_t const8;

extern Site_t const9;
extern Site_t const10;
extern Site_t const11;
extern Site_t const12;

extern Site_t const13;
extern Site_t const14;
extern Site_t const15;
extern Site_t const16;

/*****************ADC����**************************/
extern uint16 ad_valueL0;
extern uint16 ad_valueL1;
extern uint16 ad_valueL2;

extern uint16 ad_valueR0;
extern uint16 ad_valueR1;
extern uint16 ad_valueR2;

extern uint16 ad_valueM;


extern int ADVL0;
extern int ADVL1;
extern int ADVL2;

extern int ADVR0;
extern int ADVR1;
extern int ADVR2;

extern int ADVM;

extern int max_v[7];
extern int min_v[7];


/*****************�������*************************/
extern int Actual_L_pluse;    //ʵ����������
extern int Actual_R_pluse;    //ʵ����������
extern uint8 lptmr_direction;

extern int Object_pluse_left;
extern int Object_pluse_right;

/*****************�������*************************/
extern float steer_P;
extern float steer_D;
extern int steer_error;    //float
extern int last_steer_error;
extern int steer_duty;
extern int last_normal_error;
extern int compensition_error;

extern int fuzzy_error[6];      //  ��¼ǰ����ƫ��
extern int fuzzy_time;
extern int steer_ec;        //  ƫ��仯��


/**************  ���ٱ���  *************************/
extern int stop;

/**************** �������� ***********************/
extern int32 var0, var1;
extern int32 var2, var3;
extern int32 var4, var5;
extern int32 var6, var7;
extern int32 var8, var9;
extern int32 var10, var11;
extern int32 var12, var13;
extern int32 var14, var15;
extern int32 var16, var17;
extern int32 var18, var19;
extern int32 var20, var21;
extern int32 var22, var23;
extern int32 var24, var25;
extern int32 var26, var27;
extern int32 var28, var29;
extern int32 var30, var31;
extern int32 var32, var33;
extern int32 var34, var35;

#endif