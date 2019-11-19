#include "common.h"
#include "include.h"
#include "fuzzy.h"

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

#define steer_kp_PB 13   //右
#define steer_kp_PM 12
#define steer_kp_PS 11
#define steer_kp_ZO 10
#define steer_kp_NS 11
#define steer_kp_NM 12
#define steer_kp_NB 13   //左

#define steer_kd_PB 31   //右
#define steer_kd_PM 27
#define steer_kd_PS 26
#define steer_kd_ZO 24
#define steer_kd_NS 26
#define steer_kd_NM 27
#define steer_kd_NB 31   //左

#define motor_speed_PB 0 //右
#define motor_speed_PM 5
#define motor_speed_PS 9
#define motor_speed_ZO 18
#define motor_speed_NS 9
#define motor_speed_NM 5
#define motor_speed_NB 0 //左

int kp[7][7]= {  
                  {steer_kp_PB,steer_kp_PB,steer_kp_PB,steer_kp_PM,steer_kp_PM,steer_kp_PS,steer_kp_PS}, //0  -70 ~ -46
                  {steer_kp_PB,steer_kp_PB,steer_kp_PM,steer_kp_PM,steer_kp_PS,steer_kp_PS,steer_kp_PS}, //1  -70 ~ -23 
                  {steer_kp_PB,steer_kp_PM,steer_kp_PM,steer_kp_PS,steer_kp_PS,steer_kp_ZO,steer_kp_ZO}, //2  -46 ~ 0 
                  {steer_kp_PB,steer_kp_PM,steer_kp_PS,steer_kp_ZO,steer_kp_NS,steer_kp_NM,steer_kp_NB}, //3  -23 ~ +23                 
                  {steer_kp_ZO,steer_kp_ZO,steer_kp_NS,steer_kp_NS,steer_kp_NM,steer_kp_NM,steer_kp_NB}, //4   0  ~ 46
                  {steer_kp_NS,steer_kp_NS,steer_kp_NS,steer_kp_NM,steer_kp_NM,steer_kp_NB,steer_kp_NB}, //5   23 ~ 70 
                  {steer_kp_NS,steer_kp_NS,steer_kp_NM,steer_kp_NM,steer_kp_NB,steer_kp_NB,steer_kp_NB}
               };

int kd[7][7]= {   
                  {steer_kd_PS,steer_kd_PM,steer_kd_PM,steer_kd_PS,steer_kd_PM,steer_kd_PM,steer_kd_PS},                  
                  {steer_kd_PM,steer_kd_PB,steer_kd_PB,steer_kd_PM,steer_kd_PM,steer_kd_PM,steer_kd_PM},     
                  {steer_kd_PM,steer_kd_PB,steer_kd_PM,steer_kd_PM,steer_kd_PM,steer_kd_PM,steer_kd_PS}, 
                  
                  {steer_kd_PS,steer_kd_PM,steer_kd_PM,steer_kd_PS,steer_kd_PM,steer_kd_PM,steer_kd_PS},
                  
                  {steer_kd_PS,steer_kd_PM,steer_kd_PM,steer_kd_PM,steer_kd_PM,steer_kd_PB,steer_kd_PM},    
                  {steer_kd_PM,steer_kd_PM,steer_kd_PM,steer_kd_PM,steer_kd_PB,steer_kd_PB,steer_kd_PM}, 
                  {steer_kd_PS,steer_kd_PM,steer_kd_PM,steer_kd_PS,steer_kd_PM,steer_kd_PM,steer_kd_PS}  
               }; 


int speed[7][7]={  
                    {motor_speed_PB,motor_speed_PB,motor_speed_PB,motor_speed_PB,motor_speed_PM,motor_speed_PS,motor_speed_PS},//0  -90 ~ -60
                    {motor_speed_PB,motor_speed_PB,motor_speed_PB,motor_speed_PM,motor_speed_PS,motor_speed_PS,motor_speed_ZO},//1  -90 ~ -30 
                    {motor_speed_PB,motor_speed_PM,motor_speed_PM,motor_speed_PS,motor_speed_PS,motor_speed_ZO,motor_speed_ZO},//2  -60 ~ 0 
                    {motor_speed_PB,motor_speed_PM,motor_speed_PS,motor_speed_ZO,motor_speed_NS,motor_speed_NM,motor_speed_NB},//3  -30 ~ +30
                    {motor_speed_ZO,motor_speed_ZO,motor_speed_NS,motor_speed_NS,motor_speed_NM,motor_speed_NM,motor_speed_NB},//4   0  ~ 60
                    {motor_speed_ZO,motor_speed_NS,motor_speed_NS,motor_speed_NM,motor_speed_NB,motor_speed_NB,motor_speed_NB},//5   30 ~ 90    
                    {motor_speed_NS,motor_speed_NS,motor_speed_NM,motor_speed_NB,motor_speed_NB,motor_speed_NB,motor_speed_NB} //6   60 ~ 90
               };


float FTri(float x,float a,float b,float c)
{
    if(x<=a)
        return 0;
    else if((a<x)&&(x<=b))
        return (x-a)/(b-a);
    else if((b<x)&&(x<=c))
        return (c-x)/(c-b);
    else if(x>c)
        return 0;
    else
        return 0;
}

float FTraL(float x,float a,float b)
{
    if(x<=a)
        return 1;
    else if((a<x)&&(x<=b))
        return (b-x)/(b-a);
    else if(x>b)
        return 0;
    else
        return 0;
}

float FTraR(float x,float a,float b)
{
    if(x<=a)
        return 0;
    if((a<x)&&(x<b))
        return (x-a)/(b-a);
    if(x>=b)
        return 1;
    else
        return 1;
}

float fand(float a,float b)
{
    return (a<b)?a:b;
}

float  FUZZP_GET(int a,int b)
{
    float es[7],ecs[7];
    float form[7][7];
    float P1;
    float P2;
    float lsd_max[7] = {0,0,0,0,0,0,0};
    int lsd_maxj[7] = {0,0,0,0,0,0,0};
    if(a > 80)a = 80;
    if(a < -80)a = -80;
    if(b > 15)b = 15;
    if(b < - 15)b = -15;
    
    float e =  3 * a / 40;
    float ec =b * 2 / 5;

    es[NB]=FTraL(e,-6,-3.75);
    es[NM]=FTri(e,-6,-3.75,-2);
    es[NS]=FTri(e,-3.75,-2,0);
    es[ZO]=FTri(e,-2,0,2);
    es[PS]=FTri(e,0,2,3.75);
    es[PM]=FTri(e,2,3.75,6);
    es[PB]=FTraR(e,3.75,6);

    ecs[NB]=FTraL(ec,-6,-4);
    ecs[NM]=FTri(ec,-6,-4,-2);
    ecs[NS]=FTri(ec,-4,-2,0);
    ecs[ZO]=FTri(ec,-2,0,2);
    ecs[PS]=FTri(ec,0,2,4);
    ecs[PM]=FTri(ec,2,4,6);
    ecs[PB]=FTraR(ec,4,6);

    for(int i=0; i<7; i++)
    {
      for(int j=0; j<7; j++)
        {
           form[i][j]=fand(es[i],ecs[j]);
           if(form[i][j]>lsd_max[i])
            {
                lsd_max[i] = form[i][j]; //记录每一行的最大值和所在列
                lsd_maxj[i] = j;
            }
        }
    }
    P2 =  lsd_max[0] + lsd_max[1]+ lsd_max[2]+ lsd_max[3]+ lsd_max[4]+ lsd_max[5] + lsd_max[6];
    P1 =  lsd_max[0] * kp[0][lsd_maxj[0]] + lsd_max[1] * kp[1][lsd_maxj[1]] + lsd_max[2] * kp[2][lsd_maxj[2]] + lsd_max[3] * kp[3][lsd_maxj[3]] + lsd_max[4] * kp[4][lsd_maxj[4]] + lsd_max[5] * kp[5][lsd_maxj[5]] + lsd_max[6] * kp[6][lsd_maxj[6]];
    return P1 / P2;
}

float  FUZZD_GET(int a,int b)
{
    float es[7],ecs[7];
    float form[7][7];
    float D1;
    float D2;
    float lsd_max[7] = {0,0,0,0,0,0,0};
    int lsd_maxj[7] = {0,0,0,0,0,0,0};
    if(a > 80)a = 80;
    if(a < -80)a = -80;
    if(b > 15)b = 15;
    if(b < - 15)b = -15;
    
    float e =  3 * a / 40;
    float ec =b * 2 / 5;

    es[NB]=FTraL(e,-6,-3.75);
    es[NM]=FTri(e,-6,-3.75,-2);
    es[NS]=FTri(e,-3.75,-2,0);
    es[ZO]=FTri(e,-2,0,2);
    es[PS]=FTri(e,0,2,3.75);
    es[PM]=FTri(e,2,3.75,6);
    es[PB]=FTraR(e,3.75,6);

    ecs[NB]=FTraL(ec,-6,-4);
    ecs[NM]=FTri(ec,-6,-4,-2);
    ecs[NS]=FTri(ec,-4,-2,0);
    ecs[ZO]=FTri(ec,-2,0,2);
    ecs[PS]=FTri(ec,0,2,4);
    ecs[PM]=FTri(ec,2,4,6);
    ecs[PB]=FTraR(ec,4,6);

    for(int i=0; i<7; i++)
    {
      for(int j=0; j<7; j++)
        {
           form[i][j]=fand(es[i],ecs[j]);
           if(form[i][j]>lsd_max[i])
            {
                lsd_max[i] = form[i][j]; 
                lsd_maxj[i] = j;
            }
        }
    }
    D2 =  lsd_max[0] + lsd_max[1]+ lsd_max[2]+ lsd_max[3]+ lsd_max[4]+ lsd_max[5] + lsd_max[6];
    D1 =  lsd_max[0] * kd[0][lsd_maxj[0]] + lsd_max[1] * kd[1][lsd_maxj[1]] + lsd_max[2] * kd[2][lsd_maxj[2]] + lsd_max[3] * kd[3][lsd_maxj[3]] + lsd_max[4] * kd[4][lsd_maxj[4]] + lsd_max[5] * kd[5][lsd_maxj[5]] + lsd_max[6] * kd[6][lsd_maxj[6]];
    
    return D1 / D2;
}

float  FUZZ_SPEED_GET(int a,int b)
{
    float es[7],ecs[7];
    float form[7][7];
    float P1;
    float P2;
    float lsd_max[7] = {0,0,0,0,0,0,0};
    int lsd_maxj[7] = {0,0,0,0,0,0,0};
    if(a > 80)a = 80;
    if(a < -80)a = -80;
    if(b > 12)b = 12;
    if(b < - 12)b = -12;
    float e = 3 * a / 40;
    float ec =  b / 2;

    es[NB]=FTraL(e,-6,-4);
    es[NM]=FTri(e,-6,-4,-2);
    es[NS]=FTri(e,-4,-2,0);
    es[ZO]=FTri(e,-2,0,2);
    es[PS]=FTri(e,0,2,4);
    es[PM]=FTri(e,2,4,6);
    es[PB]=FTraR(e,4,6);

    ecs[NB]=FTraL(ec,-6,-4);
    ecs[NM]=FTri(ec,-6,-4,-2);
    ecs[NS]=FTri(ec,-4,-2,0);
    ecs[ZO]=FTri(ec,-2,0,2);
    ecs[PS]=FTri(ec,0,2,4);
    ecs[PM]=FTri(ec,2,4,6);
    ecs[PB]=FTraR(ec,4,6);

    for(int i=0; i<7; i++)
    {
      for(int j=0; j<7; j++)
        {
           form[i][j]=fand(es[i],ecs[j]);
           if(form[i][j]>lsd_max[i])
            {
                lsd_max[i] = form[i][j]; //记录每一行的最大值和所在列
                lsd_maxj[i] = j;
            }
        }
    }
    P2 =  lsd_max[0] + lsd_max[1]+ lsd_max[2]+ lsd_max[3]+ lsd_max[4]+ lsd_max[5] + lsd_max[6];
    P1 =  lsd_max[0] * speed[0][lsd_maxj[0]] + lsd_max[1] * speed[1][lsd_maxj[1]] + lsd_max[2] * speed[2][lsd_maxj[2]] + lsd_max[3] * speed[3][lsd_maxj[3]] + lsd_max[4] * speed[4][lsd_maxj[4]] + lsd_max[5] * speed[5][lsd_maxj[5]] + lsd_max[6] * speed[6][lsd_maxj[6]];
    return (int)(P1 / P2);
}


