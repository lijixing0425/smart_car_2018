#ifndef _STEER_H_
#define _STEER_H_

/***************定义舵机端口************************/
#define S3010_FTM   FTM1                                   //PTA12
#define S3010_CH    FTM_CH0                                //舵机精度 100000
#define S3010_HZ    (80)                                  //80HZ
#define S3010_MID    7770                                  //舵机中值 7790
#define S3010_MAX    8650                                 //zuo限幅 8620
#define S3010_MIN    7020                                 //右限幅 7020

extern void steer_init();
extern void steer_error_get();
extern void steer_control();
extern void steer_out(); 

#endif