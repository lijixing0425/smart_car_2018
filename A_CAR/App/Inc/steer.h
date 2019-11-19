#ifndef _STEER_H_
#define _STEER_H_

/***************定义舵机端口************************/
#define S3010_FTM   FTM1                                   //PTA12
#define S3010_CH    FTM_CH0                                //舵机精度 10000
#define S3010_HZ    (50)                                  // 
#define S3010_MID    7820                                  //舵机中值
#define S3010_MAX    8740                                  //zuo限幅   
#define S3010_MIN    6900                                  //右限幅    

extern void steer_init();
extern void steer_error_get();
extern void steer_control();
extern void steer_out();

#endif