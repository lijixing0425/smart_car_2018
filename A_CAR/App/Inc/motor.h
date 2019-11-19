#ifndef _MOTOR_H_
#define _MOTOR_H_

/***************定义电机端口***********************/
#define MOTOR_FTM   FTM0                                      //电机通道
#define MOTOR_LEFT1_CH   FTM_CH5                              //PTD5
#define MOTOR_LEFT2_CH   FTM_CH6                              //PTD6
#define MOTOR_RIGHT1_CH   FTM_CH3                             //PTA6
#define MOTOR_RIGHT2_CH   FTM_CH4                             //PTA7
#define MOTOR_HZ    14 * 1000                                 //电机频率设置为20000hz，精度为1000

extern void motor_init();
extern void speed_set();
extern void motor_pid();
extern void motor_out();

/*****************  定义  PID 结构体   *****************/
typedef struct
{
    float Motor_P;                      //  电机 P
    float Motor_I;                      //  电机 I
    
    float Pluse_increment;              //输出增量
    float Pluse_duty;                   //最终输出
    
    float acc_speed;
    int motor_duty;                     //占空比输出
    
    int Current_deviate;                //当前误差
    int Last_deviate;                   //上次误差
    
    int Object_pluse;
    int Actual_pluse;
    int Last_pluse;
} Speed_struct;            //变量信息



#endif