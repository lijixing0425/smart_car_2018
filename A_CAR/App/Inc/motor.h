#ifndef _MOTOR_H_
#define _MOTOR_H_

/***************�������˿�***********************/
#define MOTOR_FTM   FTM0                                      //���ͨ��
#define MOTOR_LEFT1_CH   FTM_CH5                              //PTD5
#define MOTOR_LEFT2_CH   FTM_CH6                              //PTD6
#define MOTOR_RIGHT1_CH   FTM_CH3                             //PTA6
#define MOTOR_RIGHT2_CH   FTM_CH4                             //PTA7
#define MOTOR_HZ    14 * 1000                                 //���Ƶ������Ϊ20000hz������Ϊ1000

extern void motor_init();
extern void speed_set();
extern void motor_pid();
extern void motor_out();

/*****************  ����  PID �ṹ��   *****************/
typedef struct
{
    float Motor_P;                      //  ��� P
    float Motor_I;                      //  ��� I
    
    float Pluse_increment;              //�������
    float Pluse_duty;                   //�������
    
    float acc_speed;
    int motor_duty;                     //ռ�ձ����
    
    int Current_deviate;                //��ǰ���
    int Last_deviate;                   //�ϴ����
    
    int Object_pluse;
    int Actual_pluse;
    int Last_pluse;
} Speed_struct;            //������Ϣ



#endif