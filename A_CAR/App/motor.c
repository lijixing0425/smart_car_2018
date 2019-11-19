#include "common.h"
#include "include.h"
#include "motor.h"


float left_cs[50] = {0.9986 ,0.9955 ,0.9858 ,0.9738 ,0.9665 ,0.9581 ,0.9492 ,0.9400 ,0.9322 ,0.9256 ,0.9200 ,0.9138 ,0.8955 ,0.8876 ,
                      0.8776 ,0.8682 ,0.8548 ,0.8496 ,0.8394 ,0.8295 ,0.8199 ,0.8101 ,0.8086 ,0.8014 ,0.8001 ,0.7917 ,0.7819 ,0.7679 ,
                      0.7657 ,0.7524 ,0.7460 ,0.7333 ,0.7293 ,0.7203 ,0.7137 ,0.7094 ,0.7013 ,0.6955 ,0.6892 ,0.6794 ,0.6635 ,0.6572 ,0.6450 ,0.6323 ,0.6213 ,0.6100 ,0.6, 0.592,0.581,0.574};    //(MID -> MAX)，50

float right_cs[50] = {0.9986 ,0.9953 ,0.9876 ,0.9746 ,0.9681 ,0.9577 ,0.9497 ,0.9448 ,0.9371 ,0.9307 ,0.9221 ,0.9133 ,0.9033 ,0.8966 ,
                        0.8909 ,0.8848 ,0.8758 ,0.8679 ,0.8617 ,0.8551 ,0.8461 ,0.8371 ,0.8289 ,0.8135 ,0.8075 ,0.8000 ,0.7922 ,0.7894 ,
                        0.7802 ,0.7713 ,0.7660 ,0.7635 ,0.7553 ,0.7482 ,0.7451 ,0.7295 ,0.7248 ,0.6995 ,0.6823 ,0.6761 ,0.6700 ,0.665 ,0.66 ,0.655,0.65 ,0.645,0.64,0.632,0.624,0.618};  //(MID -> MIN)，47



Speed_struct Left_sp,Right_sp;  

int meet_run = 0;
int speed1;
extern int sub_speed;

/********************电机函数**************************/
void motor_init()
{
    ftm_pwm_init(MOTOR_FTM, MOTOR_LEFT1_CH, MOTOR_HZ, 0);  
    ftm_pwm_init(MOTOR_FTM, MOTOR_LEFT2_CH, MOTOR_HZ, 0);   
    
    ftm_pwm_init(MOTOR_FTM, MOTOR_RIGHT1_CH, MOTOR_HZ, 0);  
    ftm_pwm_init(MOTOR_FTM, MOTOR_RIGHT2_CH, MOTOR_HZ, 0); 
}

float cs_find(int x)
{
    if(x >= S3010_MID)
    {
        int a = (x - S3010_MID) / 20;
        return left_cs[a] - (x - a * 20 - S3010_MID) * (left_cs[a] - left_cs[a + 1])/ 20;
    }
    else
    {
        int a = - (x - S3010_MID) / 20;
        return right_cs[a] + (x + a * 20 - S3010_MID) * (right_cs[a] - right_cs[a + 1])/ 20;
    }
}

void speed_set()
{
    if((round_left_flag && !dajiao_left) || (round_right_flag && !dajiao_right)) sub_speed = var12;    
    speed1 = (int)(var11 + FUZZ_SPEED_GET(steer_error,steer_ec) - sub_speed);
    if(ADVL0 < 10  && ADVM < 10 && ADVR0 < 10 && !meet_flag)stop = 1;
    if(yuanhuan_direction != 0 || round_left_flag != 0 || round_right_flag != 0)speed1 = 75;   
    if(podao_flag)speed1 = 65; //大坡减速
    if((steer_duty == S3010_MAX || steer_duty == S3010_MIN) && ((steer_error > 0 && steer_ec > 15) || (steer_error < 0 && steer_ec < -15)) && HD_flag == 0 && meet_flag == 0) speed1 = 55; //侧滑减速

    /******************  会车区  **************************/
    if((this_car == 0 && map_pluse > var0 * 1000) || (this_car == 1 && map_pluse > var1 * 1000))
    {
        speed1 = speed1 > var2 ? var2 : speed1;             //  会车区两侧   最高速度 var2
    }
    if(meet_flag && !meet_run )speed1 = 0;                                                                  //会车  先停车                           
    if(distance1 < 1500 && distance1 > 0 && meet_flag)meet_run = 1;
    
    if(meet_run == 1 && meet_pluse < var8 * 100 && this_car == 1)speed1 =  var3;                //  第一次会车的速度   
    else if(meet_run == 1 && meet_pluse < var8 *100 && this_car == 2)speed1 =  var4;         //  第二次会车的速度  
    
    if(stop)
    {
            Left_sp.Object_pluse  = 0 ;
            Right_sp.Object_pluse = 0 ; 
    }
    else
    { 
            if(steer_duty >= S3010_MID)                                 //左
            {
                float cs = cs_find(steer_duty);
                Left_sp.Object_pluse  = (int)(speed1 * 2 * cs / (1 + cs)); 
                Right_sp.Object_pluse = (int)(speed1 * 2 / (1 + cs));
            }
            else                                                       //右
            {
                float cs = cs_find(steer_duty);
                Left_sp.Object_pluse  = (int)(speed1 * 2 / (1 + cs));
                Right_sp.Object_pluse = (int)(speed1 * 2 * cs / (1 + cs));
            }   
    }

}


void motor_pid()
{
    Left_sp.Motor_P = 28;
    Left_sp.Motor_I = 0.35;
    
    Right_sp.Motor_P = 28;
    Right_sp.Motor_I = 0.35;
    
    Left_sp.acc_speed = Left_sp.Actual_pluse - Left_sp.Last_pluse;            
    Right_sp.acc_speed= Right_sp.Actual_pluse- Right_sp.Last_pluse;             
    
    /**********************************左电机*******************************************************/
    Left_sp.Current_deviate   =  Left_sp.Object_pluse - Left_sp.Actual_pluse;        //当前实际脉冲与目标脉冲的偏差

    if(Left_sp.Actual_pluse > 30 && Left_sp.Object_pluse == 0)
    {
        Left_sp.motor_duty      = -999;
        Left_sp.Pluse_duty      = 0;
        Left_sp.Pluse_increment = 0;
        Left_sp.Current_deviate = 0;
        Left_sp.Last_deviate    = 0;
    }
    else
    {
        if(abs(Left_sp.Current_deviate) > 30)Left_sp.Motor_I = 0;
        else Left_sp.Motor_I = (30 - abs(Left_sp.Current_deviate)) * 0.012;
        
        Left_sp.Pluse_increment   =  (Left_sp.Motor_P * (Left_sp.Current_deviate - Left_sp.Last_deviate) + Left_sp.Motor_I * Left_sp.Current_deviate);
        Left_sp.Last_deviate      =  Left_sp.Current_deviate;                    //记录上次偏差
        Left_sp.Pluse_duty       +=  Left_sp.Pluse_increment;                    //脉冲输出
        Left_sp.Pluse_duty       += 3 * Left_sp.acc_speed;
        Left_sp.motor_duty        =  (int)Left_sp.Pluse_duty;                // pid输出
        if(Left_sp.motor_duty >  999)Left_sp.motor_duty = 999;
        if(Left_sp.motor_duty < -999)Left_sp.motor_duty = -999;
    }

    /***********************************右电机*****************************************************/
    Right_sp.Current_deviate   =  Right_sp.Object_pluse - Right_sp.Actual_pluse;        //当前实际脉冲与目标脉冲的偏差
    
    if(Right_sp.Current_deviate > 30 && Right_sp.Object_pluse == 0)
    {
        Right_sp.Current_deviate = 0;
        Right_sp.Last_deviate    = 0;
        Right_sp.Pluse_increment = 0;
        Right_sp.Pluse_duty      = 0;
        Right_sp.motor_duty      = -999;
    }
    else
    { 
        if(abs(Right_sp.Current_deviate) > 30)Right_sp.Motor_I = 0;
        else    Right_sp.Motor_I = (30 - abs(Right_sp.Current_deviate)) * 0.012;
        Right_sp.Pluse_increment   =  (Right_sp.Motor_P * (Right_sp.Current_deviate - Right_sp.Last_deviate) + Right_sp.Motor_I * Right_sp.Current_deviate);
        Right_sp.Last_deviate      =  Right_sp.Current_deviate;                    //记录上次偏差
        Right_sp.Pluse_duty       +=  Right_sp.Pluse_increment;                    //脉冲输出
        Right_sp.Pluse_duty       +=  3 * Right_sp.acc_speed;
        Right_sp.motor_duty        =  (int)Right_sp.Pluse_duty;
        if(Right_sp.motor_duty >  999)Right_sp.motor_duty = 999;
        if(Right_sp.motor_duty < -999)Right_sp.motor_duty = -999;
    } 
}

void motor_out()
{
    speed_set();
    motor_pid();
    


    if(Left_sp.motor_duty >= 0)
    {
        ftm_pwm_duty( MOTOR_FTM, MOTOR_LEFT1_CH, abs(Left_sp.motor_duty));
        ftm_pwm_duty( MOTOR_FTM, MOTOR_LEFT2_CH, 0); 
    }
    else
    { 
        ftm_pwm_duty( MOTOR_FTM, MOTOR_LEFT1_CH, 0);
        ftm_pwm_duty( MOTOR_FTM, MOTOR_LEFT2_CH, abs(Left_sp.motor_duty)); 
    }
    
    if(Right_sp.motor_duty >= 0)
    {
        ftm_pwm_duty( MOTOR_FTM, MOTOR_RIGHT2_CH, abs(Right_sp.motor_duty)); 
        ftm_pwm_duty( MOTOR_FTM, MOTOR_RIGHT1_CH, 0);
    }
    else 
    {
        ftm_pwm_duty( MOTOR_FTM, MOTOR_RIGHT2_CH, 0); 
        ftm_pwm_duty( MOTOR_FTM, MOTOR_RIGHT1_CH, abs(Right_sp.motor_duty));
    }
}






