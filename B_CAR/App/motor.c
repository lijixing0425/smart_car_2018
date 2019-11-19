#include "common.h"
#include "include.h"
#include "motor.h"


float left_cs[50] = {0.9960 ,0.9936 ,0.9844 ,0.9700 ,0.9604 ,0.9502 ,0.9401 ,0.9315 ,0.9285 ,0.9166 ,0.9078 ,0.8920 ,0.8889 ,0.8755 ,0.8718 ,0.8685 ,0.8585,
                      0.8515 ,0.8458 ,0.8373 ,0.8305 ,0.8224 ,0.8140 ,0.8087 ,0.8044 ,0.7954 ,0.7839 ,0.7757 ,0.7642 ,0.7600 ,0.7382 ,0.7300 ,0.7230 ,0.7149,
                      0.7094 ,0.7044 ,0.6927 ,0.6890 ,0.6654 ,0.6564 ,0.6441 ,0.6345 ,0.6239 ,0.6139 ,0.6080 ,0.5990 ,0.5871};    //(MID -> MAX)，47

float right_cs[50] = {0.9970 ,0.9946 ,0.9850 ,0.9750 ,0.9680 ,0.9556 ,0.9460 ,0.9359 ,0.9270 ,0.9260 ,0.9150 ,0.9070 ,0.8890 ,0.8790 ,0.8720 ,0.8658 ,0.8568 ,
                       0.8480 ,0.8411 ,0.8300 ,0.8286 ,0.8156 ,0.8056 ,0.7973 ,0.7923 ,0.7858 ,0.7808 ,0.7707 ,0.7649 ,0.7555 ,0.7427 ,0.7353 ,0.7256 ,0.7140, 
                       0.6967 ,0.6941 ,0.6929 ,0.6721 ,0.6588 ,0.6495 ,0.6352 ,0.6321 ,0.6173 ,0.6148 ,0.6017 ,0.5871 ,0.5742 };  //(MID -> MIN)，47

Speed_struct Left_sp,Right_sp;  //左右轮  PID
  


extern int sub_speed;
extern int out_speed;
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
  if((round_left_flag && !dajiao_left) || (round_right_flag && !dajiao_right))sub_speed = var12;
  speed1 = var11 + FUZZ_SPEED_GET(steer_error,steer_ec) - sub_speed;
  if(yuanhuan_direction != 0 || round_left_flag != 0 || round_right_flag != 0)speed1 = 75; 
  if(podao_flag)speed1 = 65; //坡减速
  if((steer_duty == S3010_MAX || steer_duty == S3010_MIN) && ((steer_error > 0 && steer_ec > 15) || (steer_error < 0 && steer_ec < -15)) && HD_flag == 0 && meet_flag == 0) speed1 = 55; //侧滑减速
  
   // 会车赛道速度控制 
   if((meet_area_num == 0 && map_pluse > var0 * 1000) || (meet_area_num == 1 && map_pluse > var1 * 1000))
   {
     if(speed1 > var2)speed1 = var2;
   }
   if(meet_flag && meet_pluse <= var8 * 100 && meet_area_num == 1)                             
   {
     if(buff[4] == 0 && meet_run == 0)speed1 = 0;            
     else 
     {       
         gpio_set(PTC14,1);                                          
         speed1 = var3;                            
         meet_run = 1;
      }
   } 
   else if(meet_flag && meet_pluse <= 6000 && meet_area_num == 2)                             
   {
     if(buff[4] == 0 && meet_run == 0)speed1 = 0;            
     else 
     {       
         gpio_set(PTC14,1);                                          
         speed1 = var4;                            
         meet_run = 1;
      }
   }
  if(ADVL0 < 7  && ADVM < 7 && ADVR0 < 7 && meet_flag == 0)stop = 1;   
  //if(speed1 > 87)speed1 = 87;
  if(stop)
  {
          Left_sp.Object_pluse  = 0 ;
          Right_sp.Object_pluse = 0 ; 
  }
  else
  { 
      if(steer_duty >= S3010_MID)                                 //左
      {
          float cs = cs_find(steer_duty) * 1.05; // + 0.002
          if(cs >= 1)cs = 1;
          Left_sp.Object_pluse  = (int)(speed1 * 2 * cs / (1 + cs)); 
          Right_sp.Object_pluse = (int)(speed1 * 2 / (1 + cs));
      }
      else                                                       //右
      {
          float cs = cs_find(steer_duty) * 1.05; //+ 0.003
          if(cs >= 1)cs = 1;
          Left_sp.Object_pluse  = (int)(speed1 * 2 / (1 + cs));
          Right_sp.Object_pluse = (int)(speed1 * 2 * cs / (1 + cs));
      }
  }
}


void motor_pid()
{
    /*****************  结构体PID  ***************/
    Left_sp.Motor_P = 26;
    Left_sp.Motor_I = 0.3;
    
    Right_sp.Motor_P = 26;
    Right_sp.Motor_I = 0.3;
    
    Left_sp.acc_speed = Left_sp.Actual_pluse - Left_sp.Last_pluse;              // 左轮加速度
    Right_sp.acc_speed= Right_sp.Actual_pluse- Right_sp.Last_pluse;             // 右轮加速度
    
    /**********************************左电机*******************************************************/
    Left_sp.Current_deviate   =  Left_sp.Object_pluse - Left_sp.Actual_pluse;        //当前实际脉冲与目标脉冲的偏差
    
    if(Left_sp.Object_pluse == 0 && Left_sp.Actual_pluse > 30)
    {
        Left_sp.Current_deviate = 0;
        Left_sp.Last_deviate    = 0;
        Left_sp.Pluse_increment = 0;
        Left_sp.Pluse_duty      = 0;
        Left_sp.motor_duty      = -999;
    }
    else
    {
        if(abs(Left_sp.Current_deviate) > 30)Left_sp.Motor_I = 0;
        else Left_sp.Motor_I = (30 - abs(Left_sp.Current_deviate)) * 0.011;
        Left_sp.Pluse_increment   =  (Left_sp.Motor_P * (Left_sp.Current_deviate - Left_sp.Last_deviate) + Left_sp.Motor_I * Left_sp.Current_deviate);
        Left_sp.Last_deviate      =  Left_sp.Current_deviate;                    //记录上次偏差
        Left_sp.Pluse_duty       +=  Left_sp.Pluse_increment;                    //脉冲输出
        /*****************  加速度融合   ******************/
        Left_sp.Pluse_duty += 3 * Left_sp.acc_speed;
        Left_sp.motor_duty        =  (int)Left_sp.Pluse_duty;                //占空比输出
   
        if(Left_sp.motor_duty >  999)Left_sp.motor_duty = 999;
        if(Left_sp.motor_duty < -999)Left_sp.motor_duty = -999;
    }
    
    /***********************************右电机*****************************************************/
    Right_sp.Current_deviate   =  Right_sp.Object_pluse - Right_sp.Actual_pluse;        //当前实际脉冲与目标脉冲的偏差
    
    if(Right_sp.Object_pluse == 0 && Right_sp.Actual_pluse > 30)
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
          else Right_sp.Motor_I = (30 - abs(Right_sp.Current_deviate)) * 0.011;
          Right_sp.Pluse_increment   =  (Right_sp.Motor_P * (Right_sp.Current_deviate - Right_sp.Last_deviate) + Right_sp.Motor_I * Right_sp.Current_deviate);
          Right_sp.Last_deviate      =  Right_sp.Current_deviate;                    //记录上次偏差
          Right_sp.Pluse_duty       +=  Right_sp.Pluse_increment;                    //脉冲输出
          /*****************  加速度融合   ******************/
          Right_sp.Pluse_duty       +=  3 * Right_sp.acc_speed;
          
          Right_sp.motor_duty        =  (int)Right_sp.Pluse_duty;                   //占空比输出
  
          if(Right_sp.motor_duty >  999)Right_sp.motor_duty = 999;
          if(Right_sp.motor_duty < -999)Right_sp.motor_duty = -999;
     }
    
}

void motor_out()
{
    speed_set();
    motor_pid();
    
    if((steer_duty == S3010_MAX || steer_duty == S3010_MIN) && abs(steer_ec) > 20)
    {
      Left_sp.motor_duty = 0;
      Right_sp.motor_duty = 0;
    }
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








