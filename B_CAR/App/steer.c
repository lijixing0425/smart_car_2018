#include "common.h"
#include "include.h"
#include "steer.h"

int sub_speed;

int buchang_left;
int buchang_right;
int last_normal_error;
int ADVM1[4];
int pluse;

int left_value;
int left_min;
int right_value;
int right_min;
int abnormall_left; 
int abnormall_right;
int back_double_circle;
void steer_init()
{
  ftm_pwm_init(S3010_FTM ,S3010_CH, S3010_HZ, S3010_MID);
}

void steer_error_get()
{              
                //横向偏差获取
               if(ADVL0 > 7 || ADVR0 > 7 || ADVM > 7)last_normal_error = normal_error;
               int max_ad;
               if(ADVL0 > ADVM && ADVL0 > ADVR0)max_ad = 0;
               else if(ADVR0 > ADVM && ADVR0 > ADVL0)max_ad = 2;
               else max_ad = 1;
               int apart_value = 50;
               steer_error = 0;
               if(max_ad == 0)
               {
                   if(ADVM < apart_value)steer_error =  80 * (ADVL0 - ADVM) / (ADVM + ADVL0) + 100 * (ADVL0 -ADVR0 + apart_value - ADVM) / (ADVL0 + ADVR0);
                   else steer_error = 100 * (ADVL0 -ADVR0) / (ADVL0 + ADVR0) + 80 * (ADVL0 - ADVM) / (ADVM + ADVL0);
               }
               else if(max_ad == 1)
               {
                   if(ADVM < apart_value)
                   {
                     if(ADVL0 > ADVR0)steer_error = 100 * (ADVL0 -ADVR0 + apart_value - ADVM) / (ADVL0 + ADVR0);
                     else steer_error = 100 * (ADVL0 -ADVR0 + ADVM - apart_value) / (ADVL0 + ADVR0);
                   }
                   else steer_error = 100 * (ADVL0 -ADVR0) / (ADVL0 + ADVR0);
               }
               else
               {
                   if(ADVM < apart_value)steer_error =  80 * (ADVM - ADVR0) / (ADVM + ADVR0) + 100 * (ADVL0 -ADVR0 + ADVM - apart_value) / (ADVL0 + ADVR0);
                   else steer_error = 100 * (ADVL0 -ADVR0) / (ADVL0 + ADVR0) + 80 * (ADVM - ADVR0) / (ADVM + ADVR0);
               }
               normal_error1 = steer_error;    
               if(ADVL0 < 7 && ADVM < 7 && ADVR0 < 7)steer_error = last_normal_error;
               
               if(ADVM > 110 && HD_flag == 0 && ADVL0 + ADVR0 < 120)podao_flag = 1; 
               if((ADVM >= 105 || ADVR0 >= 105  || ADVL0 >= 105 || (ADVL0 > 100 && ADVM > 100) || (ADVR0 > 100 && ADVM > 100)) \
                 && round_left_flag == 0  && round_right_flag== 0 && out_flag_left1 == 0 && out_flag_right1 == 0  && out_flag_left2 == 0 \
                 && out_flag_right2 == 0 && out_left_str == 0 && out_right_str == 0 && isr_flag == 0 && yuanhuan_direction  == 0)     
                {
                      int maxy = ADVL1 > ADVR1 ? ADVL1 : ADVR1;
                      if(ADVL0 >= 105)
                      {
                          if((ADVL1 > 25 && ADVR1 > 40 && ADVL1 < ADVR1) || (ADVL1 > 70 && ADVR1 > 70 && ADVR0 < 60)) yuanhuan_direction  = 2;
                          else  yuanhuan_direction  = 1;
                      }
                      else if(ADVR0 >= 105)
                      {
                          if(ADVL1 > 40 && ADVR1 > 25 && ADVL1 > ADVR1 || (ADVL1 > 70 && ADVR1 > 70 && ADVL0 < 60))yuanhuan_direction  = 1;
                          else yuanhuan_direction  = 2;
                      }
                      if(ADVL0 > 100 && ADVM > 100) 
                      {
                          if((ADVL1 > 25 && ADVR1 > 40 && ADVL1 < ADVR1) || (ADVL1 > 70 && ADVR1 > 70 && ADVR0 < 60))yuanhuan_direction  = 2;
                          else  yuanhuan_direction  = 1;
                      }
                      else if(ADVR0 > 100 && ADVM > 100) 
                      {
                          if(ADVL1 > 40 && ADVR1 > 25 && ADVL1 > ADVR1 || (ADVL1 > 70 && ADVR1 > 70 && ADVL0 < 60))yuanhuan_direction  = 1;
                          else yuanhuan_direction  = 2;
                      }
                      else if(ADVM >= 105)
                      {
                          if(ADVL1 > ADVR1 && maxy > 25 && ADVL0 > 65) yuanhuan_direction  = 1;
                          else if(ADVL1 < ADVR1 && maxy > 25 && ADVR0 > 65)yuanhuan_direction  = 2;
                      }
               }
               if(((ADVL0 > var15 && ADVM > 110 && ADVR0 > var16) || (ADVR0 > var15 && ADVM > 100 && ADVL0 > var16)) \
                 && round_left_flag == 0  && round_right_flag== 0 && out_flag_left1 == 0 && out_flag_right1 == 0  && out_flag_left2 == 0 \
                 && out_flag_right2 == 0 && out_left_str == 0 && out_right_str == 0 && isr_flag == 0 && yuanhuan_direction != 0)     
                {
                    if(ADVL1 > 50 && ADVR1 > 50 && ADVM >= 100 && ADVL0 < 90 && ADVR0 < 90)//防止十字误判环
                    {
                      yuanhuan_direction = 0;
                      round_left_flag = 1;
                      round_right_flag = 1;
                    }
                    else 
                    {
                        if(yuanhuan_direction == 1)
                        {
                          round_left_flag = 1;
                          yuanhuan_direction = 0;
                        }
                        if(yuanhuan_direction == 2)
                        {
                          round_right_flag = 1;
                          yuanhuan_direction = 0;
                        }
                    }
                    if(round_left_flag|| round_right_flag)
                    {
                          HD_flag = 1;
                    }
               }
               if(podao_flag == 1)
               {
                 yuanhuan_direction = 0;
                 round_left_flag = 0;
                 round_right_flag = 0;
               }
               if((round_left_flag == 1 || round_right_flag == 1) && round0_pluse > 7000)  
               {
                    round_left_flag = 0;
                    left_min_to_max = 0;
                    left_max_to_min = 0;
                    round_left_flag1= 0;
                    round0_pluse    = 0;
                    
                    round_right_flag = 0;
                    right_min_to_max = 0;
                    right_max_to_min = 0;
                    round_right_flag1= 0;
               } 

               int k = 95 - ADVM;
               if(k < 0) k = 0;
               if(k > 8) k = 8;
            
               if(ADVL1 > ADVR1)compensition_error = ADVL1 - ADVR1 + ADVL2 - ADVR2 + (abs(ADVL1 - ADVL2) + ADVR2 - ADVR1) / 2;
               else compensition_error = ADVL1 - ADVR1 + ADVL2 - ADVR2 + (ADVL1 - ADVL2 - abs(ADVR2 - ADVR1)) / 2;

               if(out_flag_left1 || out_flag_right1)
               {
               }
               else 
               {
                 if(ADVM > ADVM1[3] || ADVM >= 95)compensition_error = compensition_error * k / 8;
               }
               
               if(abs(steer_error) > abs(compensition_error))steer_error = (compensition_error  + steer_error * 9) / 10;
               
               if(yuanhuan_direction == 0 && round_left_flag == 0 && round_right_flag == 0 && out_flag_right2 == 0 && out_flag_left2 == 0 && out_left_str == 0 && out_right_str == 0 && abs(steer_error) < abs(compensition_error) && steer_error * compensition_error >= 0)
               {
                          int miny = ADVL1 < ADVR1 ? ADVL1 : ADVR1;
                          int m,n;
                          if(abs(steer_error) > 10)m = 10;
                          else m = abs(steer_error);
                          if(miny > 10)n = 10;
                          else n = miny;
                          
                          if(abs(steer_error) < 10 && abs(steer_error) * 2 < abs(compensition_error) && ADVM > 87 && (out_flag_left1 ==0 && out_flag_right1 ==0))
                          {
                          }
                          else  steer_error = ((m + n) * compensition_error + (20 - m - n) * steer_error) / 20;
               }
               
               normal_error = steer_error;        
               if(round_left_flag)
               {
                     if(ADVL1 > 10 && left_min_to_max == 0) left_min_to_max = 1;
                     if(left_min_to_max && ADVL1 < 10 && left_max_to_min == 0)left_max_to_min = 1;
                     if(left_max_to_min  && ADVL1 + 5 > ADVR1 && ADVL1 > 10  && round_left_flag1 == 0)
                     {
                       round_left_flag1 = 1;                                               
                       if(round0_pluse > 7000)                    
                       {
                         round_left_flag = 0;
                         round_left_flag1 = 0;
                         left_min_to_max = 0;
                         left_max_to_min = 0;
                         dajiao_left = 0;
                         round0_pluse = 0;
                       }
                       else
                       {
                         dajiao_left = 1; 
                       }
                     }
                     if(dajiao_left == 1)
                     {
                       steer_error = 12 + ADVL1 - ADVR1 + ADVL2 - ADVR2 + (abs(ADVL1 - ADVL2) + ADVR2 - ADVR1) / 2;
                       if(steer_error < 25)steer_error = 25;
                     } 
                     if((normal_error  > steer_error && dajiao_left && normal_error != steer_error && ADVM < 70) || round_pluse > 4400)            
                     {
                        round_left_flag1 = 0;                                              
                        round_left_flag = 0;
                        left_min_to_max = 0;
                        left_max_to_min = 0;   
                        dajiao_left = 0;
                        round0_pluse = 0;
                        out_flag_left1  = 1;
                        sub_speed = 0;
                     }
               }
               else if(round_right_flag)
               {
                     if(ADVR1 > 10 && right_min_to_max == 0) right_min_to_max = 1;
                     if(right_min_to_max && ADVR1 < 10 && right_max_to_min == 0)right_max_to_min = 1;
                     if(right_max_to_min && ADVR1 + 5 > ADVL1 && ADVR1 > 15 && round_right_flag1 == 0)
                     {
                       round_right_flag1 = 1;
                       if(round0_pluse > 8000)
                       { 
                         round_right_flag = 0;
                         round_right_flag1 = 0;
                         right_min_to_max = 0;
                         right_max_to_min = 0;
                         round0_pluse =0;
                         dajiao_right = 0;
                       }
                       else 
                       {
                         dajiao_right = 1;
                       }
                     }
                     if(dajiao_right == 1)
                     {
                       steer_error = -4 + ADVL1 - ADVR1 + ADVL2 - ADVR2 + (ADVL1 - ADVL2 - abs(ADVR2 - ADVR1)) / 2;
                     }  
                     if((normal_error  < steer_error && dajiao_right && normal_error != steer_error && ADVM < 70) || round_pluse > 4400)            
                     {
                        round_right_flag = 0;
                        right_min_to_max = 0;
                        right_max_to_min = 0;
                        round_right_flag1 = 0;
                        dajiao_right = 0;
                        round0_pluse = 0;
                        out_flag_right1  = 1;
                        sub_speed = 0;
                     }
               }
               else
               {  
                 if(out_flag_left1 && ADVL1 > 60 && ADVR1 > 60 && !out_flag_left2)          
                 {                                                             
                        straight_pluse = 300 + round_pluse / 7 ; 
                        if(straight_pluse < 2300)straight_pluse = 2300;
                        out_flag_left2 = 1;
                        out_flag_left1 = 0;    
                        if(round_pluse > 50000)                               
                        {
                          out_flag_left2 = 0;
                          round_pluse = 0;
                          straight_pluse = 0;
                        }
                        pluse = 3300 + (3800 - 3300) * (round_pluse - 14000)/(31000 - 14000);
                        left_min = 50 - (50 - 32) * (round_pluse - 14000)/(31000 - 14000);
                 } 
                 if(out_flag_left1)steer_error = steer_error > 25 ? steer_error : 25;
                 if(out_flag_left2)
                 {      
                        steer_error = abs(ADVL1 - ADVR1) + ADVL2 - ADVR2 + (abs(ADVL1 - ADVL2) + ADVR2 - ADVR1) / 2 > steer_error ? abs(ADVL1 - ADVR1) + ADVL2 - ADVR2 + (abs(ADVL1 - ADVL2) + ADVR2 - ADVR1) / 2  : steer_error;
                        steer_error += 6;
                        if(steer_error > 65)steer_error = 65;
                        if(steer_error < left_min)steer_error = left_min;
                        sub_speed =  5 - 5 * (straight_pluse - 2300)/(4700 - 2300);
                 }     
                 if(round_pluse5 > pluse && out_flag_left2 && out_left_str == 0)  
                 {
                        out_left_str = 1;
                        out_flag_left2 = 0;  
                        round_pluse = 0;
                        buchang_left = 0;
                        round_pluse5 = 0;
                        abnormall_left = 0;
                        back_double_circle = 0;
                        pluse = 0;
                 } 
                 if(out_left_str == 1 && out_round_pluse < straight_pluse && out_flag_left2 == 0)   
                 {
                     if(steer_error > 0)
                     {
                       steer_error = -3 + (normal_error1 - abs(ADVL1 - ADVR1)  / 3) / 5;
                     }
                     else steer_error = normal_error1;
                 }             
                 if(out_left_str == 1 && out_round_pluse > straight_pluse)
                 {
                       straight_pluse = 0;
                       out_round_pluse= 0;
                       out_left_str   = 0;
                       sub_speed = 0;
                       HD_flag = 0;       
                 }
                 if(out_flag_right1)steer_error = steer_error < -25 ? steer_error : -25;
                 if(out_flag_right1 && ADVL1 > 60 && ADVR1 > 60 && !out_flag_right2)        
                 {
                         out_flag_right2 = 1;
                         out_flag_right1 = 0;
                         straight_pluse = 300 + round_pluse / 7;
                         if(straight_pluse < 2300)straight_pluse = 2300;
                         if(round_pluse > 50000)                                 
                         {
                           out_flag_right2 = 0;
                           round_pluse = 0;
                           straight_pluse = 0;
                         }
                         pluse = 3300 + (3800 - 3300) * (round_pluse - 14000)/(31000 - 14000);
                         right_min = 50 - (50 - 32) * (round_pluse - 14000)/(31000 - 14000);
                 } 
                 if(out_flag_right2)
                 {
                         steer_error = -abs(ADVL1 - ADVR1) + ADVL2 - ADVR2 + (ADVL1 - ADVL2 - abs(ADVR2 - ADVR1)) / 2 < steer_error ? -abs(ADVL1 - ADVR1) + ADVL2 - ADVR2 + (ADVL1 - ADVL2 - abs(ADVR2 - ADVR1)) / 2 : steer_error;
                         if(steer_error < -65)steer_error = -65;
                         if(steer_error > -right_min)steer_error = -right_min;
                         sub_speed =  5 - 5 * (straight_pluse - 2300)/(4700 - 2300);
                 } 
                 if(round_pluse5 > pluse && out_flag_right2 && out_right_str == 0 )
                 {
                         out_right_str = 1;
                         out_flag_right2 = 0;
                         round_pluse = 0;
                         buchang_right = 0;
                         round_pluse5 = 0;
                         abnormall_right = 0;
                         back_double_circle = 0;
                         pluse = 0;
                 }
                 if(out_right_str == 1 && out_round_pluse < straight_pluse && out_flag_right2 == 0)
                 {
                     if(steer_error < 0)
                     {
                       steer_error = 3 + (normal_error1 + abs(ADVL1 - ADVR1) / 3) / 5;
                     }
                     else steer_error = normal_error1;
                 }
                 else if(out_right_str == 1 && out_round_pluse > straight_pluse)
                 {
                       straight_pluse  = 0;
                       out_right_str   = 0;
                       out_round_pluse = 0;
                       HD_flag = 0;
                       sub_speed = 0;
                 }
               }
               if(dajiao_left || out_flag_left1)
               {
                 if((ADVM < ADVL0 && ADVM < ADVR0) || (ADVL0 < 25 && ADVR0 < 25))steer_error = 80;
               }
               if(dajiao_right || out_flag_right1)
               {
                 if((ADVM < ADVL0 && ADVM < ADVR0) || (ADVL0 < 25 && ADVR0 < 25))steer_error = -80;
               }
               for(int i = 3; i > 0; i --)
               {
                 steer_e[i] = steer_e[i - 1];
               }
               steer_e[0] = steer_error;
               steer_ec = steer_error - steer_e[3];
               for(int i = 3; i > 0; i --)
               {
                 ADVM1[i] = ADVM1[i - 1];
               }
               ADVM1[0] = ADVM;
}   

void steer_control()
{  
       steer_P = FUZZP_GET(steer_error,steer_ec);        //模糊PD
       steer_D = FUZZD_GET(steer_error,steer_ec) + 1;
       
       /*int max_shu;
       max_shu = ADVL1 > ADVL2 ? ADVL1 : ADVL2;
       max_shu = max_shu > ADVR2 ? max_shu : ADVR2; 
       max_shu = max_shu > ADVR1 ? max_shu : ADVR1;*/
       if(ADVL1 == 1 && ADVR1 == 1 && ADVL2 <= 2 && ADVR2 <= 2)steer_P = steer_P * 0.9;
      // else if(ADVL2 >= ADVL1 && ADVL2 > ADVR2 && !HD_flag && max_shu > 4)steer_P = steer_P + (ADVL2 - ADVL1) * 0.2; //根据赛道微调PD
       //else if(ADVR2 > ADVL2 && ADVR2 >= ADVR1 && !HD_flag && max_shu > 4)steer_P = steer_P + (ADVR2 - ADVR1) * 0.2;   
         
      /* if(abs(steer_error) < 25)
       {                   
          steer_D = 26;                 
       }
       else if(abs(steer_error) < 50)
       {                 
          steer_D = 30;                
       }
       else
       {                 
          steer_D = 25;                
       }*/
      steer_duty = (int)(S3010_MID + steer_P * steer_error + steer_D * steer_ec);
      last_steer_error = steer_error;
      if(steer_duty> S3010_MAX) steer_duty = S3010_MAX;
      if(steer_duty < S3010_MIN)steer_duty = S3010_MIN;
}

void steer_out()
{
  ftm_pwm_duty(S3010_FTM, S3010_CH, steer_duty);
} 