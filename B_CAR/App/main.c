#include "common.h"
#include "include.h"

extern ui_var_info_t  num_info[VAR_MAX];
extern void var_display(uint8 tab);
extern Speed_struct Left_sp,Right_sp;

int key_deal_flag1;
int key_deal_flag2;
int key_deal_flag3;
int key_deal_flag4;
uint16 car_stop;
int meet_distance;
int HD_OK;
/*************  归一化 FLASH *****************/
int H[8] = {100,100,100,100,100,100,100};
int AD_value[8];

void flash_load()
{
    flash_init();
    DELAY_MS(100);
    for(int num=0;num <= VAR_MAX;num++)
    {
        num_info[num].val = flash_read(FLASH_SECTOR_NUM-1, num*4, int32);
        var_display(num);
    }
    var0 = num_info[0].val;
    var1 = num_info[1].val;
    var2 = num_info[2].val;
    var3 = num_info[3].val;
    var4 = num_info[4].val;
    var5 = num_info[5].val;
    var6 = num_info[6].val;
    var7 = num_info[7].val;
    var8 = num_info[8].val;
    var9 = num_info[9].val;
    var10 = num_info[10].val;
    var11 = num_info[11].val;
    var12 = num_info[12].val;
    var13 = num_info[13].val;
    var14 = num_info[14].val;
    var15 = num_info[15].val;
    var16 = num_info[16].val;
    var17 = num_info[17].val;
    
    if(key_deal_flag2 == 1)                        
    {
        for(int num=0;num < 7;num++)
        {
            H[num] = flash_read(FLASH_SECTOR_NUM-3, num*4, uint8);
        }
    }
    else if(key_deal_flag2 == 0)
    {
        for(int i = 0;i < 7;i++)
        {
              H[i] = 0;
        }
    }
}

void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}


void PORTE_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //清中断标志位

    n = 27;
    if(flag & (1 << n))                                 //PTE27触发中断
    {
        nrf_handler();
    }
}

void DMA0_IRQHandler()
{
    camera_dma();
}

void PIT0_IRQHandler()             
{   
    PIT_Flag_Clear(PIT0);
    Left_sp.Last_pluse   = Left_sp.Actual_pluse;
    Right_sp.Last_pluse  = Right_sp.Actual_pluse;
    lptmr_direction = gpio_get(PTA8);     
    Left_sp.Actual_pluse = lptmr_pulse_get();   
    if(lptmr_direction == 0)Left_sp.Actual_pluse = -Left_sp.Actual_pluse;
    else Left_sp.Actual_pluse = abs(Left_sp.Actual_pluse);   
    lptmr_pulse_clean();  
    Right_sp.Actual_pluse = -ftm_quad_get(FTM2);         
    ftm_quad_clean(FTM2);
    
    motor_out();
    
    //全图脉冲
    if(meet_flag == 0 && meet_start == 0)map_pluse = map_pluse + (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    
    //坡道减速脉冲
    if(podao_flag)podao_pluse = podao_pluse + (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2; 
    if(podao_flag && podao_pluse > 10000)
    {
      podao_pluse = 0;
      podao_flag = 0;
    }
    //从判方向到圆环识别
    if(yuanhuan_direction == 1 && round_left_flag == 0) is_round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    else if(yuanhuan_direction == 2 && round_right_flag == 0)is_round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    
    //防止由于坡道，颠簸，和电感不稳定引起的方向识别
    if(yuanhuan_direction && is_round_pluse > 600 && is_round_pluse < 690  && (ADVL0 > 95 || ADVR0 > 95 || ADVM > 95) && HD_OK == 0)HD_OK = 1;
    else if(yuanhuan_direction && is_round_pluse > 600 && is_round_pluse < 690  && (ADVL0 < 95 && ADVR0 < 95 && ADVM < 95) && HD_OK == 0)
    {
        yuanhuan_direction = 0;
        HD_OK = 0;    
    }
    if(yuanhuan_direction && (is_round_pluse > 1200 && is_round_pluse < 1290) && HD_OK && (ADVL0 > 95 || ADVR0 > 95 || ADVM > 95) && !(ADVL1 > 50 && ADVR1 > 50))HD_OK = 0;
    else if(yuanhuan_direction && (is_round_pluse > 1200 && is_round_pluse < 1290) && HD_OK && ((ADVL0 < 95 && ADVR0 < 95 && ADVM < 95) || (ADVL1 > 50 && ADVR1 > 50)))
    {
        yuanhuan_direction = 0;
        HD_OK = 0; 
    }
    if(round_left_flag || round_right_flag)HD_OK = 0;
    //防止方向误判
    if(is_round_pluse > 4000)yuanhuan_direction = 0;
    /*************   从环岛条件成立到打角条件成立的距离   **************/
    if(round_left_flag == 1 && round_left_flag1 == 0)round0_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    else if(round_right_flag == 1 && round_right_flag1 == 0)round0_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    
    /**************  从开始打角到成功入环的距离   ******************/
    if(dajiao_left == 1)round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;           
    else if(dajiao_right == 1)round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    
    /*************   从成功入环到出环的距离  *****************/
    if(out_flag_left1 == 1 && out_flag_left2 == 0)round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    else if(out_flag_right1 == 1 && out_flag_right2 == 0)round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    
    if(out_flag_left2)round_pluse5 += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    else if(out_flag_right2)round_pluse5 += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    
    /************    直走条件成立后的直走距离     ********************/
    if(out_left_str == 1 && out_flag_left1 == 0 && out_flag_left2 == 0)out_round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    else if(out_right_str == 1 && out_flag_right1 == 0 && out_flag_right2 == 0)out_round_pluse += (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;

    /************    会车区距离    ********************/
    if(isr_flag == 2 || meet_area == 1 || meet_start == 1)meet_pluse = meet_pluse + (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
    
    if(isr_flag == 1 && meet_area_num == 1)
    {
      S_meet_distance = S_meet_distance + (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
      if(S_meet_distance > 4000)
      {
        isr_flag = 2;
        S_meet_distance = 0;
        meet_pluse = 4000;
      }
    }
    else if(isr_flag == 1 && meet_area_num == 2)
    {
      S_meet_distance = S_meet_distance + (abs(Right_sp.Actual_pluse) + abs(Left_sp.Actual_pluse)) / 2;
      if(S_meet_distance > 5000)
      {
        isr_flag = 2;
        S_meet_distance = 0;
        meet_pluse = 5000;
      }
    }
    
    if(meet_start == 1 && meet_pluse > 2000)    //发车区走2000就够了
    {
      meet_start = 0;
      meet_area = 1;
    }
    if(meet_pluse > var7 * 100)                               //控制舵机打角
    {
      isr_flag = 0;
      meet_area = 1;
      meet_distance = 0;
      gpio_set(PTC14,0);                                          //关闭超声波
      S_meet_distance = 0;
    }
    if(meet_pluse > 12000)                              //控制会车区再次识别
    {
      meet_area = 0;
      meet_flag = 0;
      meet_pluse = 0;
      meet_run = 0;
      spot_flag = 0;
      map_pluse = 0;
    }
    if(meet_pluse > 10000 && meet_area_num == 2)stop = 1;
}

void PIT2_IRQHandler()         
{
      PIT_Flag_Clear(PIT2);
      ADCDEAL();     
      steer_error_get(); 
      if(speed1 == 0)steer_error = steer_error - var17;
     //会车区舵机控制 
     if(distance1 < 1500 && distance1 > 0 && isr_flag != 2 && meet_flag == 1)isr_flag = 1;                //开始沿边界走
     if(isr_flag == 1 && meet_distance == 0)meet_distance = distance1;       //记录最初检测值，防止撞车
     if(isr_flag == 1 && distance1 < 700 && distance1 > 0)isr_flag = 2;     //开始计算脉冲4000后恢复正常打角
      
     if(meet_distance < 700 && meet_pluse < 1000 && meet_distance > 0)steer_error = -100;
     else if(isr_flag != 0 || meet_start)steer_error -= var5; 
      
     if(meet_area_num == 0 && meet_pluse > 2000 && meet_pluse < 3500)steer_error = steer_error / 4;
     //else if(meet_area_num == 1 && meet_pluse > var7 * 100 && meet_pluse < var8 * 100)steer_error = steer_error / 3;
     else if(meet_area_num == 2 && meet_pluse > 4000 && meet_pluse < 6000)steer_error = steer_error / 5;
      
     steer_control();
     steer_out();
    
     car_stop ++;
     if(car_stop> var10 * 200)
     {
       stop = 1;
       car_stop = var10 * 200 + 1;
     }
}

void show()
{
  if(key_deal_flag1 == 0)
  {               
        key_IRQHandler();
        deal_key_event(); 
  }
  if(key_deal_flag3 == 0)
  {               
        if(var9 == 1)                                           //初始值
        {
          LCD_num_BC(const1, (uint32)ad_valueL0, 3, BLUE, RED);
          LCD_num_BC(const2, (uint32)ad_valueL1, 3, BLUE, RED);
          LCD_num_BC(const3, (uint32)ad_valueL2, 3, BLUE, RED);
          LCD_num_BC(const4, (uint32)ad_valueM, 3, BLUE, RED);
          LCD_num_BC(const5, (uint32)ad_valueR0, 3, BLUE, RED);
          LCD_num_BC(const6, (uint32)ad_valueR1, 3, BLUE, RED);
          LCD_num_BC(const7, (uint32)ad_valueR2, 3, BLUE, RED);
          LCD_num_BC(const8, (uint32)round_pluse, 5, BLUE, RED);
        }
                        
        if(var9 == 2)                                           // 归一值  
        {
          LCD_num_BC(const1, (uint32)ADVL0, 3, BLUE, RED);
          LCD_num_BC(const2, (uint32)ADVL1, 3, BLUE, RED);
          LCD_num_BC(const3, (uint32)ADVL2, 3, BLUE, RED);
          LCD_num_BC(const4, (uint32)ADVM,  3, BLUE, RED);
          LCD_num_BC(const5, (uint32)ADVR0, 3, BLUE, RED);
          LCD_num_BC(const6, (uint32)ADVR1, 3, BLUE, RED);
          LCD_num_BC(const7, (uint32)ADVR2, 3, BLUE, RED); 
          LCD_num_BC(const9, (uint32)abs(normal_error1), 3, BLUE, RED);
          LCD_num_BC(const10, (uint32)abs(compensition_error), 3, BLUE, RED);
          LCD_num_BC(const11, (uint32)abs(steer_error), 3, BLUE, RED);
          LCD_num_BC(const8, (uint32)abs(map_pluse), 7, BLUE, RED);
        }
         
        if(var9 == 3)
        {
          LCD_num_BC(const1, (uint32)round_left_flag, 3, BLUE, RED);
          LCD_num_BC(const2, (uint32)out_flag_left1, 3, BLUE, RED);
          LCD_num_BC(const3, (uint32)out_flag_left2, 3, BLUE, RED);
          LCD_num_BC(const4, (uint32)out_left_str, 3, BLUE, RED);
          
          LCD_num_BC(const5, (uint32)round_right_flag, 3, BLUE, RED);
          LCD_num_BC(const6, (uint32)out_flag_right1, 3, BLUE, RED);
          LCD_num_BC(const7, (uint32)out_flag_right2, 3, BLUE, RED);
          LCD_num_BC(const8, (uint32)out_right_str, 5, BLUE, RED); 
        }
        LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
        line_Display();
  }
}

void ad_to_one()
{
  /*****************   初始归一化电感值写入flash  ******************/
          while(key_deal_flag2 == 0)
          {
                // 左  左  右 右 中
                AD_value[0] =  ad_valueL0;
                AD_value[1] =  ad_valueL1;
                AD_value[2] =  ad_valueL2;
                
                AD_value[3] =  ad_valueR0;
                AD_value[4] =  ad_valueR1;
                AD_value[5] =  ad_valueR2;
                
                AD_value[6] =  ad_valueM;
              
                for(int ad_i = 0;ad_i < 7;ad_i++)                              
                {
                    if(AD_value[ad_i] > H[ad_i])
                    {
                          H[ad_i] = AD_value[ad_i];                  // 找最大电感值               
                    }
                }
                if(key_deal_flag1 == 0)
                {               
                  key_IRQHandler();
                  deal_key_event(); 
                }
                /**************  显示最大值   **************/
                LCD_num_BC(const1, (uint32)H[0], 3, BLUE, RED);
                LCD_num_BC(const2, (uint32)H[1], 3, BLUE, RED);
                LCD_num_BC(const3, (uint32)H[2], 3, BLUE, RED);
                
                LCD_num_BC(const4, (uint32)H[6], 3, BLUE, RED);
                
                LCD_num_BC(const5, (uint32)H[3], 3, BLUE, RED);
                LCD_num_BC(const6, (uint32)H[4], 3, BLUE, RED);
                LCD_num_BC(const7, (uint32)H[5], 3, BLUE, RED);       
          }
}
void main()
{
    DisableInterrupts;
    NVIC_SetPriorityGrouping(4);   
    NVIC_SetPriority(PIT2_IRQn,0);
    NVIC_SetPriority(PIT0_IRQn,1);              
    NVIC_SetPriority(PORTA_IRQn,2);         
    NVIC_SetPriority(DMA0_IRQn,3);          
    NVIC_SetPriority(PORTE_IRQn,4);
    
    camera_init(imgbuff);                             
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);   
    enable_irq(PORTA_IRQn);
    
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);   //鹰眼  
    enable_irq(DMA0_IRQn);
    
    pit_init_ms(PIT2,5);                                 //AD采集和舵机
    set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler);     
    enable_irq(PIT2_IRQn);
    
    pit_init_ms(PIT0,5);                                 //电机                         
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      
    enable_irq(PIT0_IRQn);
    
    while(!nrf_init()); 
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler); //NRF
    enable_irq(PORTE_IRQn);
    
    adc_init(ADL0);                                      //电感     
    adc_init(ADL1);     
    adc_init(ADL2);
    adc_init(ADR0);
    adc_init(ADR1);
    adc_init(ADR2);
    adc_init(ADM);
    
    LCD_init();                                          //液晶按键
    key_event_init();          
    
    ftm_quad_init(FTM2);                                 //编码器
    lptmr_pulse_init(LPT0_ALT1, 0xFFFF, LPT_Rising);  
    gpio_init(PTA8, GPI,0); 
    
    gpio_init(PTD0, GPI,0);                              //拨码开关
    gpio_init(PTD1, GPI,0); 
    gpio_init(PTD3, GPI,0); 
    gpio_init(PTD2, GPI,0); 
    gpio_init(PTB18,GPO,0);                             //蜂鸣器
    
    key_deal_flag1 = gpio_get(PTD0);              //获取拨码开关状态 拨右为低电平
    if(key_deal_flag1)
    {
      gpio_init(PTC14, GPO, 0);   //跑的时候一定要把开关1拨左
      gpio_set(PTC14, 0);
    }
    key_deal_flag2 = gpio_get(PTD1);
    key_deal_flag4 = gpio_get(PTD2);
                   
    steer_init();
    motor_init();
    flash_load();
    while(SCCB_WriteByte ( OV7725_CNST , var6 ) == 0);
    EnableInterrupts;               
    
    while(1)
    {     
          key_deal_flag3 = gpio_get(PTD3);
          ad_to_one();
          
          if(meet_flag == 0 && meet_start == 0 && meet_pluse == 0)
          {
            if((meet_area_num == 0 && map_pluse > var0 * 1000) || (meet_area_num == 1 && map_pluse > var1 * 1000))
            {
              camera_get_img();                                                 
              img_extract(img, imgbuff,CAMERA_SIZE); 
              meet_area_find();
              if(meet_area_num == 0 && map_pluse > var0 * 1000 && map_pluse < var0 * 1000 + 1000)gpio_set(PTB18,1); 
              else if(meet_area_num == 1 && map_pluse > var1 * 1000 && map_pluse < var1 * 1000 + 1000)gpio_set(PTB18,1); 
              else gpio_set(PTB18,0);
            }
          }
          nrf_rx(buff,32); 
          distance1 = 100 * buff[0];
          show();  
          /*if(HD_flag)gpio_set(PTB18,1);               
          else gpio_set(PTB18,0);*/
    }
}


