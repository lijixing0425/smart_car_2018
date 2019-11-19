#include "common.h"
#include "include.h"

extern ui_var_info_t  num_info[VAR_MAX];
extern void var_display(uint8 tab);

/*************  �������  *******************/
int key_deal_flag1;
int key_deal_flag2;
int key_deal_flag3;
int key_deal_flag4;
uint16 car_stop;

/*************  ��һ�� FLASH ***************/
int H[8] = {100, 100, 100 , 100,  100, 100, 100};
int max_min_changed = 0;
int AD_value[8];   

/*************  ������   ******************/
uint8 dat[3];
uint8 num; 
uint8 dat1[3];

extern int meet_run;
extern int normal_error;
extern Speed_struct Left_sp,Right_sp;
extern int speed1;

int run_flag;
int meet_distance;
int S_meet_distance;                            


/**************  �������ж�  ***********/
void UART4_RX_TX_IRQHandler(void)
{
    uart_getchar(UART4,&dat[num]);
   //dat1[num] = (uint8)(dat[num]);

    if(dat[0] != 0xa5)num = 0;	                //���ͷ֡�Ƿ���ȷ������ȷ�����½���
    else num++;
    
    if(num==3)					//������ɣ���ʼ��������
    {
            num = 0;
            distance1 = dat[1]<<8 | dat[2];
            buff[0] = (uint8)(distance1 / 100);   
    }
}

/*************  FLASH ��ȡ   **********/     
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
    
    key_deal_flag2 = gpio_get(PTD1);
    if(key_deal_flag2 == 1)                        //���뿪�����󣬶�ȡɨ�������ֵ����Сֵ
    {
        for(int num=0;num <= 6;num++)
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
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}

void PORTE_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;

    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //���жϱ�־λ

    n = 27;
    if(flag & (1 << n))                                 //PTE27�����ж�
    {
        nrf_handler();
    }
}

void DMA0_IRQHandler()
{
    camera_dma();
}

/**********  ����ջ�  ********/
void PIT0_IRQHandler()           
{ 
    PIT_Flag_Clear(PIT0); 
  
    Left_sp.Last_pluse = Left_sp.Actual_pluse;
    Right_sp.Last_pluse= Right_sp.Actual_pluse;
  
    /**************   �Ȿ�η�������         ***************/     
    lptmr_direction = gpio_get(PTA8);          
    Left_sp.Actual_pluse = lptmr_pulse_get();   
    if(lptmr_direction == 0)Left_sp.Actual_pluse = -Left_sp.Actual_pluse;
    else Left_sp.Actual_pluse = abs(Left_sp.Actual_pluse);   
    lptmr_pulse_clean();  
    
    Right_sp.Actual_pluse = -ftm_quad_get(FTM2);         
    ftm_quad_clean(FTM2);
    
    motor_out();               
    
    /**************   �ᳵ������һ�����벻ʶ��ᳵ��   *************/
    if(meet_start == 0 && meet_flag == 0)map_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2; 
    
    /************** �ᳵ��  ******/
    if(isr_flag == 1)S_meet_distance += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;   //S �䳬�������ô�
    
    if(S_meet_distance > 4000 && this_car == 1)
    {
          isr_flag = 2;
          meet_pluse = 4000;
    }
    else if(S_meet_distance > 5000 && this_car == 2)
    {
          isr_flag = 2;
          meet_pluse = 5000;
    }
    
    
    if(isr_flag == 2 || run_flag == 1 || meet_start == 1)meet_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
    
    if(meet_pluse > 2000 && meet_start == 1)
    {
        meet_start = 0;
        run_flag   = 1;    // �����ۼ�����
    }
    
    if(meet_pluse > var7 * 100)  // ����ʼ����
    {
        isr_flag   = 0;
        buff[4] = 0;
        run_flag   = 1;
        meet_distance = 0;
        S_meet_distance = 0;
    }
    if(meet_pluse > 12000)
    {
        meet_flag = 0;
        meet_run  = 0;
        meet_pluse= 0;
        run_flag  = 0;
        spot_flag = 1;
        map_pluse = 0;
    }
    
    if(this_car == 2 && meet_pluse > 10000)stop = 1;
    
    //�µ���������
    if(podao_flag)podao_pluse = podao_pluse + (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2; 
    if(podao_flag && podao_pluse > 10000)
    {
      podao_pluse = 0;
      podao_flag = 0;
    }
    //���з���Բ��ʶ��
    if(yuanhuan_direction == 1 && round_left_flag == 0) is_round_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
    else if(yuanhuan_direction == 2 && round_right_flag == 0)is_round_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
    //��ֹ�����µ����������͵�в��ȶ�����ķ���ʶ��
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
    //��ֹ��������
    if(is_round_pluse > 4000)yuanhuan_direction = 0;
    //ʶ�� �� ��� 
    if(round_left_flag == 1 && round_left_flag1 == 0)round_pluse1 += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
    else if(round_right_flag == 1 && round_right_flag1 == 0)round_pluse1 += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2; 
  
    //��� �� ����
    if(round_left_flag1 == 1 && out_flag_left2 == 0)round_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
    else if(round_right_flag1 == 1 && out_flag_right2 == 0)round_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;  
    
    //ֱ�߳���������� 
    if(out_flag_left2)str_Yes_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
    else if(out_flag_right2)str_Yes_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
    
    //ֱ�ߵľ��� 
    if(out_left_str == 1)out_round_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2; 
    else if( out_right_str == 1)out_round_pluse += (abs(Left_sp.Actual_pluse) + abs(Right_sp.Actual_pluse)) / 2;
     
}

/*********** ��вɼ�  *******/
void PIT2_IRQHandler()        
{
      PIT_Flag_Clear(PIT2);  
  
      ADCDEAL();           
      steer_error_get();    // ƫ���ȡ
      if(speed1 == 0)steer_error -= var17;
      

      /************ �ᳵ����ǿ���  *****************/
      if(distance1 < 1500 && distance1 != 0 && isr_flag != 2 && meet_flag == 1)isr_flag = 1;     //��ʼ���
      if(isr_flag == 1 && meet_distance == 0)meet_distance = distance1;                         //��¼��ʼ���ʱ�� ����
      if(isr_flag == 1 && distance1 <= 700 && distance1 != 0 && meet_flag == 1)isr_flag = 2;    //˫��ʮ�ֽӽ���������
      
      //�����ᳵ   ���Ҵ�� meet_pluse > 2000ʱ meet_start = 0
                         // meet_pluse > 4000ʱ isr_flag   = 0
      if(isr_flag != 0 || meet_start == 1)steer_error -= var5;    //  �ᳵʱ�Ĵ��ƫ����     
      if(meet_distance != 0 && meet_distance < 700 && meet_pluse < 1000)steer_error = -100;    //��һ������̫��ֱ�����Ҵ���
      
      if(this_car == 0 && meet_pluse > 2000 && meet_pluse < 3500 )              //����������  2000����ʱ meet_start = 0
      {
            steer_error = steer_error / 4;
      }
      else if(this_car == 1 && meet_pluse > var7 * 100 && meet_pluse < var8 * 1000 )        //�м�ᳵ������
      {
            steer_error = steer_error / 3;
      }
      else if(this_car == 2 && meet_pluse > 4000 && meet_pluse < 6000 )        //���ͣ��������
      {
            steer_error = steer_error / 5;
      }
      
     
      steer_control();     
      steer_out();         
      
      car_stop ++;
      if(car_stop>   var10 * 200)
      {
        car_stop = var10 * 200 + 1;
        stop = 1;
      }
        
}

void ad_to_one()
{
    while(key_deal_flag2 == 0)
    {
      key_deal_flag2 = gpio_get(PTD1);
      // ��  ��  �� �� ��
      AD_value[0] =  ad_valueL0;
      AD_value[1] =  ad_valueL1;
      AD_value[2] =  ad_valueL2;
      
      AD_value[3] =  ad_valueR0;
      AD_value[4] =  ad_valueR1;
      AD_value[5] =  ad_valueR2;
      
      AD_value[6] =  ad_valueM;
    
      for(int ad_i = 0;ad_i <= 6;ad_i++)                              
      {
          if(AD_value[ad_i] > H[ad_i])
          {
                H[ad_i] = AD_value[ad_i];                  // �������ֵ
                max_min_changed = 1;             
          }
      }

      if(key_deal_flag1 == 0)
      {
            key_IRQHandler();
            deal_key_event(); 
      }  
      
      /**************  ��ʾ���ֵ   **************/
      LCD_num_BC(const1, (uint32)H[0], 3, BLUE, RED);
      LCD_num_BC(const2, (uint32)H[1], 3, BLUE, RED);
      LCD_num_BC(const3, (uint32)H[2], 3, BLUE, RED);
      
      LCD_num_BC(const5, (uint32)H[3], 3, BLUE, RED);
      LCD_num_BC(const6, (uint32)H[4], 3, BLUE, RED);
      LCD_num_BC(const7, (uint32)H[5], 3, BLUE, RED);
      
      LCD_num_BC(const4, (uint32)H[6], 3, BLUE, RED);
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
            /************  ��ʼֵ  *********/
            if(var9 == 1)
            {
                LCD_num_BC(const1, (uint32)ad_valueL0, 3, BLUE, RED);
                LCD_num_BC(const2, (uint32)ad_valueL1, 3, BLUE, RED);
                LCD_num_BC(const3, (uint32)ad_valueL2, 3, BLUE, RED);
                
                LCD_num_BC(const5, (uint32)ad_valueR0, 3, BLUE, RED);
                LCD_num_BC(const6, (uint32)ad_valueR1, 3, BLUE, RED);
                LCD_num_BC(const7, (uint32)ad_valueR2, 3, BLUE, RED);
                
                LCD_num_BC(const4, (uint32)ad_valueM,  3, BLUE, RED);
                
                LCD_num_BC(const8, (uint32)round_pluse,  5, BLUE, RED);
                
                
            }  
            /************* ��һ��  *********/
            if(var9 == 2)
            {
                LCD_num_BC(const1, (uint32)ADVL0, 3, BLUE, RED);
                LCD_num_BC(const2, (uint32)ADVL1, 3, BLUE, RED);
                LCD_num_BC(const3, (uint32)ADVL2, 3, BLUE, RED);
                
                LCD_num_BC(const5, (uint32)ADVR0, 3, BLUE, RED);
                LCD_num_BC(const6, (uint32)ADVR1, 3, BLUE, RED);
                LCD_num_BC(const7, (uint32)ADVR2, 3, BLUE, RED);
                
                LCD_num_BC(const4, (uint32)ADVM, 3, BLUE, RED);
                
                LCD_num_BC(const9, (uint32)abs(normal_error), 3, BLUE, RED);
                LCD_num_BC(const10, (uint32)abs(compensition_error), 3, BLUE, RED);
                LCD_num_BC(const11, (uint32)abs(steer_error), 3, BLUE, RED); 
                
                LCD_num_BC(const8,(uint32)map_pluse,7,BLUE,RED);
            }
         
            /***********     Բ����־λ  **********************/
            if(var9 == 3)
            {
                LCD_num_BC(const1, (uint32)round_left_flag, 3, BLUE, RED);
                LCD_num_BC(const5, (uint32)round_right_flag, 3, BLUE, RED);  
                LCD_num_BC(const2, (uint32)out_flag_left1, 3, BLUE, RED);
                LCD_num_BC(const6, (uint32)out_flag_right1, 3, BLUE, RED);   
                LCD_num_BC(const3, (uint32)out_flag_left2, 3, BLUE, RED);
                LCD_num_BC(const7, (uint32)out_flag_right2, 3, BLUE, RED);
                LCD_num_BC(const4, (uint32)out_left_str, 3, BLUE, RED);
                LCD_num_BC(const8, (uint32)out_right_str, 3, BLUE, RED);
            }
            LCD_Img_Binary_Z(site, size, imgbuff, imgsize);
            line_Display();
      }  
}

void main()
{
    DisableInterrupts;
    NVIC_SetPriorityGrouping(4);  
    
    NVIC_SetPriority(UART4_RX_TX_IRQn,0);   //������
    NVIC_SetPriority(PIT2_IRQn,1);         //���
    NVIC_SetPriority(PIT0_IRQn,2);        //���
    NVIC_SetPriority(PORTA_IRQn,3);       //����ͷ���ж� + ���ж�
    NVIC_SetPriority(DMA0_IRQn,4);       // DMA �ж�        
    NVIC_SetPriority(PORTE_IRQn,5);      // NRF �˿��ж�
    
    camera_init(imgbuff);                                       //����ͷ
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);   
    enable_irq(PORTA_IRQn);
    
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      
    enable_irq(DMA0_IRQn);
    
    pit_init_ms(PIT0,5);                                        //��������ж�
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      
    enable_irq(PIT0_IRQn);
   
    pit_init_ms(PIT2,5);                                        //����ж�
    set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler);     
    enable_irq(PIT2_IRQn);
    
    while(!nrf_init());                                         //nrf
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);
    enable_irq(PORTE_IRQn);
 
    adc_init(ADL0);                                              //���
    adc_init(ADL1);    
    adc_init(ADL2);    
    adc_init(ADR0);
    adc_init(ADR1);
    adc_init(ADR2);    
    adc_init(ADM);
                                      
    LCD_init();                                                  //Һ��
    key_event_init();           
    
    ftm_quad_init(FTM2);                                        //������
    lptmr_pulse_init(LPT0_ALT1, 0xFFFF, LPT_Rising);  
    gpio_init(PTA8, GPI,0);

    gpio_init(PTD0, GPI,0);                                     //���뿪��
    gpio_init(PTD1, GPI,0);
    gpio_init(PTD2, GPI,0);
    gpio_init(PTD3, GPI,0);
    
    gpio_init(PTB18,GPO,0);
    
    key_deal_flag1 = gpio_get(PTD0);
    key_deal_flag2 = gpio_get(PTD1);
    key_deal_flag4 = gpio_get(PTD2);
    steer_init();
    motor_init();
    flash_load();
    while(SCCB_WriteByte ( OV7725_CNST , var6 ) == 0);
    
    uart_init (UART4, 115200);                                 //������
    set_vector_handler(UART4_RX_TX_VECTORn ,UART4_RX_TX_IRQHandler);
    enable_irq(UART4_RX_TX_IRQn);
    uart_rx_irq_en(UART4);     
    EnableInterrupts;      
    
    while(1)
    { 
        ad_to_one(); 
        key_deal_flag3 = gpio_get(PTD3);
        show();
   
        //  �ᳵ�����࿪������ͷ
       if(meet_flag == 0 && meet_start == 0 && meet_pluse == 0 && ((this_car == 0 && map_pluse > var0 * 1000) || (this_car == 1 && map_pluse > var1 * 1000)))             
       {
              camera_get_img();     
              img_extract(img, imgbuff,CAMERA_SIZE); 
              meet_area_find();
              meet_solve();
              
              if(this_car == 0 && map_pluse > var0 * 1000 && map_pluse < var0 * 1000 + 1000)gpio_set(PTB18,1);
              else if(this_car == 1 && map_pluse > var1 * 1000 && map_pluse < var1 * 1000 + 1000)gpio_set(PTB18,1);
              else gpio_set(PTB18,0);
              
        }
        
        if(meet_flag == 1)buff[4] = meet_flag;
        nrf_tx(buff, 32);
    }
}


