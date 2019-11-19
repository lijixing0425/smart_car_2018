#include "common.h"
#include "include.h"
#include "picture.h"

direction_struct dir[8] = {{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1}};
direction_struct right_black[400];
uint8 img_have_seacher[80*60];
int circle_flag;

int find_stop , count;
int board_num;
int max_lie;
int max_lie_num;

int meet_flag3;
int meet_flag4;
int aa;
int bb;
int cc;
int dd;
/*!
 *  @brief      二值化图像解压（空间 换 时间 解压）
 *  @param      dst             图像解压目的地址
 *  @param      src             图像解压源地址
 *  @param      srclen          二值化图像的占用空间大小
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:   sendimg(imgbuff, CAMERA_W * CAMERA_H);                    //发送到上位机
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {1, 0}; //0 和 1 分别对应的颜色
    //注：山外的摄像头 0 表示 白色，1表示 黑色
    uint8 tmpsrc;
    while(srclen --)
    {
        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}


/***********************  边界寻找  **************************/
void board_find(int plot_x,int plot_y,int direction)
{
    if(plot_y == 0 || board_num > 200 || plot_x == 15 || plot_x == 59) 
    {
      find_stop = 1;
      return;
    }
    else
    {
      for(int k = 0; k < 8 && !find_stop;  k ++)
      {
          int i = (direction + k) % 8;
          int img_limit_flag = 0;
          int changed_x = plot_x + dir[i].m;
          int changed_y = plot_y + dir[i].n;
          if(changed_x < 0 || changed_x > 59 || changed_y < 0 || changed_y > 79)img_limit_flag = 1;
          if(img_have_seacher[changed_x * 80 + changed_y] == 1 && k < 7) //表示这个点以前找过，陷入了循环
          {
            img_limit_flag = 0;
            circle_flag = 1;
            find_stop = 1;
            return;
          }
          if(!img_limit_flag && img[80 * changed_x + changed_y] == 1)
          {
              plot_x = changed_x;
              plot_y = changed_y;
              board_num++;
              right_black[board_num].m  =  changed_x;
              right_black[board_num].n  =  changed_y;
              img_have_seacher[changed_x * 80 + changed_y] = 1;
              direction = (i + 6) % 8; 
              board_find(right_black[board_num].m, right_black[board_num].n, direction);
              if(find_stop == 1) break;
          }
          count = k;
      }
      if(count == 7)
      {
        find_stop = 1;
        return;
      }
    }
}


/************************  会车区识别    *********************/
void meet_area_find()
{ 
    find_stop = 0; 
    count = 0;
    board_num = 0;
    memset(right_black,0,sizeof(right_black));
    right_black[0].m = 59;
    right_black[0].n = 79;
    Max_cha = 0;
    for(int i = 59;i > 15; i --)       
    {
        if(img[i*80 + 79] == 0)
        {
            right_black[0].m = i;
            right_black[0].n = 79;
            break; 
        }
    }  
    if(!(ADVL1 > 50 && ADVR1 > 50) && right_black[0].m > 15 && right_black[0].m < 50 && !(round_right_flag || round_left_flag || out_flag_left2 || out_flag_right2 || out_flag_left1 || out_flag_right1 || out_left_str || out_right_str))
    {
        circle_flag = 0;
        for(int i = 0; i < 80; i++)
        {
          bord[i] = 0;
        }
        for(int i = 0; i < 60; i++)
          for(int j = 0; j < 80; j ++)
            img_have_seacher[i * 80 + j] = 0;
        
        board_find(right_black[0].m, right_black[0].n, 0);
        
        if(circle_flag == 0)
        {
          for(int i = 0; i <= board_num; i ++)
          {
              if(bord[right_black[i].n] == 0)bord[right_black[i].n] = right_black[i].m;
          }
          /***************        利用边沿跳变              ******************************/
          max_lie = 0;
          max_lie_num = 0;
          for(int i = 78;i >= 0;i--)
          {
              if(abs(bord[i] - bord[i + 1]) < 9 && abs(bord[i] - bord[i + 1]) > 4)
              {
                  if(Max_cha < abs(bord[i] - bord[i + 1]))
                  {
                    Max_cha = abs(bord[i] - bord[i + 1]);  
                    max_lie = i;
                    max_lie_num ++;
                    if(abs(bord[max_lie + 2 < 79 ? max_lie + 2 : 79] - bord[max_lie - 2 > 0 ? max_lie - 2 : 0]) > 12)
                    {
                        max_lie = 0;
                        max_lie_num = 0;
                        Max_cha = 0;
                        break;
                    }
                    if(abs(bord[79] - bord[0]) < 4)
                    {
                      if(abs(bord[max_lie + 3 < 79 ? max_lie + 3 : 79] - bord[max_lie]) > 3 && abs(bord[max_lie - 3 > 0 ? max_lie - 3 : 0] - bord[max_lie]) > 3)
                      {
                        max_lie = 0;
                        max_lie_num = 0;
                        Max_cha = 0;
                        break;
                      }
                    }
                  }
              }
          }
          if(Max_cha >= var13 && Max_cha <= var14 && bord[max_lie - 3 > 0 ? max_lie - 3 : 0] != 0 && max_lie_num <= 2)meet_flag = 1;  
          /***********           识别凹陷                  *******************************/
           meet_flag3 = 0;
           meet_flag4 = 0;
           aa = 0;
           bb = 0;
           cc = 0;
           dd = 0;
          for(int i = 5; i < board_num; i ++)
          {
              if(right_black[i].m - right_black[i - 4].m <= -3 && right_black[i].m - right_black[i - 4].m >= -7)
              {
                meet_flag3 = 1;
                aa = i - 4;
                bb = i;
              }
              if(right_black[i].m - right_black[i - 4].m >= 3  && right_black[i].m - right_black[i - 4].m <= 7 && meet_flag3)
              {
                meet_flag4 = 1;
                cc = i - 4;
                dd = i;
                break;
              }
          }
          if(meet_flag3 && meet_flag4 && abs(right_black[aa].m - right_black[dd].m) <= 5 && abs(right_black[cc].m - right_black[bb].m) <= 5 && abs(right_black[cc].n - right_black[bb].n) > 26)
          {
             meet_flag = 1;
          }
        }
        
        if(ADVL1 > 50 && ADVL2 > 50 && meet_flag)
        {
          if(meet_area_num == 1)
          {
            meet_flag = 0;
            meet_area_num = 0;
            spot_flag = 0;
            gpio_set(PTC14,0); 
          }
          if(meet_area_num == 2)
          {
            meet_flag = 0;
            meet_area_num = 1;
            spot_flag = 0;
            gpio_set(PTC14,0); 
          }
        }
        if(meet_flag && meet_area_num == 0 && spot_flag == 0)
        {
          meet_area_num = 1;
          spot_flag = 1;
        }
        if(meet_flag && meet_area_num == 1 && spot_flag == 0)
        {
          meet_area_num = 2;
          spot_flag = 1;
        }
    }
}

void LCD_points(Site_t *site,uint32 point_num, uint16 rgb565)                 //画一堆点
{    
   while(point_num--)    
   {        
          LCD_point(site[point_num],rgb565);                 //画点    
   }
}

void line_Display(void)
{
        for(int i = 0; i < 80; i++)
        {
            angle_line1[i].x = i;
            angle_line1[i].y = bord[i] + 1;
        }
        for(int i = 0; i < 80; i++)
        {
            angle_lie[i].x = i;
            angle_lie[i].y = 30;
        }
        LCD_points( angle_line1,80, RED);
        LCD_points( angle_lie,80, GREEN);
}






