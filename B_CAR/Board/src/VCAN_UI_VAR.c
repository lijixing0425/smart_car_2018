#include "common.h"
#include "VCAN_LCD.h"
#include "VCAN_UI_VAR.h"
#include "VCAN_NRF24L0_MSG.h"
#include "define.h"
#include "MK60_flash.h"

extern int key_deal_flag2;
extern int H[8];
#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)         //尽量用最后面的扇区，确保安全

#define VAR_VALUE(var_tab)      num_info[var_tab].val       //指定标号的变量的值
#define VAR_OLDVALUE(var_tab)   num_info[var_tab].oldval    //指定标号的变量的最后确认值
#define VAR_MAXVALUE(var_tab)   num_info[var_tab].maxval
#define VAR_MINVALUE(var_tab)   num_info[var_tab].minval
#define VAR_SITE(var_tab)       num_info[var_tab].site

static uint8  car_ctrl = CAR_NULL;


int32 *var_addr[VAR_MAX] = 
{
  (int32 *)& var0,(int32 *)& var1,(int32 *)& var2,(int32 *)& var3,(int32 *)& var4,(int32 *)& var5,(int32 *)& var6,(int32 *)& var7,
  (int32 *)& var8,(int32 *)& var9,(int32 *)&var10,(int32 *)&var11,(int32 *)&var12,(int32 *)&var13,(int32 *)&var14,(int32 *)&var15,
  (int32 *)&var16,(int32 *)&var17,
//  (int32 *)&var18,(int32 *)&var19,(int32 *)&var20,(int32 *)&var21,(int32 *)&var22,(int32 *)&var23,(int32 *)&var24,(int32 *)&var25,
//  (int32 *)&var26,(int32 *)&var27,(int32 *)&var28,(int32 *)&var29,(int32 *)&var30,(int32 *)&var31,
//  (int32 *)&var32,(int32 *)&var33,(int32 *)&var34,(int32 *)&var35
};

ui_var_info_t  num_info[VAR_MAX] =
{
    //  {val,oldval,minval,maxval,{x,y}}
    //val,oldval,会在调用key_event_init的时候从其对应变量里赋值过来，所以这里的值可以随意写
    //需要设置minval,maxval,{x,y}
    //务必注意最小值不要大于最大值
    {0, 0, -65540, 65540, { 80,  0}},        //变量 var0 ，
    {0, 0, -65540, 65540, { 80, 14}},        //变量 var1 ，
    {0, 0, -65540, 65540, { 80, 28}},        //变量 var2 ，
    {0, 0, -65540, 65540, { 80, 42}},        //变量 var3 ，
    {0, 0, -65540, 65540, { 80, 56}},        //变量 var4 ，
    {0, 0, -65540, 65540, { 80, 70}},        //变量 var5 ，
    {0, 0, -65540, 65540, { 80, 84}},        //变量 var6 ，
    {0, 0, -65540, 65540, { 80, 98}},        //变量 var7 ，
    {0, 0, -65540, 65540, { 80,112}},        //变量 var8 ，
    {0, 0, -65540, 65540, {106,  0}},        //变量 var9 ，
    {0, 0, -65540, 65540, {106, 14}},        //变量 var10，
    {0, 0, -65540, 65540, {106, 28}},        //变量 var11，
    {0, 0, -65540, 65540, {106, 42}},        //变量 var12，
    {0, 0, -65540, 65540, {106, 56}},        //变量 var13，
    {0, 0, -65540, 65540, {106, 70}},        //变量 var14，
    {0, 0, -65540, 65540, {106, 84}},        //变量 var15，
    {0, 0, -65540, 65540, {106, 98}},        //变量 var16，
    {0, 0, -65540, 65540, {106,112}},        //变量 var17，
//    /*第二页*/
//    {0, 0, -65540, 65540, { 80,  0}},        //变量 var0 ，
//    {0, 0, -65540, 65540, { 80, 14}},        //变量 var1 ，
//    {0, 0, -65540, 65540, { 80, 28}},        //变量 var2 ，
//    {0, 0, -65540, 65540, { 80, 42}},        //变量 var3 ，
//    {0, 0, -65540, 65540, { 80, 56}},        //变量 var4 ，
//    {0, 0, -65540, 65540, { 80, 70}},        //变量 var5 ，
//    {0, 0, -65540, 65540, { 80, 84}},        //变量 var6 ，
//    {0, 0, -65540, 65540, { 80, 98}},        //变量 var7 ，
//    {0, 0, -65540, 65540, { 80,112}},        //变量 var8 ，
//    {0, 0, -65540, 65540, {106,  0}},        //变量 var9 ，
//    {0, 0, -65540, 65540, {106, 14}},        //变量 var10，
//    {0, 0, -65540, 65540, {106, 28}},        //变量 var11，
//    {0, 0, -65540, 65540, {106, 42}},        //变量 var12，
//    {0, 0, -65540, 65540, {106, 56}},        //变量 var13，
//    {0, 0, -65540, 65540, {106, 70}},        //变量 var14，
//    {0, 0, -65540, 65540, {106, 84}},        //变量 var15，
//    {0, 0, -65540, 65540, {106, 98}},        //变量 var16，
//    {0, 0, -65540, 65540, {106,112}},        //变量 var17，
    
};

uint8   new_tab = 0;        //当前选择的变量编号
uint32  last_tab;           //最后接收到的变量编号



//同步指定的值。tab 为 VAR_NUM 时表示全部同步，小于则同步对应的
//必须先同步再显示全部，因为有可能同步失败。
//static uint8 var_syn(uint8 tab);         //同步数据，1表示成功，0表示失败

void save_var2buff(var_tab_e var_num, uint8 *sendbuf);              //把需要发送的变量的数据写入到缓冲区

void var_init()
{
    uint8   var_num;
    uint32  vartemp;

    for(var_num = 0; var_num < VAR_MAX; var_num++)
    {
        get_var((var_tab_e)var_num, &vartemp);
        num_info[var_num].val       = vartemp;
        num_info[var_num].oldval    = vartemp;

        //检测最小值与最大值
        ASSERT(num_info[var_num].maxval  >=  num_info[var_num].minval );
    }
}
/************修改 ************/
void save_var(var_tab_e var_tal, uint32 var_data)
{
    *((int32 *)(var_addr[var_tal])) = var_data;

    VAR_VALUE(var_tal) = var_data;
    VAR_OLDVALUE(var_tal) = var_data;
}
/************修改************/
void get_var(var_tab_e var_tal, uint32 *var_data)
{
    *var_data = (int32) * ((int32 *)(var_addr[var_tal]));
}


void updata_var(var_tab_e var_tal)
{
    uint32 vartemp;

    get_var(var_tal, &vartemp);

    VAR_VALUE(var_tal) = vartemp;
}

//对变量的加减进行处理
void var_value(ui_var_event_e ctrl)
{
    ASSERT(new_tab < VAR_MAX);


    //修改当前变量的值
    switch(ctrl)
    {
    case VAR_ADD:
        if(VAR_VALUE(new_tab) < VAR_MAXVALUE(new_tab))
        {
            VAR_VALUE(new_tab)++;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab);
        }
        break;

    case VAR_SUB:
        if(VAR_VALUE(new_tab) > VAR_MINVALUE(new_tab))
        {
            VAR_VALUE(new_tab)--;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//最小值减一为最大值
        }
        break;

    case VAR_ADD_HOLD:
        if(   (VAR_MAXVALUE(new_tab) - VAR_VALUE(new_tab))  >  VAR_VALUE_HOLE_OFFSET )
        {
            VAR_VALUE(new_tab) += VAR_VALUE_HOLE_OFFSET;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab);
        }
        break;

    case VAR_SUB_HOLD:
        if( ( VAR_VALUE(new_tab) - VAR_MINVALUE(new_tab)) > VAR_VALUE_HOLE_OFFSET  )
        {
            VAR_VALUE(new_tab) -= VAR_VALUE_HOLE_OFFSET;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//最小值减一为最大值
        }
        break;

    default:                        //无效选择，不需要切换
        break;
    }

    var_display(new_tab);
}

//对变量进行选择
void var_select(ui_var_event_e  ctrl)
{
    ASSERT(new_tab < VAR_MAX);

    uint8 old_tab = new_tab;       //先备份当前变量标号

    //切换到下一个变量
    switch(ctrl)
    {
    case VAR_NEXT:                      //下一个
        new_tab++;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = 0;               //从头开始
        }
        break;

    case VAR_PREV:                      //上一个
        new_tab--;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = VAR_MAX - 1;     //从尾开始
        }
        break;

    case VAR_NEXT_HOLD:                 //快下
        new_tab += VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = 0;               //从头开始
        }
        break;

    case VAR_PREV_HOLD:                 //快上
        new_tab -= VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = VAR_MAX - 1;     //从尾开始
        }
        break;

    default:                        //无效选择，不需要切换
        return;
    }

    var_display(old_tab);               //处理上一个变量

    var_display(new_tab);              //处理当前变量：

}


//确认当前选择的
void var_ok()
{
    ASSERT(new_tab < VAR_MAX);

    //比较是否有改变值
    if(VAR_VALUE(new_tab) != VAR_OLDVALUE(new_tab))   //值改变了，则需要处理
    {
        var_syn(new_tab);          //同步新的值
        flash_erase_sector(SECTOR_NUM);                     //擦除扇区
        for(int number = 0;number <= VAR_MAX;number++)
        {
            flash_write(SECTOR_NUM , number * 4 , VAR_VALUE(number));
        }
    }
    if(key_deal_flag2 == 0)
    {
        flash_erase_sector(FLASH_SECTOR_NUM - 3); 
        DELAY_MS(10);
        for(int number = 0; number < 7; number++)
        {
            flash_write(FLASH_SECTOR_NUM - 3 , number * 4 , H[number]);    
        }
    }
    var_display(new_tab);
}

//取消当前选择的值  OK
void val_cancel()
{
    ASSERT(new_tab < VAR_MAX);

    //直接还原当前值
    VAR_VALUE(new_tab) = VAR_OLDVALUE(new_tab);

    var_display(new_tab);
}



//显示指定的值。tab 为 VAR_MAX 时表示全部显示，小于则显示对应的

void var_display(uint8 tab)
{
#if UI_VAR_USE_LCD

    //定义背景延时
#define SELECT_NO_CHANGE_BG         WHITE   //当前选中，而且没有改变
#define SELECT_CHANGE_BG            WHITE   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE_BG      RED     //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE_BG         RED     //没有选中，而且改变了

    //定义文字颜色
#define SELECT_NO_CHANGE            BLUE    //当前选中，而且没有改变
#define SELECT_CHANGE               GREEN   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE         BLUE    //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE            GREEN   //没有选中，而且改变了

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;

    ASSERT((new_tab < VAR_MAX) && (tab <= VAR_MAX));

    if(tab == VAR_MAX)      //显示全部
    {
        i = VAR_MAX - 1;    //循环的次数
        tab = 0;
    }

    do
    {
        if(tab == new_tab)
        {
            //显示当前的值：判断值是否改变
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  SELECT_NO_CHANGE;
                bkColor =  SELECT_NO_CHANGE_BG;
            }
            else
            {
                Color   =  SELECT_CHANGE;
                bkColor =  SELECT_CHANGE_BG;
            }
        }
        else
        {
            //显示非当前的值
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  NO_SELECT_NO_CHANGE;
                bkColor =  NO_SELECT_NO_CHANGE_BG;

            }
            else
            {
                Color   =  NO_SELECT_CHANGE;
                bkColor =  NO_SELECT_CHANGE_BG;
            }
        }

        //显示文字
        //LCD_num_C(VAR_SITE(tab), VAR_VALUE(tab), Color, bkColor);
          LCD_num_BC(VAR_SITE(tab), VAR_VALUE(tab), 3 , Color, bkColor);
        tab++;
    }
    while(i--);         //tab != VAR_MAX 的时候，执行一次就跳出
#else
    tab = tab;          //消除编译警告
#endif
}

void save_var2buff(var_tab_e var_num, uint8 *sendbuf)
{
    uint32 temp;
    get_var(var_num, &temp);
    *((uint32 *)&sendbuf[COM_LEN]) = (uint32)var_num;
    *((uint32 *)&sendbuf[COM_LEN + sizeof(uint32)]) = temp;
}

//同步指定的值。tab 为 VAR_MAX 时表示全部同步，小于则同步对应的
uint8 var_syn(uint8 tab)
{
    ASSERT((new_tab < VAR_MAX) && (tab <= VAR_MAX));

    uint8  i = 0;
    uint8 tempbuff[DATA_PACKET];
    uint32 oldvalue;

    if(tab == VAR_MAX)
    {
        i = VAR_MAX - 1;
        tab = 0;
    }

    do
    {
        oldvalue = VAR_OLDVALUE(tab);                   //备份旧的值

        //把值复制到对应的变量
        save_var((var_tab_e)tab, VAR_VALUE(tab));

        //发送新的值
        save_var2buff((var_tab_e)tab, tempbuff);        //把变量写进 tempbuff 里
       /* nrf_msg_tx(COM_VAR, tempbuff);                //发送数据

        while(nrf_tx_state() == NRF_TXING);             //等待发送完成

        if(NRF_TX_ERROR == nrf_tx_state())             //发送失败
        {
            VAR_OLDVALUE(tab) = oldvalue;               //还原旧的值

            //放弃同步
            return 0;
        }*/
        tab++;
    }
    while(i--);

    return 1;
}

uint8    car_ctrl_get()
{
    return  car_ctrl;
}


//车状态控制
//uint8 car_ctrl_syn(CAR_CTRL_MODE_e mode)
//{
//    uint8 ret;
//    ASSERT(mode < CAR_CTRL_MAX);
//
//    VAR_VALUE(CAR_CTRL) =  mode;
//
//    ret =  var_syn(CAR_CTRL);
//
//    var_display(CAR_CTRL);
//
//    return ret;
//};



