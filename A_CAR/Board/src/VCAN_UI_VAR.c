#include "common.h"
#include "VCAN_LCD.h"
#include "VCAN_UI_VAR.h"
#include "VCAN_NRF24L0_MSG.h"
#include "define.h"
#include "MK60_flash.h"

#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)         //������������������ȷ����ȫ

#define VAR_VALUE(var_tab)      num_info[var_tab].val       //ָ����ŵı�����ֵ
#define VAR_OLDVALUE(var_tab)   num_info[var_tab].oldval    //ָ����ŵı��������ȷ��ֵ
#define VAR_MAXVALUE(var_tab)   num_info[var_tab].maxval
#define VAR_MINVALUE(var_tab)   num_info[var_tab].minval
#define VAR_SITE(var_tab)       num_info[var_tab].site

static uint8  car_ctrl = CAR_NULL;

//��̬��һ��
extern int H[8];
extern int key_deal_flag2;

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
    //val,oldval,���ڵ���key_event_init��ʱ������Ӧ�����︳ֵ���������������ֵ��������д
    //��Ҫ����minval,maxval,{x,y}
    //���ע����Сֵ��Ҫ�������ֵ
    {0, 0, -65540, 65540, { 80,  0}},        //���� var0 ��
    {0, 0, -65540, 65540, { 80, 14}},        //���� var1 ��
    {0, 0, -65540, 65540, { 80, 28}},        //���� var2 ��
    {0, 0, -65540, 65540, { 80, 42}},        //���� var3 ��
    {0, 0, -65540, 65540, { 80, 56}},        //���� var4 ��
    {0, 0, -65540, 65540, { 80, 70}},        //���� var5 ��
    {0, 0, -65540, 65540, { 80, 84}},        //���� var6 ��
    {0, 0, -65540, 65540, { 80, 98}},        //���� var7 ��
    {0, 0, -65540, 65540, { 80,112}},        //���� var8 ��
    {0, 0, -65540, 65540, {106,  0}},        //���� var9 ��
    {0, 0, -65540, 65540, {106, 14}},        //���� var10��
    {0, 0, -65540, 65540, {106, 28}},        //���� var11��
    {0, 0, -65540, 65540, {106, 42}},        //���� var12��
    {0, 0, -65540, 65540, {106, 56}},        //���� var13��
    {0, 0, -65540, 65540, {106, 70}},        //���� var14��
    {0, 0, -65540, 65540, {106, 84}},        //���� var15��
    {0, 0, -65540, 65540, {106, 98}},        //���� var16��
    {0, 0, -65540, 65540, {106,112}},        //���� var17��
//    /*�ڶ�ҳ*/
//    {0, 0, -65540, 65540, { 80,  0}},        //���� var0 ��
//    {0, 0, -65540, 65540, { 80, 14}},        //���� var1 ��
//    {0, 0, -65540, 65540, { 80, 28}},        //���� var2 ��
//    {0, 0, -65540, 65540, { 80, 42}},        //���� var3 ��
//    {0, 0, -65540, 65540, { 80, 56}},        //���� var4 ��
//    {0, 0, -65540, 65540, { 80, 70}},        //���� var5 ��
//    {0, 0, -65540, 65540, { 80, 84}},        //���� var6 ��
//    {0, 0, -65540, 65540, { 80, 98}},        //���� var7 ��
//    {0, 0, -65540, 65540, { 80,112}},        //���� var8 ��
//    {0, 0, -65540, 65540, {106,  0}},        //���� var9 ��
//    {0, 0, -65540, 65540, {106, 14}},        //���� var10��
//    {0, 0, -65540, 65540, {106, 28}},        //���� var11��
//    {0, 0, -65540, 65540, {106, 42}},        //���� var12��
//    {0, 0, -65540, 65540, {106, 56}},        //���� var13��
//    {0, 0, -65540, 65540, {106, 70}},        //���� var14��
//    {0, 0, -65540, 65540, {106, 84}},        //���� var15��
//    {0, 0, -65540, 65540, {106, 98}},        //���� var16��
//    {0, 0, -65540, 65540, {106,112}},        //���� var17��
    
};

uint8   new_tab = 0;        //��ǰѡ��ı������
uint32  last_tab;           //�����յ��ı������



//ͬ��ָ����ֵ��tab Ϊ VAR_NUM ʱ��ʾȫ��ͬ����С����ͬ����Ӧ��
//������ͬ������ʾȫ������Ϊ�п���ͬ��ʧ�ܡ�
//static uint8 var_syn(uint8 tab);         //ͬ�����ݣ�1��ʾ�ɹ���0��ʾʧ��

void save_var2buff(var_tab_e var_num, uint8 *sendbuf);              //����Ҫ���͵ı���������д�뵽������

void var_init()
{
    uint8   var_num;
    uint32  vartemp;

    for(var_num = 0; var_num < VAR_MAX; var_num++)
    {
        get_var((var_tab_e)var_num, &vartemp);
        num_info[var_num].val       = vartemp;
        num_info[var_num].oldval    = vartemp;

        //�����Сֵ�����ֵ
        ASSERT(num_info[var_num].maxval  >=  num_info[var_num].minval );
    }
}
/************�޸� ************/
void save_var(var_tab_e var_tal, uint32 var_data)
{
    *((int32 *)(var_addr[var_tal])) = var_data;

    VAR_VALUE(var_tal) = var_data;
    VAR_OLDVALUE(var_tal) = var_data;
}
/************�޸�************/
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

//�Ա����ļӼ����д���
void var_value(ui_var_event_e ctrl)
{
    ASSERT(new_tab < VAR_MAX);


    //�޸ĵ�ǰ������ֵ
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
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
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
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
        }
        break;

    default:                        //��Чѡ�񣬲���Ҫ�л�
        break;
    }

    var_display(new_tab);
}

//�Ա�������ѡ��
void var_select(ui_var_event_e  ctrl)
{
    ASSERT(new_tab < VAR_MAX);

    uint8 old_tab = new_tab;       //�ȱ��ݵ�ǰ�������

    //�л�����һ������
    switch(ctrl)
    {
    case VAR_NEXT:                      //��һ��
        new_tab++;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = 0;               //��ͷ��ʼ
        }
        break;

    case VAR_PREV:                      //��һ��
        new_tab--;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = VAR_MAX - 1;     //��β��ʼ
        }
        break;

    case VAR_NEXT_HOLD:                 //����
        new_tab += VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = 0;               //��ͷ��ʼ
        }
        break;

    case VAR_PREV_HOLD:                 //����
        new_tab -= VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (VAR_MAX) )
        {
            new_tab = VAR_MAX - 1;     //��β��ʼ
        }
        break;

    default:                        //��Чѡ�񣬲���Ҫ�л�
        return;
    }

    var_display(old_tab);               //������һ������

    var_display(new_tab);              //����ǰ������

}


//ȷ�ϵ�ǰѡ���
void var_ok()
{
    ASSERT(new_tab < VAR_MAX);

    //�Ƚ��Ƿ��иı�ֵ
    if(VAR_VALUE(new_tab) != VAR_OLDVALUE(new_tab))   //ֵ�ı��ˣ�����Ҫ����
    {
        var_syn(new_tab);          //ͬ���µ�ֵ
        flash_erase_sector(SECTOR_NUM);                     //��������
        for(int number = 0;number <= VAR_MAX;number++)
        {
            flash_write(SECTOR_NUM , number * 4 , VAR_VALUE(number));
        }
        
        if(key_deal_flag2 == 0)
        {
            flash_erase_sector(FLASH_SECTOR_NUM - 3);  
            DELAY_MS(10);
            for(int number = 0; number <= 6; number++)
            {
                flash_write(FLASH_SECTOR_NUM - 3 , number * 4 , H[number]); 
            }           
        }
    }

    var_display(new_tab);
}

//ȡ����ǰѡ���ֵ  OK
void val_cancel()
{
    ASSERT(new_tab < VAR_MAX);

    //ֱ�ӻ�ԭ��ǰֵ
    VAR_VALUE(new_tab) = VAR_OLDVALUE(new_tab);

    var_display(new_tab);
}



//��ʾָ����ֵ��tab Ϊ VAR_MAX ʱ��ʾȫ����ʾ��С������ʾ��Ӧ��

void var_display(uint8 tab)
{
#if UI_VAR_USE_LCD

    //���屳����ʱ
#define SELECT_NO_CHANGE_BG         WHITE   //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE_BG            WHITE   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE_BG      RED     //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE_BG         RED     //û��ѡ�У����Ҹı���

    //����������ɫ
#define SELECT_NO_CHANGE            BLUE    //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE               GREEN   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE         BLUE    //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE            GREEN   //û��ѡ�У����Ҹı���

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;

    ASSERT((new_tab < VAR_MAX) && (tab <= VAR_MAX));

    if(tab == VAR_MAX)      //��ʾȫ��
    {
        i = VAR_MAX - 1;    //ѭ���Ĵ���
        tab = 0;
    }

    do
    {
        if(tab == new_tab)
        {
            //��ʾ��ǰ��ֵ���ж�ֵ�Ƿ�ı�
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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
            //��ʾ�ǵ�ǰ��ֵ
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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

        //��ʾ����
        //LCD_num_C(VAR_SITE(tab), VAR_VALUE(tab), Color, bkColor);
          LCD_num_BC(VAR_SITE(tab), VAR_VALUE(tab), 3 , Color, bkColor);
        tab++;
    }
    while(i--);         //tab != VAR_MAX ��ʱ��ִ��һ�ξ�����
#else
    tab = tab;          //�������뾯��
#endif
}

void save_var2buff(var_tab_e var_num, uint8 *sendbuf)
{
    uint32 temp;
    get_var(var_num, &temp);
    *((uint32 *)&sendbuf[COM_LEN]) = (uint32)var_num;
    *((uint32 *)&sendbuf[COM_LEN + sizeof(uint32)]) = temp;
}

//ͬ��ָ����ֵ��tab Ϊ VAR_MAX ʱ��ʾȫ��ͬ����С����ͬ����Ӧ��
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
        oldvalue = VAR_OLDVALUE(tab);                   //���ݾɵ�ֵ

        //��ֵ���Ƶ���Ӧ�ı���
        save_var((var_tab_e)tab, VAR_VALUE(tab));

        //�����µ�ֵ
        save_var2buff((var_tab_e)tab, tempbuff);        //�ѱ���д�� tempbuff ��
       /* nrf_msg_tx(COM_VAR, tempbuff);                //��������

        while(nrf_tx_state() == NRF_TXING);             //�ȴ��������

        if(NRF_TX_ERROR == nrf_tx_state())             //����ʧ��
        {
            VAR_OLDVALUE(tab) = oldvalue;               //��ԭ�ɵ�ֵ

            //����ͬ��
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


//��״̬����
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



