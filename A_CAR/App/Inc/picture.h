#ifndef _PICTURE_H_
#define _PICTURE_H_

extern void img_extract(uint8 *dst, uint8 *src, uint32 srclen);

extern void board_find();
extern void meet_area_find();
extern void meet_solve();

extern void LCD_points(Site_t *site,uint32 point_num, uint16 rgb565);
extern void line_Display();


typedef struct
{
    int m;
    int n;
} direction_struct;            //变量信息

#endif