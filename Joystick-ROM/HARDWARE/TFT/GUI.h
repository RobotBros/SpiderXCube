#ifndef __GUI_H
#define __GUI_H




void GUI_DrawPoint(u16 x,u16 y,u16 color);
void GUI_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);
void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
void GUI_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
void GUI_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
void _draw_circle_8(int xc, int yc, int x, int y, u16 c);
void GUI_Circle(int xc, int yc,u16 c,int r, int fill);
void GUI_ShowChar(u16 x,u16 y,u16 fc, u16 bc, u8 num,u8 size,u8 mode);
void GUI_ShowString(u16 x,u16 y,u8 size,u8 *p,u8 mode);
u32 mypow(u8 m,u8 n);
void GUI_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size);
void Gui_Drawbmp16(u16 x,u16 y,const unsigned char *p);
void LCD_power_on_action(void);


#endif
