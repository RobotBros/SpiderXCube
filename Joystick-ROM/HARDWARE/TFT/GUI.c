#include "stm32f10x.h"
#include "Lcd_Driver.h"
#include "GUI.h"
#include "delay.h"
#include "font.h"

//******************************************************************
//功能：    GUI描绘一个点
//输入参数：x:光标位置x坐标
//        	y:光标位置y坐标
//			color:要填充的颜色
//返回值：  无
//修改记录：无
//******************************************************************
void GUI_DrawPoint(u16 x,u16 y,u16 color)
{
	Lcd_SetXY(x,y);				//设置光标位置 
	LCD_WriteData_16Bit(color); 
}

//******************************************************************
//功能：    在指定区域内填充颜色
//输入参数：sx:指定区域开始点x坐标
//        	sy:指定区域开始点y坐标
//			ex:指定区域结束点x坐标
//			ey:指定区域结束点y坐标
//        	color:要填充的颜色
//返回值：  无
//修改记录：无
//******************************************************************
void GUI_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{  	

	u16 i,j;			
	u16 width=ex-sx+1; 				//得到填充的宽度
	u16 height=ey-sy+1;				//高度
	
	Lcd_SetRegion(sx,sy,ex-1,ey-1);	//设置显示窗口
	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		LCD_WriteData_16Bit(color);	//写入数据 	 
	}

	Lcd_SetRegion(0,0,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);		//恢复窗口设置为全屏
}

//******************************************************************
//功能：    GUI画矩形(填充)
//输入参数：(x1,y1),(x2,y2):矩形的对角坐标
//返回值：  无
//修改记录：无
//******************************************************************   
void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	GUI_Fill(x1,y1,x2,y2,Point_color);
}

//******************************************************************
//功能：    GUI画线
//输入参数：x1,y1:起点坐标
//        	x2,y2:终点坐标 
//返回值：  无
//修改记录：无
//****************************************************************** 
void GUI_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; 					//计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 				//设置单步方向 
	else if(delta_x==0)incx=0;			//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;			//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )		//画线输出 
	{  
		Lcd_DrawPoint(uRow,uCol);		//画点 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
}

//******************************************************************
//功能：    GUI画矩形(非填充)
//输入参数：(x1,y1),(x2,y2):矩形的对角坐标
//返回值：  无
//修改记录：无
//******************************************************************  
void GUI_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	GUI_DrawLine(x1,y1,x2,y1);
	GUI_DrawLine(x1,y1,x1,y2);
	GUI_DrawLine(x1,y2,x2,y2);
	GUI_DrawLine(x2,y1,x2,y2);
}  

//******************************************************************
//功能：    8对称性画圆算法(内部调用)
//输入参数：(xc,yc) :圆中心坐标
// 			(x,y):光标相对于圆心的坐标
//         	c:填充的颜色
//返回值：  无
//修改记录：无
//******************************************************************  
void _draw_circle_8(int xc, int yc, int x, int y, u16 c)
{
	GUI_DrawPoint(xc + x, yc + y, c);

	GUI_DrawPoint(xc - x, yc + y, c);

	GUI_DrawPoint(xc + x, yc - y, c);

	GUI_DrawPoint(xc - x, yc - y, c);

	GUI_DrawPoint(xc + y, yc + x, c);

	GUI_DrawPoint(xc - y, yc + x, c);

	GUI_DrawPoint(xc + y, yc - x, c);

	GUI_DrawPoint(xc - y, yc - x, c);
}

//******************************************************************
//功能：    在指定位置画一个指定大小的圆(填充)
//输入参数：(xc,yc) :圆中心坐标
//         	c:填充的颜色
//		 	r:圆半径
//		 	fill:填充判断标志，1-填充，0-不填充
//返回值：  无
//修改记录：无
//******************************************************************  
void GUI_Circle(int xc, int yc,u16 c,int r, int fill)
{
	int x = 0, y = r, yi, d;

	d = 3 - 2 * r;


	if (fill) 
	{
		// 如果填充（画实心圆）
		while (x <= y) {
			for (yi = x; yi <= y; yi++)
				_draw_circle_8(xc, yc, x, yi, c);

			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	} else 
	{
		// 如果不填充（画空心圆）
		while (x <= y) {
			_draw_circle_8(xc, yc, x, y, c);
			if (d < 0) {
				d = d + 4 * x + 6;
			} else {
				d = d + 4 * (x - y) + 10;
				y--;
			}
			x++;
		}
	}
}


//******************************************************************
//功能：    显示单个英文字符
//输入参数：(x,y):字符显示位置起始坐标
//        		fc:前置画笔颜色
//			bc:背景颜色
//			num:数值（0-94）
//			size:字体大小 12 16
//			mode:模式  0,填充模式;1,叠加模式
//返回值：  无
//修改记录：无
//******************************************************************  
void GUI_ShowChar(u16 x,u16 y,u16 fc, u16 bc, u8 num,u8 size,u8 mode)
{  
    	u8 temp;
    	u8 pos,t;
	u16 colortemp=Point_color; 	
		   
	num=num-' ';								//得到偏移后的值
	Lcd_SetRegion(x,y,x+size/2-1,y+size-1);		//设置单个文字显示窗口
	if(!mode)									 //非叠加方式
	{		
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//调用1206字体
			else temp=asc2_1608[num][pos];		 //调用1608字体
			for(t=0;t<size/2;t++)
		    	{                 
		       	 if(temp&0x01)LCD_WriteData_16Bit(fc); 
				 else LCD_WriteData_16Bit(bc); 
				temp>>=1; 				
		    	}			
		}	
	}
	else												 //叠加方式
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];	//调用1206字体
			else temp=asc2_1608[num][pos];		 	//调用1608字体
			for(t=0;t<size/2;t++)
		    	{ 
		    		Point_color = fc;
		       	if(temp&0x01)Lcd_DrawPoint(x+t,y+pos);//画一个点    
		        	temp>>=1; 
		    }
		}
	}
	Point_color=colortemp;	
	Lcd_SetRegion(0,0,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);					//恢复窗口为全屏    	   	 	  
} 

//******************************************************************
//功能：    显示英文字符串
//输入参数：x,y :起点坐标	 
//			size:字体大小
//			*p:字符串起始地址
//			mode:模式	0,填充模式;1,叠加模式
//返回值：  无
//修改记录：无
//******************************************************************  	  
void GUI_ShowString(u16 x,u16 y,u8 size,u8 *p,u8 mode)
{         
    while((*p<='~')&&(*p>=' '))//判断是不是非法字符!
    {   
	if(x>(LCD_X_MAX_PIXEL-1)||y>(LCD_Y_MAX_PIXEL-1))  return;     
        GUI_ShowChar(x,y,Point_color,Back_color,*p,size,mode);
        x+=size/2;
        p++;
    }  
} 

//******************************************************************
//功能：    求m的n次方(gui内部调用)
//输入参数：m:乘数
//	        n:幂
//返回值：  m的n次方
//修改记录：无
//******************************************************************  
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

//******************************************************************
//功能：    显示单个数字变量值
//输入参数：x,y :起点坐标	 
//			len :指定显示数字的位数
//			size:字体大小(12,16)
//			color:颜色
//			num:数值(0~4294967295)
//返回值：  无
//修改记录：无
//******************************************************************  			 
void GUI_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				GUI_ShowChar(x+(size/2)*t,y,Point_color,Back_color,' ',size,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	GUI_ShowChar(x+(size/2)*t,y,Point_color,Back_color,temp+'0',size,0); 
	}
} 

//******************************************************************
//功能：    显示一副16位BMP图像
//输入参数：x,y :起点坐标
// 			*p :图像数组起始地址
//返回值：  无
//修改记录：无
//******************************************************************  
void Gui_Drawbmp16(u16 x,u16 y,const unsigned char *p) //显示40*40 QQ图片
{
  	int i; 
	unsigned char picH,picL; 
	
	Lcd_SetRegion(x,y,x+40-1,y+40-1);				 //窗口设置
    	for(i=0;i<40*40;i++)
	{	
	 	picL=*(p+i*2);								//数据低位在前
		picH=*(p+i*2+1);				
		LCD_WriteData_16Bit(picH<<8|picL);  						
	}	
	Lcd_SetRegion(0,0,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);//恢复显示窗口为全屏	
}

//----------------------------------
//系统开机画面显示
//-----------------------------------
void LCD_power_on_action(void)
{
	u8 cnt;
	
	LCD_BL_PWM_duty_set(40);
//----------LOGO-------------------//
	//Gui_Drawbmp16();
	//Lcd_Clear(Back_color);
	//for(cnt=0;cnt<255;cnt++)
	//	{GUI_Circle(64,64,GREEN+10,cnt+1,0);delay_ms(10);}	
	//GUI_Tetris_Frame_ini();
	//delay_ms(1000);
	//delay_ms(1000);
//-----------LOGO TXT--------------//
	Back_color = LIGHTBLUE;
	Point_color = BLACK;
	Lcd_Clear(Back_color,0,0);
	delay_ms(200);
	GUI_ShowString(30,40,16,"RobotBros",0);
	GUI_ShowString(20,60,16,"RobotBros.cn",0);
	delay_ms(1000);
	delay_ms(1000);
//------------Initial state------------//	
	Back_color = BLACK;
	Point_color = BLUE;
	Lcd_Clear(Back_color,0,0);
	GUI_ShowString(10,0,12,"Spider_System",0);
	Point_color = RED;
	GUI_ShowString(10,12,12,"Starting... ",0);
	delay_ms(500);
	Point_color = GREEN;
	//----------led-----------------//
	if(System_ini_state.led_ini_state) GUI_ShowString(10,24,12,"LED is OK...",0);
	else {Point_color = RED;GUI_ShowString(10,24,12,"LED is error...",0);Point_color = GREEN;}
	delay_ms(500);
	//----------lcd-----------------//
	if(System_ini_state.lcd_ini_state) GUI_ShowString(10,36,12,"LCD is OK...",0);
	else {Point_color = RED;GUI_ShowString(10,36,12,"LCD is error...",0);Point_color = GREEN;}
	delay_ms(500);
	//----------DMA-----------------//
	if(System_ini_state.dma_ini_state) GUI_ShowString(10,48,12,"DMA is OK...",0);
	else {Point_color = RED;GUI_ShowString(10,48,12,"DMA is error...",0);Point_color = GREEN;}
	delay_ms(500);
	//----------adc-----------------//
	if(System_ini_state.adc_ini_state) GUI_ShowString(10,60,12,"ADC is OK...",0);
	else {Point_color = RED;GUI_ShowString(10,60,12,"ADC is error...",0);Point_color = GREEN;}
	delay_ms(500);
	//----------nrf-----------------//
	if(System_ini_state.nrf_ini_state) GUI_ShowString(10,72,12,"2.4G is OK...",0);
	else {Point_color = RED;GUI_ShowString(10,72,12,"2.4G is error...",0);Point_color = GREEN;}
	delay_ms(500);
	//----------beep-----------------//
	if(System_ini_state.beep_ini_state) GUI_ShowString(10,84,12,"Beep is OK...",0);
	else {Point_color = RED;GUI_ShowString(10,84,12,"Beep is error...",0);Point_color = GREEN;}
	delay_ms(500);
	//----------key-----------------//
	if(System_ini_state.key_ini_state) GUI_ShowString(10,96,12,"Key is OK...",0);
	else {Point_color = RED;GUI_ShowString(10,96,12,"Key is error...",0);Point_color = GREEN;}
	delay_ms(500);
	//----------adc-----------------//
	if(System_ini_state.system_timer_ini_state) GUI_ShowString(10,108,12,"System timer is OK..",0);
	else {Point_color = RED;GUI_ShowString(10,108,12,"System timer error..",0);}
	delay_ms(500);

	Lcd_Init_Clear(Back_color);
}



