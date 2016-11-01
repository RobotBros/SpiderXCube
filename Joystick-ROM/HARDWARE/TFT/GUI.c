#include "stm32f10x.h"
#include "Lcd_Driver.h"
#include "GUI.h"
#include "delay.h"
#include "font.h"

//******************************************************************
//���ܣ�    GUI���һ����
//���������x:���λ��x����
//        	y:���λ��y����
//			color:Ҫ������ɫ
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************
void GUI_DrawPoint(u16 x,u16 y,u16 color)
{
	Lcd_SetXY(x,y);				//���ù��λ�� 
	LCD_WriteData_16Bit(color); 
}

//******************************************************************
//���ܣ�    ��ָ�������������ɫ
//���������sx:ָ������ʼ��x����
//        	sy:ָ������ʼ��y����
//			ex:ָ�����������x����
//			ey:ָ�����������y����
//        	color:Ҫ������ɫ
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************
void GUI_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{  	

	u16 i,j;			
	u16 width=ex-sx+1; 				//�õ����Ŀ��
	u16 height=ey-sy+1;				//�߶�
	
	Lcd_SetRegion(sx,sy,ex-1,ey-1);	//������ʾ����
	for(i=0;i<height;i++)
	{
		for(j=0;j<width;j++)
		LCD_WriteData_16Bit(color);	//д������ 	 
	}

	Lcd_SetRegion(0,0,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);		//�ָ���������Ϊȫ��
}

//******************************************************************
//���ܣ�    GUI������(���)
//���������(x1,y1),(x2,y2):���εĶԽ�����
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************   
void LCD_DrawFillRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	GUI_Fill(x1,y1,x2,y2,Point_color);
}

//******************************************************************
//���ܣ�    GUI����
//���������x1,y1:�������
//        	x2,y2:�յ����� 
//����ֵ��  ��
//�޸ļ�¼����
//****************************************************************** 
void GUI_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; 					//������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; 				//���õ������� 
	else if(delta_x==0)incx=0;			//��ֱ�� 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;			//ˮƽ�� 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )		//������� 
	{  
		Lcd_DrawPoint(uRow,uCol);		//���� 
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
//���ܣ�    GUI������(�����)
//���������(x1,y1),(x2,y2):���εĶԽ�����
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************  
void GUI_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
	GUI_DrawLine(x1,y1,x2,y1);
	GUI_DrawLine(x1,y1,x1,y2);
	GUI_DrawLine(x1,y2,x2,y2);
	GUI_DrawLine(x2,y1,x2,y2);
}  

//******************************************************************
//���ܣ�    8�Գ��Ի�Բ�㷨(�ڲ�����)
//���������(xc,yc) :Բ��������
// 			(x,y):��������Բ�ĵ�����
//         	c:������ɫ
//����ֵ��  ��
//�޸ļ�¼����
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
//���ܣ�    ��ָ��λ�û�һ��ָ����С��Բ(���)
//���������(xc,yc) :Բ��������
//         	c:������ɫ
//		 	r:Բ�뾶
//		 	fill:����жϱ�־��1-��䣬0-�����
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************  
void GUI_Circle(int xc, int yc,u16 c,int r, int fill)
{
	int x = 0, y = r, yi, d;

	d = 3 - 2 * r;


	if (fill) 
	{
		// �����䣨��ʵ��Բ��
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
		// �������䣨������Բ��
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
//���ܣ�    ��ʾ����Ӣ���ַ�
//���������(x,y):�ַ���ʾλ����ʼ����
//        		fc:ǰ�û�����ɫ
//			bc:������ɫ
//			num:��ֵ��0-94��
//			size:�����С 12 16
//			mode:ģʽ  0,���ģʽ;1,����ģʽ
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************  
void GUI_ShowChar(u16 x,u16 y,u16 fc, u16 bc, u8 num,u8 size,u8 mode)
{  
    	u8 temp;
    	u8 pos,t;
	u16 colortemp=Point_color; 	
		   
	num=num-' ';								//�õ�ƫ�ƺ��ֵ
	Lcd_SetRegion(x,y,x+size/2-1,y+size-1);		//���õ���������ʾ����
	if(!mode)									 //�ǵ��ӷ�ʽ
	{		
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];//����1206����
			else temp=asc2_1608[num][pos];		 //����1608����
			for(t=0;t<size/2;t++)
		    	{                 
		       	 if(temp&0x01)LCD_WriteData_16Bit(fc); 
				 else LCD_WriteData_16Bit(bc); 
				temp>>=1; 				
		    	}			
		}	
	}
	else												 //���ӷ�ʽ
	{
		for(pos=0;pos<size;pos++)
		{
			if(size==12)temp=asc2_1206[num][pos];	//����1206����
			else temp=asc2_1608[num][pos];		 	//����1608����
			for(t=0;t<size/2;t++)
		    	{ 
		    		Point_color = fc;
		       	if(temp&0x01)Lcd_DrawPoint(x+t,y+pos);//��һ����    
		        	temp>>=1; 
		    }
		}
	}
	Point_color=colortemp;	
	Lcd_SetRegion(0,0,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);					//�ָ�����Ϊȫ��    	   	 	  
} 

//******************************************************************
//���ܣ�    ��ʾӢ���ַ���
//���������x,y :�������	 
//			size:�����С
//			*p:�ַ�����ʼ��ַ
//			mode:ģʽ	0,���ģʽ;1,����ģʽ
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************  	  
void GUI_ShowString(u16 x,u16 y,u8 size,u8 *p,u8 mode)
{         
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {   
	if(x>(LCD_X_MAX_PIXEL-1)||y>(LCD_Y_MAX_PIXEL-1))  return;     
        GUI_ShowChar(x,y,Point_color,Back_color,*p,size,mode);
        x+=size/2;
        p++;
    }  
} 

//******************************************************************
//���ܣ�    ��m��n�η�(gui�ڲ�����)
//���������m:����
//	        n:��
//����ֵ��  m��n�η�
//�޸ļ�¼����
//******************************************************************  
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

//******************************************************************
//���ܣ�    ��ʾ�������ֱ���ֵ
//���������x,y :�������	 
//			len :ָ����ʾ���ֵ�λ��
//			size:�����С(12,16)
//			color:��ɫ
//			num:��ֵ(0~4294967295)
//����ֵ��  ��
//�޸ļ�¼����
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
//���ܣ�    ��ʾһ��16λBMPͼ��
//���������x,y :�������
// 			*p :ͼ��������ʼ��ַ
//����ֵ��  ��
//�޸ļ�¼����
//******************************************************************  
void Gui_Drawbmp16(u16 x,u16 y,const unsigned char *p) //��ʾ40*40 QQͼƬ
{
  	int i; 
	unsigned char picH,picL; 
	
	Lcd_SetRegion(x,y,x+40-1,y+40-1);				 //��������
    	for(i=0;i<40*40;i++)
	{	
	 	picL=*(p+i*2);								//���ݵ�λ��ǰ
		picH=*(p+i*2+1);				
		LCD_WriteData_16Bit(picH<<8|picL);  						
	}	
	Lcd_SetRegion(0,0,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);//�ָ���ʾ����Ϊȫ��	
}

//----------------------------------
//ϵͳ����������ʾ
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



