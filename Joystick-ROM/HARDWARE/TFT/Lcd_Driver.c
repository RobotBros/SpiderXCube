#include "Lcd_Driver.h"

static u8 lcd_bl_flag_100ms = 0;
//������ɫ,������ɫ
//_lcd_dev lcddev;
u16 Point_color = 0x0000,Back_color = 0xFFFF;  

//LCD TFT 1.44 inch I/O��ʼ��
void Lcd_144inch_IO_Init(u16 arr,u16 psc)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
//LCD   BL	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;						//TIM3_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
//LCD  PC13-RST PC14-CLK 	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 |GPIO_Pin_14;					
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_SetBits(GPIOC,GPIO_Pin_14);
//LCD   PB6-DIN PB7-DC PB8-CE	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7 |GPIO_Pin_8;					
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	GPIO_SetBits(GPIOB,GPIO_Pin_6);
	GPIO_SetBits(GPIOB,GPIO_Pin_7);
	GPIO_SetBits(GPIOB,GPIO_Pin_8);

	   //��ʼ��TIM3��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 							//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 						//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 						//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3  PWMģʽ����	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			//�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  						//����Tָ���Ĳ�����ʼ������TIM3_CH1 

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 			//ʹ��TIM3 CH2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3

}


//��SPI���ߴ���һ��8λ����
void  SPI_WriteData(u8 Data)
{
	unsigned char i=0;
	for(i=8;i>0;i--)
	{
		if(Data&0x80) LCD_SDA = 1; 	//�������
      		else LCD_SDA = 0;
	   
      		LCD_SCL = 0;       
      		LCD_SCL = 1;
      		Data<<=1; 
	}
}

//��Һ����дһ��8λָ��
void Lcd_WriteIndex(u8 Index)
{
   //SPI д����ʱ��ʼ
   	LCD_CS = 0;
   	LCD_RS = 0;
	SPI_WriteData(Index);
  	LCD_CS = 1;
}

//��Һ����дһ��8λ����
void Lcd_WriteData(u8 Data)
{
   	LCD_CS = 0;
   	LCD_RS = 1;
   	SPI_WriteData(Data);
   	LCD_CS = 1; 
}
//��Һ����дһ��16λ����
void LCD_WriteData_16Bit(u16 Data)
{
   	LCD_CS = 0;
   	LCD_RS = 1;
	SPI_WriteData(Data>>8); 	//д���8λ����
	SPI_WriteData(Data); 		//д���8λ����
   	LCD_CS = 1;
}

void Lcd_WriteReg(u8 Index,u8 Data)
{
	Lcd_WriteIndex(Index);
  	Lcd_WriteData(Data);
}

void Lcd_Reset(void)
{
	LCD_RST = 0;
	delay_ms(100);
	LCD_RST = 1;
	delay_ms(50);	
}

//LCD Init For 1.44Inch LCD Panel with ST7735R.
void Lcd_144inch_Init(void)
{	
	Lcd_144inch_IO_Init(LCD_BL_SCALE,LCD_BL_DIV);		// LCD 1.44 I/O BL_PWM��ʼ��	
	Lcd_Reset(); 										//Reset before LCD Init.

	//LCD Init For 1.44Inch LCD Panel with ST7735R.
	Lcd_WriteIndex(0x11);								//Sleep exit 
	delay_ms (120);
		
	//ST7735R Frame Rate
	Lcd_WriteIndex(0xB1); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB2); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB3); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	
	Lcd_WriteIndex(0xB4); //Column inversion 
	Lcd_WriteData(0x07); 
	
	//ST7735R Power Sequence
	Lcd_WriteIndex(0xC0); 
	Lcd_WriteData(0xA2); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x84); 
	Lcd_WriteIndex(0xC1); 
	Lcd_WriteData(0xC5); 

	Lcd_WriteIndex(0xC2); 
	Lcd_WriteData(0x0A); 
	Lcd_WriteData(0x00); 

	Lcd_WriteIndex(0xC3); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0x2A); 
	Lcd_WriteIndex(0xC4); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0xEE); 
	
	Lcd_WriteIndex(0xC5); //VCOM 
	Lcd_WriteData(0x0E); 
	
	Lcd_WriteIndex(0x36); //MX, MY, RGB mode 
	Lcd_WriteData(0xC8); 
	
	//ST7735R Gamma Sequence
	Lcd_WriteIndex(0xe0); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1a); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x18); 
	Lcd_WriteData(0x2f); 
	Lcd_WriteData(0x28); 
	Lcd_WriteData(0x20); 
	Lcd_WriteData(0x22); 
	Lcd_WriteData(0x1f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x23); 
	Lcd_WriteData(0x37); 
	Lcd_WriteData(0x00); 	
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x10); 

	Lcd_WriteIndex(0xe1); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x17); 
	Lcd_WriteData(0x33); 
	Lcd_WriteData(0x2c); 
	Lcd_WriteData(0x29); 
	Lcd_WriteData(0x2e); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x39); 
	Lcd_WriteData(0x3f); 
	Lcd_WriteData(0x00); 
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x03); 
	Lcd_WriteData(0x10);  
	
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x7f);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x9f);
	
	Lcd_WriteIndex(0xF0); //Enable test command  
	Lcd_WriteData(0x01); 
	Lcd_WriteIndex(0xF6); //Disable ram power save mode 
	Lcd_WriteData(0x00); 
	
	Lcd_WriteIndex(0x3A); //65k mode 
	Lcd_WriteData(0x05); 
	
	
	Lcd_WriteIndex(0x29);//Display on	

	System_ini_state.lcd_ini_state =DONE;
}


/*************************************************
��������LCD_Set_Region
���ܣ�����lcd��ʾ�����ڴ�����д�������Զ�����
��ڲ�����xy�����յ�
����ֵ����
*************************************************/
void Lcd_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end)
{		
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_start+2);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_end+2);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_start+3);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_end+3);
	
	Lcd_WriteIndex(0x2c);

}

/*************************************************
��������LCD_Set_XY
���ܣ�����lcd��ʾ��ʼ��
��ڲ�����xy����
����ֵ����
*************************************************/
void Lcd_SetXY(u16 x,u16 y)
{
  	Lcd_SetRegion(x,y,x,y);
}

	
/*************************************************
��������LCD_DrawPoint
���ܣ���һ����
��ڲ�������
����ֵ����
*************************************************/
void Lcd_DrawPoint(u16 x,u16 y)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteData_16Bit(Point_color);
}    

/*****************************************
 �������ܣ���TFTĳһ�����ɫ                          
 ���ڲ�����color  ����ɫֵ                                 
******************************************/
unsigned int Lcd_ReadPoint(u16 x,u16 y)
{
  unsigned int Data;
  Lcd_SetXY(x,y);

  //Lcd_ReadData();//���������ֽ�
  //Data=Lcd_ReadData();
  Lcd_WriteData(Data);
  return Data;
}

/*************************************************
��������Lcd_Clear
���ܣ�ȫ����������
��ڲ����������ɫCOLOR,x,y��ʼ����
����ֵ����
*************************************************/
void Lcd_Clear(u16 Color,u8 x,u8 y)               
{	
   unsigned int i,m;
   Lcd_SetRegion(x,y,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<LCD_X_MAX_PIXEL;i++)
    for(m=0;m<LCD_Y_MAX_PIXEL;m++)
    {	
	  	LCD_WriteData_16Bit(Color);
    }   
}

/*************************************************
��������Lcd_init_Clear
���ܣ���ʼ������ȥ��������������			"Spider_System"   Blue
��ڲ����������ɫCOLOR				"Starting..."		Red
����ֵ����
*************************************************/
void Lcd_Init_Clear(u16 Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,25,LCD_X_MAX_PIXEL-1,LCD_Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<LCD_X_MAX_PIXEL;i++)
    for(m=0;m<LCD_Y_MAX_PIXEL;m++)
    {	
	  	LCD_WriteData_16Bit(Color);
    }   
}

// LCD BL ռ�ձ�����
void LCD_BL_PWM_duty_set(u8 duty)
{
	u16 ccr;

	ccr = ((LCD_BL_SCALE + 1)  * duty) / 100;

	TIM_SetCompare1(TIM3,ccr);
}



//----------------------------------
//ϵͳ������ʾ
//-----------------------------------
void LCD_fault_display(u8 fault)
{	
	if(fault == SYSTEM_NRF_NULL)
	{
		LCD_BL_PWM_duty_set(40);
		Point_color = RED;
		Back_color = BLACK;
		Lcd_Clear(Back_color,0,0);	
		GUI_ShowString(20,0,12,"Spider_System",0);
		Point_color = GREEN;
		GUI_ShowString(0,24,12,"NRF is null!",0);
	}
	else if(fault == SYSTEM_VOLTAGE_FAULT)
	{
		LCD_BL_PWM_duty_set(40);
		Point_color = RED;
		Back_color = BLACK;
		Lcd_Clear(Back_color,0,0);	
		GUI_ShowString(20,0,12,"Spider_System",0);
		GUI_ShowString(0,24,12,"Bat error!",0);
	}
}

//------------------------------
//����LCD���ⶨʱ�� 100ms
//------------------------------
void LCD_backlight_set_timer(void)
{
	static u8 tick_4ms = 0;

	tick_4ms++;
	
	if (tick_4ms >= 25)			// 4*25 = 100ms
	{	
		tick_4ms=0;
		lcd_bl_flag_100ms = 1;
	}

}

//---------------------------------
//LCD�����Զ�����
//---------------------------------
void LCD_backlight_set(void)
{
	if(lcd_bl_flag_100ms)
	{
		lcd_bl_flag_100ms = 0;
		
		if(Pre_photoresistor < 6)				//100����ٷֱ���С��ռ�ձ���С		
			{LCD_BL_PWM_duty_set(20);}
		else if(Pre_photoresistor > 90)		//0��⣬�ٷֱ����ռ�ձ����
			{LCD_BL_PWM_duty_set(80);}
		else
			{LCD_BL_PWM_duty_set(Pre_photoresistor);}
	}
}

