 #include "adc.h"
 #include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V01 
//Battery AD value read and Joystick AD value turn to percentage   Data 20150808 By fishcan
////////////////////////////////////////////////////////////////////////////////// 

u16 Adc_converted_value[6];
u16 Val_ladx,Val_lady,Val_radx,Val_rady;				//采样AD值
u16 Bat_val,Photoresistor_val;
u8 Pre_ladx,Pre_lady,Pre_radx,Pre_rady;				//采样百分比
u8 Pre_bat,Pre_photoresistor;
static u8 adc_flag_48ms = 0;

//extern static u8 key_type;

void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE );	  //使能ADC1通道时钟
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA0 -PA5: IN0-IN5; 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;				//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			  			//多信道扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;					//模数转换工作在连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 6;								//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);								//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

//ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
       ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );                
       ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 ); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 ); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5 ); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_239Cycles5 ); 
	
  	ADC_DMACmd(ADC1, ENABLE);       									//使能ADC1的DMA传输      
	ADC_Cmd(ADC1, ENABLE);											//使能指定的ADC1	
	ADC_ResetCalibration(ADC1);										//使能复位校准  	 
	while(ADC_GetResetCalibrationStatus(ADC1));						//等待复位校准结束	
	ADC_StartCalibration(ADC1);								  		//开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));		 					 //等待校准结束

	DMA_Cmd(DMA1_Channel1, ENABLE);     								 //启动DMA通道
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 					 //软件启动AD转换

	System_ini_state.adc_ini_state =DONE;
}

//初始化ADC1 DMA功能
void Adc_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure; 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						// 使能DMA1时钟	DMA_DeInit(DMA1_Channel1);												// 复位DMA1 通道1

	DMA_DeInit(DMA1_Channel1);  												 //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_Address;				// 设置ADC1基址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Adc_converted_value;  		// DMA1内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  						// 数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = 6;  										// DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 				// 外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					// 内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  	// ADC数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 		// 缓存数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  							// 工作在循环缓存模式，传输完成后又从初始化位置执行
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 							// DMA通道 x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 								// DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  		

	System_ini_state.dma_ini_state =DONE;
}		   

//-----------------------------
//获得ADC值
//采样异常返回0，正常返回1
//-----------------------------
u8 Get_Adc(void)
{
	Bat_val   = 	Adc_converted_value[BATTERY_AD_CHANNEL];
	Val_ladx =	Adc_converted_value[JOYKIT_LEFT_X_AD_CHANNEL];
	Val_lady =	Adc_converted_value[JOYKIT_LEFT_Y_AD_CHANNEL];
	Photoresistor_val =	Adc_converted_value[PHOTORESISTOR_AD_CHANNEL];
	Val_radx =	Adc_converted_value[JOYKIT_RIGHT_X_AD_CHANNEL];
	Val_rady =	Adc_converted_value[JOYKIT_RIGHT_Y_AD_CHANNEL];
//-----------------debug---------------
//	Bat_val = 2009;
//-------------------------------------
	if((Bat_val <=BATTERY_UNDER_VALTAGE) || (Bat_val >=BATTERY_FULL_VALTAGE))
	{
		System_fault_flag = SYSTEM_VOLTAGE_FAULT;
		//System_state_flag = SYSTEM_FAULT;
		return 0;
	}
	else {return 1;}
}

//获取AD对应百分比
u8 Get_Percentage_Ad(u16 adc)
{
  	u32 percentage;	
	
	percentage = adc * 25/1024;
	
	return percentage;
}

//获取光敏电阻AD对应百分比
u8 Get_Photoresistor_Percentage_Ad(u16 adc)
{
  	u32 percentage;	
	
	percentage = adc * 25 / 1024;
	return percentage;
}


//返回所有AD对应百分比
void Get_all_percentage_ad(void)
{
	u32 bat_voltage;

	if(adc_flag_48ms)
	{
		adc_flag_48ms = 0;			//清除定时标记
		
		if(Get_Adc() == 0)
		{
			//LCD_BL_PWM_duty_set(0);
			return;
		}
		Pre_ladx = Get_Percentage_Ad(Val_ladx);  		  	
		Pre_lady = Get_Percentage_Ad(Val_lady);					
		Pre_radx = Get_Percentage_Ad(Val_radx);				
		Pre_rady = Get_Percentage_Ad(Val_rady);	
//------------------Battery电量-------------------------//
		if(Bat_val < BATTERY_UNDER_VALTAGE)
			{Pre_bat = 0;}
		else if(Bat_val > BATTERY_FULL_VALTAGE)
			{Pre_bat = 100;}
		else
			{Pre_bat = (Bat_val - BATTERY_UNDER_VALTAGE) * 100 / (BATTERY_FULL_VALTAGE - BATTERY_UNDER_VALTAGE);}
//------------------光线强度百分比-----------------------//	
		Pre_photoresistor = Get_Photoresistor_Percentage_Ad(Photoresistor_val);
//-------------------Battery precentage-----------------//
//-------------------LCD 1.44 inch--------------------//
#ifdef	LCD_144INCH
		GUI_ShowNum(90,24,Pre_bat,2,12);
		GUI_ShowString(108,24,12,"%",0);
#else
//-------------------LCD nokia5110--------------------//
		//LCD_write_number(60,	1, Pre_bat, 3);    		//X:78-6*3=,取电压整数位
		//LCD_write_String(78,	1, "% ");			 		//y:1
#endif
		
//-------------------Battery voltage--------------------//
		bat_voltage = (Bat_val * 660) / 4096;			 	//放大100倍
//-------------------LCD 1.44 inch---------------------//
#ifdef	LCD_144INCH
		GUI_ShowNum(78,36,bat_voltage/100,1,12);
		GUI_ShowString(84,36,12,".",0);
		GUI_ShowNum(90,36,bat_voltage - (bat_voltage/100)*100,2,12);
		GUI_ShowString(108,36,12,"V",0);
#else
//-------------------LCD nokia5110--------------------//
		//LCD_write_String(1,	3, "Voltage:");			 	//y:2
		//LCD_write_number(48,	3, bat_voltage/100, 1);    //X:8*6,取电压整数位
		//LCD_write_String(54,	3, ".");			 		//x:48+6
		//LCD_write_number(60,	3, bat_voltage - (bat_voltage/100)*100 , 2);   //X:54+6,取电压小数部分
		//LCD_write_String(72,	3, "V");			 		//x:60+12
#endif


		if(System_key_type.new_key_type == KEY_PRESS_LEFT)
		{
//---------------------Left joykit----------------------//
#ifdef	LCD_144INCH
			GUI_ShowString(10,70,12,"L-X:",0);				//12*6 size
			GUI_ShowNum(34,70,Pre_ladx,2,12);
			GUI_ShowString(58,70,12,"R-X:",0);
			GUI_ShowNum(82,70,Pre_radx,2,12);
#else
//-------------------LCD nokia5110--------------------//
			//LCD_write_String(1, 	4, "L-X:");		 			//y:4
			//LCD_write_number(24,	4,Pre_ladx, 2);  			//X:6*4
			//LCD_write_String(42, 	4, "R-X:");		 			//x:6*6+6
			//LCD_write_number(66, 	4, Pre_radx, 2); 			//x:6*4
#endif
		
//---------------------Right joykit---------------------//
//-------------------LCD 1.44 inch---------------------//
#ifdef	LCD_144INCH
			GUI_ShowString(10,82,12,"L-Y:",0);				//12*6 size
			GUI_ShowNum(34,82,Pre_lady,2,12);
			GUI_ShowString(58,82,12,"R-Y:",0);
			GUI_ShowNum(82,82,Pre_rady,2,12);
#else
//-------------------LCD nokia5110--------------------//
			//LCD_write_String(1, 	5, "L-Y:");		 			//y:5  总列6*8=48
			//LCD_write_number(24,	5,Pre_lady, 2);  			//X:6*4
			//LCD_write_String(42, 	5, "R-Y:");		 			//x:6*6+6
			//LCD_write_number(66, 	5, Pre_rady, 2); 			//x:6*4
#endif
		}
	}
}


//--------------------------------------
//系统ADC采样处理定时器 48ms
//--------------------------------------
void Adc_timer(void)
{
	static u8 tick_4ms = 0;

	tick_4ms++;
	
	if (tick_4ms >= 12)			// 4*12 = 48ms
	{	
		tick_4ms=0;
		adc_flag_48ms = 1;
	}
}

