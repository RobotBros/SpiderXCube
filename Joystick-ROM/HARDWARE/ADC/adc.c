 #include "adc.h"
 #include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V01 
//Battery AD value read and Joystick AD value turn to percentage   Data 20150808 By fishcan
////////////////////////////////////////////////////////////////////////////////// 

u16 Adc_converted_value[6];
u16 Val_ladx,Val_lady,Val_radx,Val_rady;				//����ADֵ
u16 Bat_val,Photoresistor_val;
u8 Pre_ladx,Pre_lady,Pre_radx,Pre_rady;				//�����ٷֱ�
u8 Pre_bat,Pre_photoresistor;
static u8 adc_flag_48ms = 0;

//extern static u8 key_type;

void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PA0 -PA5: IN0-IN5; ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;				//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			  			//���ŵ�ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;					//ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 6;								//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);								//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

//ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ239.5����
       ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );                
       ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 ); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 ); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5 ); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_239Cycles5 ); 
	
  	ADC_DMACmd(ADC1, ENABLE);       									//ʹ��ADC1��DMA����      
	ADC_Cmd(ADC1, ENABLE);											//ʹ��ָ����ADC1	
	ADC_ResetCalibration(ADC1);										//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));						//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);								  		//����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));		 					 //�ȴ�У׼����

	DMA_Cmd(DMA1_Channel1, ENABLE);     								 //����DMAͨ��
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 					 //�������ADת��

	System_ini_state.adc_ini_state =DONE;
}

//��ʼ��ADC1 DMA����
void Adc_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure; 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						// ʹ��DMA1ʱ��	DMA_DeInit(DMA1_Channel1);												// ��λDMA1 ͨ��1

	DMA_DeInit(DMA1_Channel1);  												 //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_Address;				// ����ADC1��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Adc_converted_value;  		// DMA1�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  						// ���ݴ��䷽�򣬴������ȡ���͵��ڴ�
	DMA_InitStructure.DMA_BufferSize = 6;  										// DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 				// �����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					// �ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  	// ADC���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 		// �������ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  							// ������ѭ������ģʽ��������ɺ��ִӳ�ʼ��λ��ִ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 							// DMAͨ�� xӵ�и����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 								// DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  		

	System_ini_state.dma_ini_state =DONE;
}		   

//-----------------------------
//���ADCֵ
//�����쳣����0����������1
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

//��ȡAD��Ӧ�ٷֱ�
u8 Get_Percentage_Ad(u16 adc)
{
  	u32 percentage;	
	
	percentage = adc * 25/1024;
	
	return percentage;
}

//��ȡ��������AD��Ӧ�ٷֱ�
u8 Get_Photoresistor_Percentage_Ad(u16 adc)
{
  	u32 percentage;	
	
	percentage = adc * 25 / 1024;
	return percentage;
}


//��������AD��Ӧ�ٷֱ�
void Get_all_percentage_ad(void)
{
	u32 bat_voltage;

	if(adc_flag_48ms)
	{
		adc_flag_48ms = 0;			//�����ʱ���
		
		if(Get_Adc() == 0)
		{
			//LCD_BL_PWM_duty_set(0);
			return;
		}
		Pre_ladx = Get_Percentage_Ad(Val_ladx);  		  	
		Pre_lady = Get_Percentage_Ad(Val_lady);					
		Pre_radx = Get_Percentage_Ad(Val_radx);				
		Pre_rady = Get_Percentage_Ad(Val_rady);	
//------------------Battery����-------------------------//
		if(Bat_val < BATTERY_UNDER_VALTAGE)
			{Pre_bat = 0;}
		else if(Bat_val > BATTERY_FULL_VALTAGE)
			{Pre_bat = 100;}
		else
			{Pre_bat = (Bat_val - BATTERY_UNDER_VALTAGE) * 100 / (BATTERY_FULL_VALTAGE - BATTERY_UNDER_VALTAGE);}
//------------------����ǿ�Ȱٷֱ�-----------------------//	
		Pre_photoresistor = Get_Photoresistor_Percentage_Ad(Photoresistor_val);
//-------------------Battery precentage-----------------//
//-------------------LCD 1.44 inch--------------------//
#ifdef	LCD_144INCH
		GUI_ShowNum(90,24,Pre_bat,2,12);
		GUI_ShowString(108,24,12,"%",0);
#else
//-------------------LCD nokia5110--------------------//
		//LCD_write_number(60,	1, Pre_bat, 3);    		//X:78-6*3=,ȡ��ѹ����λ
		//LCD_write_String(78,	1, "% ");			 		//y:1
#endif
		
//-------------------Battery voltage--------------------//
		bat_voltage = (Bat_val * 660) / 4096;			 	//�Ŵ�100��
//-------------------LCD 1.44 inch---------------------//
#ifdef	LCD_144INCH
		GUI_ShowNum(78,36,bat_voltage/100,1,12);
		GUI_ShowString(84,36,12,".",0);
		GUI_ShowNum(90,36,bat_voltage - (bat_voltage/100)*100,2,12);
		GUI_ShowString(108,36,12,"V",0);
#else
//-------------------LCD nokia5110--------------------//
		//LCD_write_String(1,	3, "Voltage:");			 	//y:2
		//LCD_write_number(48,	3, bat_voltage/100, 1);    //X:8*6,ȡ��ѹ����λ
		//LCD_write_String(54,	3, ".");			 		//x:48+6
		//LCD_write_number(60,	3, bat_voltage - (bat_voltage/100)*100 , 2);   //X:54+6,ȡ��ѹС������
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
			//LCD_write_String(1, 	5, "L-Y:");		 			//y:5  ����6*8=48
			//LCD_write_number(24,	5,Pre_lady, 2);  			//X:6*4
			//LCD_write_String(42, 	5, "R-Y:");		 			//x:6*6+6
			//LCD_write_number(66, 	5, Pre_rady, 2); 			//x:6*4
#endif
		}
	}
}


//--------------------------------------
//ϵͳADC��������ʱ�� 48ms
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

