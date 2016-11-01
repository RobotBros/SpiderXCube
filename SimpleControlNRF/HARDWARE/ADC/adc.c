 #include "adc.h"
 #include "delay.h"
  #include "stm32f10x_dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V01 
//Battery AD value read and Joystick AD value turn to percentage   Data 20150808 By fishcan
////////////////////////////////////////////////////////////////////////////////// 
	   
//��ʼ��ADC
//�������ǽ��Թ���ͨ��Ϊ��
//����Ĭ�Ͻ�����ͨ��10 11	

extern u16 Bat_adc_converted_value[2];				// ����������  0:��ѹ 1:����

void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	| RCC_APB2Periph_AFIO, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

	//PC0 IN10;PC1 IN11; ��Ϊģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;				//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			  			//���ŵ�ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;					//ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 2;								//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);								//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

//ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ239.5����
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );                
        ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5 );

  	ADC_DMACmd(ADC1, ENABLE);       									//ʹ��ADC1��DMA����      
	ADC_Cmd(ADC1, ENABLE);											//ʹ��ָ����ADC1	
	ADC_ResetCalibration(ADC1);										//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));						//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);								  		//����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));		 					 //�ȴ�У׼����

	DMA_Cmd(DMA1_Channel1, ENABLE);     								 //����DMAͨ��
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 					 //�������ADת��
}	

//��ʼ��ADC1 DMA����
void Adc_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure; 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						// ʹ��DMA1ʱ��	DMA_DeInit(DMA1_Channel1);												// ��λDMA1 ͨ��1

	DMA_DeInit(DMA1_Channel1);  												 //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_Address;				// ����ADC1��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Bat_adc_converted_value;  	// DMA1�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  						// ���ݴ��䷽�򣬴������ȡ���͵��ڴ�
	DMA_InitStructure.DMA_BufferSize = 2;  										// DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 				// �����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					// �ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  	// ADC���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 		// �������ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  							// ������ѭ������ģʽ��������ɺ��ִӳ�ʼ��λ��ִ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 							// DMAͨ�� xӵ�и����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 								// DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  			
}

//����ƽ��ֵ�˲� channel:BAT_VOL_CHANNEL or BAT_CUR_CHANNEL
//u16 Adc_recursion_avg(u8 channel)
//{
//	u8 cnt;
//	u32 sum=0;
//
//	for(cnt=0;cnt<SAMPLE_NUM;cnt++)
//		{sum += Bat_adc_converted_value[channel][cnt];}
//	//return (sum /SAMPLE_NUM);								// �����㷨
//	return (sum >> 3);										// ��8������8λ
//}
 
//���ADCֵ  ��ͨ�������β���
//ch:ͨ��ֵ 10 11
u16 Get_Adc(u8 ch)   
{
  //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);			//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);														  //ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));									    //�ȴ�ת������

	return ADC_GetConversionValue(ADC1);													      //�������һ��ADC1�������ת�����
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		//delay_ms(2);
	}
	return temp_val/times;
} 	 

//����ƽ���˲�


//��������ƽ��ֵ
u16 AD_get_avg(u16* buff, u8 size) 
{
	u32 sum = 0;
	u8 cnt;
	
	for(cnt=0; cnt<size; cnt++) 
		{sum += buff[cnt];}
	return sum / size;
}

//���һ���µ�ADֵ������β �����˲�,ad:AD data;len:���г���:
void AD_add_queue(u16* buff,u16 ad,u8 len) 
{
	u8 cnt;
	
	for(cnt=1;cnt<len;cnt++)
		buff[cnt-1] = buff[cnt];
	buff[len-1] = ad;
}


//����AD�ٷֱȣ����ذٷֱ�
u8 Get_AD_Percentage(u16 adc_val)
{
	u8 percentage;
	percentage = adc_val* 100 / 4096 ;
  return 	percentage;
}

//������ص����ٷֱȣ����ذٷֱ���ֵ ��3.3V 100%
u8 Get_Bat_Percentage_Ad(void)
{
	u16 adc_val;
  u8 percentage;	
	adc_val = Get_Bat_Ad();
	percentage = adc_val* 100 / 4096 ;
	return percentage;
}

//�������ADֵ�����ص��ADֵ���������Σ���ƽ��ֵ
u16	Get_Bat_Ad(void)
{
	u16 adc_val;	
	adc_val = Get_Adc_Average(ADC_Channel_10,2); // ����ת��ʱ��33.1us ƽ���������5ms ������2��
	return adc_val;
}

//������ҡ��X��ٷֱȣ����ذٷֱ���ֵ
u8 Get_Left_Joystick_X_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_12,2); // ����ת��ʱ��33.1us ƽ���������5ms ������2��
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}

//������ҡ��Y��ٷֱȣ����ذٷֱ���ֵ
u8 Get_Left_Joystick_Y_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_13,2); // ����ת��ʱ��33.1us ƽ���������5ms ������2��
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}

//������ҡ��X��ٷֱȣ����ذٷֱ���ֵ
u8 Get_Right_Joystick_X_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_14,2); // ����ת��ʱ��33.1us ƽ���������5ms ������2��
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}

//������ҡ��Y��ٷֱȣ����ذٷֱ���ֵ
u8 Get_Right_Joystick_Y_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_15,2); // ����ת��ʱ��33.1us ƽ���������5ms ������2��
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}












