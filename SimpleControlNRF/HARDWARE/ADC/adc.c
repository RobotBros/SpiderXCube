 #include "adc.h"
 #include "delay.h"
  #include "stm32f10x_dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V01 
//Battery AD value read and Joystick AD value turn to percentage   Data 20150808 By fishcan
////////////////////////////////////////////////////////////////////////////////// 
	   
//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道10 11	

extern u16 Bat_adc_converted_value[2];				// 采样缓存器  0:电压 1:电流

void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	| RCC_APB2Periph_AFIO, ENABLE );	  //使能ADC1通道时钟
 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PC0 IN10;PC1 IN11; 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;				//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			  			//多信道扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;					//模数转换工作在连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 2;								//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);								//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

//ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );                
        ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5 );

  	ADC_DMACmd(ADC1, ENABLE);       									//使能ADC1的DMA传输      
	ADC_Cmd(ADC1, ENABLE);											//使能指定的ADC1	
	ADC_ResetCalibration(ADC1);										//使能复位校准  	 
	while(ADC_GetResetCalibrationStatus(ADC1));						//等待复位校准结束	
	ADC_StartCalibration(ADC1);								  		//开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));		 					 //等待校准结束

	DMA_Cmd(DMA1_Channel1, ENABLE);     								 //启动DMA通道
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 					 //软件启动AD转换
}	

//初始化ADC1 DMA功能
void Adc_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure; 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						// 使能DMA1时钟	DMA_DeInit(DMA1_Channel1);												// 复位DMA1 通道1

	DMA_DeInit(DMA1_Channel1);  												 //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_Address;				// 设置ADC1基址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Bat_adc_converted_value;  	// DMA1内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  						// 数据传输方向，从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = 2;  										// DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 				// 外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					// 内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  	// ADC数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 		// 缓存数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  							// 工作在循环缓存模式，传输完成后又从初始化位置执行
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 							// DMA通道 x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 								// DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  			
}

//递推平均值滤波 channel:BAT_VOL_CHANNEL or BAT_CUR_CHANNEL
//u16 Adc_recursion_avg(u8 channel)
//{
//	u8 cnt;
//	u32 sum=0;
//
//	for(cnt=0;cnt<SAMPLE_NUM;cnt++)
//		{sum += Bat_adc_converted_value[channel][cnt];}
//	//return (sum /SAMPLE_NUM);								// 常规算法
//	return (sum >> 3);										// 除8，右移8位
//}
 
//获得ADC值  单通道，单次采样
//ch:通道值 10 11
u16 Get_Adc(u8 ch)   
{
  //设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);			//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);														  //使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));									    //等待转换结束

	return ADC_GetConversionValue(ADC1);													      //返回最近一次ADC1规则组的转换结果
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

//滑动平均滤波


//计算算数平均值
u16 AD_get_avg(u16* buff, u8 size) 
{
	u32 sum = 0;
	u8 cnt;
	
	for(cnt=0; cnt<size; cnt++) 
		{sum += buff[cnt];}
	return sum / size;
}

//添加一个新的AD值到队列尾 进行滤波,ad:AD data;len:队列长度:
void AD_add_queue(u16* buff,u16 ad,u8 len) 
{
	u8 cnt;
	
	for(cnt=1;cnt<len;cnt++)
		buff[cnt-1] = buff[cnt];
	buff[len-1] = ad;
}


//计算AD百分比，返回百分比
u8 Get_AD_Percentage(u16 adc_val)
{
	u8 percentage;
	percentage = adc_val* 100 / 4096 ;
  return 	percentage;
}

//采样电池电量百分比，返回百分比数值 ：3.3V 100%
u8 Get_Bat_Percentage_Ad(void)
{
	u16 adc_val;
  u8 percentage;	
	adc_val = Get_Bat_Ad();
	percentage = adc_val* 100 / 4096 ;
	return percentage;
}

//采样电池AD值，返回电池AD值，采样两次，求平均值
u16	Get_Bat_Ad(void)
{
	u16 adc_val;	
	adc_val = Get_Adc_Average(ADC_Channel_10,2); // 单次转换时间33.1us 平均采样间隔5ms ，采样2次
	return adc_val;
}

//采样左摇杆X轴百分比，返回百分比数值
u8 Get_Left_Joystick_X_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_12,2); // 单次转换时间33.1us 平均采样间隔5ms ，采样2次
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}

//采样左摇杆Y轴百分比，返回百分比数值
u8 Get_Left_Joystick_Y_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_13,2); // 单次转换时间33.1us 平均采样间隔5ms ，采样2次
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}

//采样右摇杆X轴百分比，返回百分比数值
u8 Get_Right_Joystick_X_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_14,2); // 单次转换时间33.1us 平均采样间隔5ms ，采样2次
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}

//采样右摇杆Y轴百分比，返回百分比数值
u8 Get_Right_Joystick_Y_Percentage_Ad(void)
{
	u16 adc_val;
	u8 percentage;
	adc_val = Get_Adc_Average(ADC_Channel_15,2); // 单次转换时间33.1us 平均采样间隔5ms ，采样2次
	percentage = Get_AD_Percentage(adc_val);
	return percentage;
}












