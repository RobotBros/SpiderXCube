/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/power.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   ADC电压采样
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 RobotBros.cn</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of RobotBros.cn nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "power.h"
    
/* Private variables **********************************************************/

static u16 currentSampleBuffer[POWER_MAX_SAMPLE_SIZE];     /*<! 电流采样缓冲区                */
static u16 voltageSampleBuffer[POWER_MAX_SAMPLE_SIZE];     /*<! 电压采样缓冲区                */
static u8  avgFilterPos;                                   /*<! 电压/电流滤波移位计数器       */

/* Global variables ***********************************************************/

u16 current;                                               /*<! 当前电流采样值                */
u16 voltage;                                               /*<! 当前电压采样值                */
Boolean overCurrentFlag = FALSE;                           /*<! 过流标记                      */

/*
 *@brief  初始化ADC
 *        这里我们仅以规则通道为例, 我们默认将开启通道10 11
 */
u16 adcConvertedValues[2];                            /*<! ADC值 DMA缓存器 0:电压 1:电流 */

/**
 *@brief  初始化ADC1
 *@param  None
 *@reval  None
 */
void  Adc_Init(void)
{     
    ADC_InitTypeDef ADC_InitStructure; 
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE );//使能ADC1通道时钟
 
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

    //PC0 IN10;PC1 IN11; 作为模拟通道输入引脚                         
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;        //模拟输入引脚
    GPIO_Init(GPIOC, &GPIO_InitStructure);    

    ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC工作模式:ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;        //多信道扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //模数转换工作在连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //转换由软件而不是外部触发启动
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               //ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 2;                              //顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure); //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

    //ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );                
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5 );

    ADC_DMACmd(ADC1, ENABLE);                                         //使能ADC1的DMA传输      
    ADC_Cmd(ADC1, ENABLE);                                            //使能指定的ADC1    
    ADC_ResetCalibration(ADC1);                                       //使能复位校准       
    while(ADC_GetResetCalibrationStatus(ADC1));                       //等待复位校准结束    
    ADC_StartCalibration(ADC1);                                       //开启AD校准
    while(ADC_GetCalibrationStatus(ADC1));                            //等待校准结束

    DMA_Cmd(DMA1_Channel1, ENABLE);                                   //启动DMA通道
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);                           //软件启动AD转换
}    

/**
 *@brief  初始化ADC1 DMA功能
 *@param  None
 *@reval  None
 */
void Adc_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure; 

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 使能DMA1时钟    DMA_DeInit(DMA1_Channel1);                                                // 复位DMA1 通道1

    DMA_DeInit(DMA1_Channel1); //将DMA的通道1寄存器重设为缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_Address; // 设置ADC1基址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&adcConvertedValues; // DMA1内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;               // 数据传输方向，从外设读取发送到内存
    DMA_InitStructure.DMA_BufferSize = 2;                            // DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;          // 内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ADC数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // 缓存数据宽度为16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   // 工作在循环缓存模式，传输完成后又从初始化位置执行
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;     // DMA通道 x拥有最高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                // DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);              
}

/**
 *@brief  电压滑动平均滤波 
 *@param  None
 *@return None
 */
void Adc_Moving_Avg_Filtering(void)
{
	u8 cnt;
	u32 sum_vol = 0;
	u32 sum_cur = 0;
    
	voltageSampleBuffer[avgFilterPos]  = adcConvertedValues[0];
	currentSampleBuffer[avgFilterPos]  = adcConvertedValues[1];
    
	if ( avgFilterPos == POWER_MAX_SAMPLE_SIZE - 1 )
        avgFilterPos = 0;
	else
        avgFilterPos ++;
	
	for( cnt = 0; cnt < POWER_MAX_SAMPLE_SIZE; cnt ++ )
	{
		sum_vol += voltageSampleBuffer[cnt];
		sum_cur += currentSampleBuffer[cnt];
	}		
    
	voltage = (u16)(sum_vol / POWER_MAX_SAMPLE_SIZE);
	current = (u16)(sum_cur / POWER_MAX_SAMPLE_SIZE);
	
}

/**
 *@brief  获得ADC值  单通道，单次采样
 *@param  ch:通道值 10 11
 *@retval AD value
 */
u16 Get_Adc(u8 ch)   
{
    //设置指定ADC的规则组通道，一个序列，采样时间
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);//ADC1,ADC通道,采样时间为239.5周期                      
  
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//使能指定的ADC1的软件转换启动功能    
     
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

    return ADC_GetConversionValue(ADC1);//返回最近一次ADC1规则组的转换结果
}

u16 Get_Adc_Average(u8 ch, u8 times)
{
    u32 temp_val = 0;
    u8 t;
    for (t = 0; t < times; t++)
    {
        temp_val += Get_Adc(ch);
    }
    
    return temp_val/times;
}      

/**
 *@brief  计算算数平均值
 *@param  buff: ADC 缓冲区
 *@param  size: 缓冲区大小
 *@retval None
 */
u16 AD_get_avg(u16* buff, u8 size) 
{
    u32 sum = 0;
    u8 cnt;
    
    for(cnt=0; cnt<size; cnt++) 
        {sum += buff[cnt];}
    return sum / size;
}

/**
 *@brief  添加一个新的AD值到队列尾 进行滤波
 *@param  ad:AD data;
 *@param  len:队列长度
 *@retval None
 */
void AD_add_queue(u16* buff,u16 ad,u8 len) 
{
    u8 cnt;
    
    for(cnt=1;cnt<len;cnt++)
        buff[cnt-1] = buff[cnt];
    buff[len-1] = ad;
}

/**
 *@brief  计算AD百分比，返回百分比
 *@param  adc_val: ADC值
 *@retval None
 */
u8 Get_AD_Percentage(u16 adc_val)
{
    u8 percentage;
    percentage = adc_val* 100 / 4096 ;
    return percentage;
}

/*
 *@brief  采样电池AD值，返回电池AD值，采样两次，求平均值
 *@param  None
 *@retval ADC value
 */
u16	Get_Bat_Ad(void)
{
	u16 adc_val;	
	adc_val = Get_Adc_Average(ADC_Channel_10,2); // 单次转换时间33.1us 平均采样间隔5ms ，采样2次
	return adc_val;
}

/**
 *@brief  采样电池电量百分比.
 *@param   None
 *@retval  返回百分比数值 ：3.3V 100%
 */
u8 Get_Bat_Percentage_Ad(void)
{
    u16 adc_val;
    u8 percentage;    
    adc_val = Get_Bat_Ad();
    percentage = adc_val* 100 / 4096 ;
    return percentage;
}

/**
 *@brief  TIMx 通用定时器初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 *@reval  None
 */
void Adc_Timer_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(ADC_RCC_TIMx, ENABLE); 
    
    /* 计数时钟:TDTS = Tck_tim  72MHZ */
    TIM_TimeBaseStructure.TIM_Period = arr; 
    TIM_TimeBaseStructure.TIM_Prescaler = psc; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ADC_TIMx, &TIM_TimeBaseStructure); 
 
    /* 使能中断 */
    TIM_ITConfig(ADC_TIMx,TIM_IT_Update,ENABLE );

    /* NVIC中断优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = ADC_TIMx_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_ADC_Sample_Priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);          

    TIM_Cmd(ADC_TIMx, ENABLE);
}

/**
 *@brief  电压检测
 *@param  None
 *@retval None
 */
void Adc_Voltage_Sampling(void)
{   
    /* 电压滑动平均值滤波 */
    Adc_Moving_Avg_Filtering();
}


/**
 *@brief  外部中断4初始化,Peak current protection (GPIOB.5)
 *@param  None
 *@retval None
 */
void Adc_CurrentOverStopInit(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能PB,AF端口时钟 */
    __OVER_CURRENT_EXTI_RCC_INIT__();
    
	
	GPIO_InitStructure.GPIO_Pin  = OVERCURRENT_EXTI_PIN;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //PB5-IRQ 输入  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(OVERCURRENT_EXTI_PORT, &GPIO_InitStructure);

	GPIO_SetBits(OVERCURRENT_EXTI_PORT, OVERCURRENT_EXTI_PIN); //current kill IRQ上拉

    /*---------------- EXTIX init --------------------*/
	/* GPIOB.5 中断线以及中断初始化配置   下降沿触发 */
  	GPIO_EXTILineConfig(OVERCURRENT_EXTI_PORTSOURCE, OVERCURRENT_EXTI_PINSOURCE);

  	EXTI_InitStructure.EXTI_Line = OVERCURRENT_EXTI_LINE;													
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = OVERCURRENT_EXTI_IRQn; 
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_ADC_OverCur_Priority;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}
