/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/power.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   ADC��ѹ����
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

static u16 currentSampleBuffer[POWER_MAX_SAMPLE_SIZE];     /*<! ��������������                */
static u16 voltageSampleBuffer[POWER_MAX_SAMPLE_SIZE];     /*<! ��ѹ����������                */
static u8  avgFilterPos;                                   /*<! ��ѹ/�����˲���λ������       */

/* Global variables ***********************************************************/

u16 current;                                               /*<! ��ǰ��������ֵ                */
u16 voltage;                                               /*<! ��ǰ��ѹ����ֵ                */
Boolean overCurrentFlag = FALSE;                           /*<! �������                      */

/*
 *@brief  ��ʼ��ADC
 *        �������ǽ��Թ���ͨ��Ϊ��, ����Ĭ�Ͻ�����ͨ��10 11
 */
u16 adcConvertedValues[2];                            /*<! ADCֵ DMA������ 0:��ѹ 1:���� */

/**
 *@brief  ��ʼ��ADC1
 *@param  None
 *@reval  None
 */
void  Adc_Init(void)
{     
    ADC_InitTypeDef ADC_InitStructure; 
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE );//ʹ��ADC1ͨ��ʱ��
 
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M

    //PC0 IN10;PC1 IN11; ��Ϊģ��ͨ����������                         
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;        //ģ����������
    GPIO_Init(GPIOC, &GPIO_InitStructure);    

    ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;        //���ŵ�ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //ģ��ת������������ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //ת��������������ⲿ��������
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;               //ADC�����Ҷ���
    ADC_InitStructure.ADC_NbrOfChannel = 2;                              //˳����й���ת����ADCͨ������Ŀ
    ADC_Init(ADC1, &ADC_InitStructure); //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

    //ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ239.5����
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );                
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5 );

    ADC_DMACmd(ADC1, ENABLE);                                         //ʹ��ADC1��DMA����      
    ADC_Cmd(ADC1, ENABLE);                                            //ʹ��ָ����ADC1    
    ADC_ResetCalibration(ADC1);                                       //ʹ�ܸ�λУ׼       
    while(ADC_GetResetCalibrationStatus(ADC1));                       //�ȴ���λУ׼����    
    ADC_StartCalibration(ADC1);                                       //����ADУ׼
    while(ADC_GetCalibrationStatus(ADC1));                            //�ȴ�У׼����

    DMA_Cmd(DMA1_Channel1, ENABLE);                                   //����DMAͨ��
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);                           //�������ADת��
}    

/**
 *@brief  ��ʼ��ADC1 DMA����
 *@param  None
 *@reval  None
 */
void Adc_DMA_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure; 

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // ʹ��DMA1ʱ��    DMA_DeInit(DMA1_Channel1);                                                // ��λDMA1 ͨ��1

    DMA_DeInit(DMA1_Channel1); //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_Address; // ����ADC1��ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&adcConvertedValues; // DMA1�ڴ����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;               // ���ݴ��䷽�򣬴������ȡ���͵��ڴ�
    DMA_InitStructure.DMA_BufferSize = 2;                            // DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // �����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;          // �ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ADC���ݿ��Ϊ16λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // �������ݿ��Ϊ16λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   // ������ѭ������ģʽ��������ɺ��ִӳ�ʼ��λ��ִ��
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;     // DMAͨ�� xӵ��������ȼ�
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                // DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);              
}

/**
 *@brief  ��ѹ����ƽ���˲� 
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
 *@brief  ���ADCֵ  ��ͨ�������β���
 *@param  ch:ͨ��ֵ 10 11
 *@retval AD value
 */
u16 Get_Adc(u8 ch)   
{
    //����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);//ADC1,ADCͨ��,����ʱ��Ϊ239.5����                      
  
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);//ʹ��ָ����ADC1�����ת����������    
     
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

    return ADC_GetConversionValue(ADC1);//�������һ��ADC1�������ת�����
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
 *@brief  ��������ƽ��ֵ
 *@param  buff: ADC ������
 *@param  size: ��������С
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
 *@brief  ���һ���µ�ADֵ������β �����˲�
 *@param  ad:AD data;
 *@param  len:���г���
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
 *@brief  ����AD�ٷֱȣ����ذٷֱ�
 *@param  adc_val: ADCֵ
 *@retval None
 */
u8 Get_AD_Percentage(u16 adc_val)
{
    u8 percentage;
    percentage = adc_val* 100 / 4096 ;
    return percentage;
}

/*
 *@brief  �������ADֵ�����ص��ADֵ���������Σ���ƽ��ֵ
 *@param  None
 *@retval ADC value
 */
u16	Get_Bat_Ad(void)
{
	u16 adc_val;	
	adc_val = Get_Adc_Average(ADC_Channel_10,2); // ����ת��ʱ��33.1us ƽ���������5ms ������2��
	return adc_val;
}

/**
 *@brief  ������ص����ٷֱ�.
 *@param   None
 *@retval  ���ذٷֱ���ֵ ��3.3V 100%
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
 *@brief  TIMx ͨ�ö�ʱ����ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 *@reval  None
 */
void Adc_Timer_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(ADC_RCC_TIMx, ENABLE); 
    
    /* ����ʱ��:TDTS = Tck_tim  72MHZ */
    TIM_TimeBaseStructure.TIM_Period = arr; 
    TIM_TimeBaseStructure.TIM_Prescaler = psc; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ADC_TIMx, &TIM_TimeBaseStructure); 
 
    /* ʹ���ж� */
    TIM_ITConfig(ADC_TIMx,TIM_IT_Update,ENABLE );

    /* NVIC�ж����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannel = ADC_TIMx_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_ADC_Sample_Priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);          

    TIM_Cmd(ADC_TIMx, ENABLE);
}

/**
 *@brief  ��ѹ���
 *@param  None
 *@retval None
 */
void Adc_Voltage_Sampling(void)
{   
    /* ��ѹ����ƽ��ֵ�˲� */
    Adc_Moving_Avg_Filtering();
}


/**
 *@brief  �ⲿ�ж�4��ʼ��,Peak current protection (GPIOB.5)
 *@param  None
 *@retval None
 */
void Adc_CurrentOverStopInit(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

    /* ʹ��PB,AF�˿�ʱ�� */
    __OVER_CURRENT_EXTI_RCC_INIT__();
    
	
	GPIO_InitStructure.GPIO_Pin  = OVERCURRENT_EXTI_PIN;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //PB5-IRQ ����  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(OVERCURRENT_EXTI_PORT, &GPIO_InitStructure);

	GPIO_SetBits(OVERCURRENT_EXTI_PORT, OVERCURRENT_EXTI_PIN); //current kill IRQ����

    /*---------------- EXTIX init --------------------*/
	/* GPIOB.5 �ж����Լ��жϳ�ʼ������   �½��ش��� */
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
