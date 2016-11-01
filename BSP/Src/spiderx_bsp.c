/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/spider_bsp.c
  * @author  Fishcan, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   SpiderX���������Ӳ������
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, ROBOTBROS.  SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 Robotbros.Inc </center></h2>
  ******************************************************************************
  */

#include "spiderx_bsp.h"

static __IO u32 uwTick;

Boolean timer_led_enable = FALSE;
u16 timer_led_base_counter = 0;
u16 timer_led_reset_counter = 0;

Boolean timer_beep_enable = FALSE;
u16 timer_beep_base_counter = 0;
u16 timer_beep_reset_counter = 0;

/**
  * @brief ��������������systickϵͳʱ�ӣ��ж����ȼ����û�ָ��
  * @note Ĭ�������£�systick�������ж�Ϊ1ms
  * @note  ����������Ϊ __weak���û�������Ӧ�ò����¶��帲�Ǹú���
  * @param TickPriority: Tick interrupt priority.
  * @param TickSubPriority: Tick interrupt sub-priority.
  * @retval None
  */
__weak void HAL_InitTick(u32 TickPriority, u32 TickSubPriority)
{ 
	uint32_t prioritygroup = 0x00;	
	
	/* Check the parameters */
  assert_param(IS_NVIC_SUB_PRIORITY(TickPriority));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(TickSubPriority));
  
	/*Configure the SysTick to have interrupt in 1ms time basis*/
  SysTick_Config(HAL_HCLKFreq/1000);
  
  prioritygroup = NVIC_GetPriorityGrouping();
  
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, TickPriority, TickSubPriority));
}

/**
  * @brief ����ȫ�ֱ���uwTick��Ϊϵͳʱ��ֵ
  * @note  Ĭ�������£����ֵ��1ms��Systick����һ��
  * @note  ����������Ϊ __weak���û�������Ӧ�ò����¶��帲�Ǹú���
  * @retval None
  */
__weak void HAL_IncTick(void)
{
  uwTick++;
}

/**
  * @brief ����tickֵ����λΪms.
  * @note  ����������Ϊ __weak���û�������Ӧ�ò����¶��帲�Ǹú���
  * @retval tickֵ
  */
__weak u32 HAL_GetTick(void)
{
  return uwTick;
}

/**
  * @brief ����������systick timer�ṩ��׼����ʱ(��λms)
  * @note  ����������Ϊ __weak���û�������Ӧ�ò����¶��帲�Ǹú���
  * @param Delay: ��ʱʱ�䣬��λms
  * @retval None
  */
__weak void HAL_Delay(__IO u32 Delay)
{
  u32 tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay)
  {
  }
}

/**
  * @brief ֹͣsystickֵ����.����HAL_SuspendTick��Systick�жϽ�ֹͣ��
  * @note  ����������Ϊ __weak���û�������Ӧ�ò����¶��帲�Ǹú���
  * @retval None
  */
__weak void HAL_SuspendTick(void)
{
  /* Disable SysTick Interrupt */
  CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief �ָ�systickֵ����
  * @note  ����������Ϊ __weak���û�������Ӧ�ò����¶��帲�Ǹú���
  * @retval None
  */
__weak void HAL_ResumeTick(void)
{
  /* Enable SysTick Interrupt */
  SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
}

/**
 *@brief  LED IO��ʼ��(��ʼ��PC13)
 */
void LED_Init(void)
{
//	RCC->APB2ENR |=1 <<4;     //ʹ��PORTCʱ��	 
//	   	 
//	GPIOC->CRH &= 0XFF0FFFFF; 
//	GPIOC->CRH |= 0X00300000; //PC.13 �������   	 
//	GPIOC->ODR |= 1<<13;      //PC.13 �����	
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
 	RCC_APB2PeriphClockCmd(LED0_RCC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED0_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED0_PORT, &GPIO_InitStructure);
    
    GPIO_SetBits(LED0_PORT, LED0_PIN);
}

/**
 *@brief  LED��interval���(ms)�ڷ�תtimes��
 *@param  time:����
 *@param  interval:���
 *@retval None
 */
void LED_flash(u8 times, u16 interval)
{
    u8 cnt;

	LED_Off();

	for( cnt = 0; cnt < times; cnt ++ )
	{
		HAL_Delay(interval);
		LED_Toggle();
	}
}


/**
 *@brief  LED ������1S�ı�һ�ε�״̬������˸���Ϊ2S��
 *@param  time:����
 *@retval None
 */
void LED_flash_short(u8 times)
{
	LED_flash(times, 1000);
}

/**
 *@brief  LED ������0.5S�ı�һ�ε�״̬������˸���Ϊ1S
 *@param  time:����
 *@retval None
 */
void LED_flash_fast(u8 times)
{
	LED_flash(times, 2000);
}

/**
 *@brief  LED����
 *@param  None
 *@retval None
 */
void LED_On(void)
{
    GPIO_WriteBit(LED0_PORT, LED0_PIN, Bit_RESET);
}

/**
 *@brief  LEDϨ��
 *@param  None
 *@retval None
 */
void LED_Off(void)
{
    GPIO_WriteBit(LED0_PORT, LED0_PIN, Bit_SET);
}

/**
 *@brief  LED��ת
 *@param  None
 *@retval None
 */
void LED_Toggle(void)
{
    LED0_PORT->ODR ^= LED0_PIN;
}

/**
 *@brief  ��ʼ��PC14�����(beep IO��ʼ��)
 *@param  None 
 *@retval None
 */
void BEEP_Init(void)
{    
    GPIO_InitTypeDef GPIO_InitStructure;
    
 	RCC_APB2PeriphClockCmd(BEEP_RCC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
    
    GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
}

/**
 *@brief  BEEP��
 *@param  None
 *@retval None
 */
void BEEP_On(void)
{
    GPIO_WriteBit(BEEP_PORT, BEEP_PIN, Bit_RESET);
}

/**
 *@brief  BEEP�ر�
 *@param  None
 *@retval None
 */
void BEEP_Off(void)
{
    GPIO_WriteBit(BEEP_PORT, BEEP_PIN, Bit_SET);
}

/**
 *@brief  BEEP��ת
 *@param  None
 *@retval None
 */
void BEEP_Toggle(void)
{
    BEEP_PORT->ODR ^= BEEP_PIN;
}

/**
 *@brief  ����������
 *@param  time:����
 *@param  ms: ���ʱ�����
 *@retval None
 */
void BEEP_flash(u8 times, u16 ms)
{
	u8 cnt;
	
	for ( cnt = 0; cnt < times; cnt ++ )
	{
		BEEP_Toggle();
		HAL_Delay(ms);
		BEEP_Toggle();
		HAL_Delay(ms);
	}

}

u16 Timer7_base_cnt = 0;    //TIM7ʱ��������
u8 System_tim7_display_flag;

/**
 *@brief  TIM7 ͨ�ö�ʱ����ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 *@reval  None
 */
void TIM7_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ��:TDTS = Tck_tim  72MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_TIM7_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  		

	TIM_Cmd(TIM7, ENABLE);
    
    timer_led_base_counter = 0; //TIM7ʱ��������
    timer_led_reset_counter= TIM7_TIME_1S; //ϵͳ״̬��ʾ��ʱ��־
    
    timer_beep_base_counter = 0; //TIM7ʱ��������
    timer_beep_reset_counter= TIM7_TIME_1S; //ϵͳ״̬��ʾ��ʱ��־
}

/**
 *@brief  TIM7 �ж���� (100ms time base)
 *@param  None
 *@retval None
 */
void TIM7_IRQHandler(void)  
{	
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		/* LED */
        if ( timer_led_enable && timer_led_base_counter == timer_led_reset_counter )
        {
            LED_Toggle();
            timer_led_base_counter = 0;
        }
        
        /* BEEP */
        if ( timer_beep_enable && timer_beep_base_counter == timer_beep_reset_counter )
        {
            BEEP_Toggle();
            timer_beep_base_counter = 0;
        }
        
        if (timer_led_enable)
            timer_led_base_counter ++;
        
        if (timer_beep_enable)
            timer_beep_base_counter ++;
        
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}
