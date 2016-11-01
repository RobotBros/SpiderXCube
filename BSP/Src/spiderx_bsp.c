/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/spider_bsp.c
  * @author  Fishcan, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   SpiderX开发板相关硬件定义
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
  * @brief 本函数用于配置systick系统时钟，中断优先级由用户指定
  * @note 默认配置下，systick产生的中断为1ms
  * @note  本函数声明为 __weak，用户可以在应用层重新定义覆盖该函数
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
  * @brief 增加全局变量uwTick作为系统时钟值
  * @note  默认设置下，这个值是1ms在Systick更新一次
  * @note  本函数声明为 __weak，用户可以在应用层重新定义覆盖该函数
  * @retval None
  */
__weak void HAL_IncTick(void)
{
  uwTick++;
}

/**
  * @brief 返回tick值，单位为ms.
  * @note  本函数声明为 __weak，用户可以在应用层重新定义覆盖该函数
  * @retval tick值
  */
__weak u32 HAL_GetTick(void)
{
  return uwTick;
}

/**
  * @brief 本函数基于systick timer提供精准的延时(单位ms)
  * @note  本函数声明为 __weak，用户可以在应用层重新定义覆盖该函数
  * @param Delay: 延时时间，单位ms
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
  * @brief 停止systick值增加.调用HAL_SuspendTick后，Systick中断将停止。
  * @note  本函数声明为 __weak，用户可以在应用层重新定义覆盖该函数
  * @retval None
  */
__weak void HAL_SuspendTick(void)
{
  /* Disable SysTick Interrupt */
  CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
}

/**
  * @brief 恢复systick值增加
  * @note  本函数声明为 __weak，用户可以在应用层重新定义覆盖该函数
  * @retval None
  */
__weak void HAL_ResumeTick(void)
{
  /* Enable SysTick Interrupt */
  SET_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
}

/**
 *@brief  LED IO初始化(初始化PC13)
 */
void LED_Init(void)
{
//	RCC->APB2ENR |=1 <<4;     //使能PORTC时钟	 
//	   	 
//	GPIOC->CRH &= 0XFF0FFFFF; 
//	GPIOC->CRH |= 0X00300000; //PC.13 推挽输出   	 
//	GPIOC->ODR |= 1<<13;      //PC.13 输出高	
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
 	RCC_APB2PeriphClockCmd(LED0_RCC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = LED0_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED0_PORT, &GPIO_InitStructure);
    
    GPIO_SetBits(LED0_PORT, LED0_PIN);
}

/**
 *@brief  LED在interval间隔(ms)内反转times次
 *@param  time:次数
 *@param  interval:间隔
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
 *@brief  LED 短闪，1S改变一次灯状态，即闪烁间隔为2S，
 *@param  time:次数
 *@retval None
 */
void LED_flash_short(u8 times)
{
	LED_flash(times, 1000);
}

/**
 *@brief  LED 快闪，0.5S改变一次灯状态，即闪烁间隔为1S
 *@param  time:次数
 *@retval None
 */
void LED_flash_fast(u8 times)
{
	LED_flash(times, 2000);
}

/**
 *@brief  LED点亮
 *@param  None
 *@retval None
 */
void LED_On(void)
{
    GPIO_WriteBit(LED0_PORT, LED0_PIN, Bit_RESET);
}

/**
 *@brief  LED熄灭
 *@param  None
 *@retval None
 */
void LED_Off(void)
{
    GPIO_WriteBit(LED0_PORT, LED0_PIN, Bit_SET);
}

/**
 *@brief  LED反转
 *@param  None
 *@retval None
 */
void LED_Toggle(void)
{
    LED0_PORT->ODR ^= LED0_PIN;
}

/**
 *@brief  初始化PC14输出口(beep IO初始化)
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
 *@brief  BEEP打开
 *@param  None
 *@retval None
 */
void BEEP_On(void)
{
    GPIO_WriteBit(BEEP_PORT, BEEP_PIN, Bit_RESET);
}

/**
 *@brief  BEEP关闭
 *@param  None
 *@retval None
 */
void BEEP_Off(void)
{
    GPIO_WriteBit(BEEP_PORT, BEEP_PIN, Bit_SET);
}

/**
 *@brief  BEEP反转
 *@param  None
 *@retval None
 */
void BEEP_Toggle(void)
{
    BEEP_PORT->ODR ^= BEEP_PIN;
}

/**
 *@brief  蜂鸣器发声
 *@param  time:次数
 *@param  ms: 间隔时间毫秒
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

u16 Timer7_base_cnt = 0;    //TIM7时基计数器
u8 System_tim7_display_flag;

/**
 *@brief  TIM7 通用定时器初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 *@reval  None
 */
void TIM7_Int_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //计数时钟:TDTS = Tck_tim  72MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_TIM7_Priority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  		

	TIM_Cmd(TIM7, ENABLE);
    
    timer_led_base_counter = 0; //TIM7时基计数器
    timer_led_reset_counter= TIM7_TIME_1S; //系统状态显示定时标志
    
    timer_beep_base_counter = 0; //TIM7时基计数器
    timer_beep_reset_counter= TIM7_TIME_1S; //系统状态显示定时标志
}

/**
 *@brief  TIM7 中断入口 (100ms time base)
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
