/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/power.h
  * @author  Fishcan, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   ADC电压采样
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

#ifndef __POWER_H
#define __POWER_H

#include "stm32f10x.h"
#include "spiderx_bsp.h"

/* Global definition **********************************************************/

#define BATTERY_AD_VOL_CHANNEL     10      /*!< 电池采样通道                  */ 
#define BATTERY_AD_CUR_CHANNEL     11      /*!< 电池采样通道                  */
#define CHANNEL_NUM                2       /*!< ADC采样双通道                 */

#define BAT_VOL_CHANNEL            0       /*!< ADC voltage channel of buffer */
#define BAT_CUR_CHANNEL            1       /*!< ADC voltage channel of buffer */

/**
 *@brief  过流/过压保护值
 */

#ifdef BATTERY_7V4									   
    #define POWER_WARN_VOLTAGE               2358   /*!< 7.6V, 对应AD值       */
    #define POWER_LOW_VOLTAGE                2234   /*!< 7.2V, 对应AD值       */
#else
	#define POWER_WARN_VOLTAGE		 	     3537   /*!< 11.4V AD值           */
	#define POWER_LOW_VOLTAGE		   		 3351   /*!< 10.8V AD值           */
#endif

#define POWER_OVER_CURRENT               846    /*!< 15A, 对应AD值1000:17.74A */
#define POWER_MAX_SAMPLE_SIZE            5      /*!< 电压/电流缓冲区大小      */

/**
 *@brief  中断优先级
 */

#if defined ( __CC_ARM   )                 /* MDK */
#define ADC_TIMx_IRQHandler                TIM6_IRQHandler
#else                                      /* IAR */
#define ADC_TIMx_IRQHandler                TIM6_DAC_IRQHandler
#endif 

#define ADC_TIMx                           TIM6
#define ADC_TIMx_IRQn                      TIM6_IRQn
#define ADC_RCC_TIMx                       RCC_APB1Periph_TIM6
#define OVERCURRENT_EXTI_LINE              EXTI_Line5
#define OVERCURRENT_EXTI_IRQn              EXTI9_5_IRQn

#define OVERCURRENT_EXTIx_IRQHandler       EXTI9_5_IRQHandler

#define __OVER_CURRENT_EXTI_RCC_INIT__()   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |\
                                                                  RCC_APB2Periph_AFIO, ENABLE)
#define OVERCURRENT_EXTI_PORT              GPIOB
#define OVERCURRENT_EXTI_PIN               GPIO_Pin_5
#define OVERCURRENT_EXTI_PORTSOURCE        GPIO_PortSourceGPIOB
#define OVERCURRENT_EXTI_PINSOURCE         GPIO_PinSource5

extern u16 current;                                               /*!< 当前电流采样值                */
extern u16 voltage;                                               /*!< 当前电压采样值                */
extern Boolean overCurrentFlag;                                   /*!< 过流标记                      */

/* Public functions ***********************************************************/

void Adc_Init(void);
void Adc_DMA_Init(void);
u16  Get_Adc(u8 ch); 
u16  Get_Adc_Average(u8 ch,u8 times); 
u8 	 Get_AD_Percentage(u16 adc_val);
u16  AD_get_avg(u16* buff, u8 size) ;
void AD_add_queue(u16* buff,u16 ad,u8 len);
u8	 Get_Bat_Percentage_Ad(void);
u16	 Get_Bat_Ad(void);
void Adc_Timer_Init(u16 arr, u16 psc);
void Adc_Moving_Avg_Filtering(void);
void Adc_Voltage_Sampling(void);
void Adc_CurrentOverStopInit(void);

#endif /* __POWER_H */
