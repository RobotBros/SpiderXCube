/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/spider_bsp.h
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

#ifndef __SPIDER_BSP_H
#define __SPIDER_BSP_H 

#include "SpiderNANOConfig.h"
#include "stm32f10x.h"

typedef enum Boolean 
{
    FALSE = 0,
    TRUE = 1
} Boolean;

/** 
 *@brief  IO口操作宏定义 
 */
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

/**
 *@brief  IO口地址映射
 */
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
/**
 *@brief  IO口操作,只对单一的IO口!
 */
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入


#define HAL_HCLKFreq                        72000000   /* 系统时钟频率        */

/* ADC Related */
#define ADC1_DR_Address                     ((u32)0x4001244C)

/* LED端口定义 */
#define LED0 PCout(13)
#define LED0_RCC                            RCC_APB2Periph_GPIOC
#define LED0_PORT                           GPIOC
#define LED0_PIN                            GPIO_Pin_13

#define ON		0
#define OFF		1

/**
 *@brief  beep端口定义
 */
#define BEEP                                PCout(14)
#define BEEP_RCC                            RCC_APB2Periph_GPIOC
#define BEEP_PORT                           GPIOC
#define BEEP_PIN                            GPIO_Pin_14

/* IRQ Priority */
#define IRQ_ADC_OverCur_Priority           1
#define IRQ_ADC_Sample_Priority            2
#define IRQ_BT_USARTx_Priority             3
#define IRQ_24G_Priority                   4
#define IRQ_TIM7_Priority                  5
#define IRQ_SYSTICK_Priority               8

typedef enum _TIM7RestTime
{
    TIM7_TIME_100MS = 0x01,
    TIM7_TIME_200MS = 0x02,
    TIM7_TIME_300MS = 0x03,
    TIM7_TIME_400MS = 0x04,
    TIM7_TIME_500MS = 0x05,
    TIM7_TIME_600MS = 0x06,
    TIM7_TIME_700MS = 0x07,
    TIM7_TIME_800MS = 0x08,
    TIM7_TIME_900MS = 0x09,
    TIM7_TIME_1S    = 0x0A,
    TIM7_TIME_2S    = 0x14,
} TIM7ResetTime;

/* Global functions ***********************************************************/

/* Systick related */
void HAL_InitTick(u32 TickPriority, u32 TickSubPriority);
void HAL_IncTick(void);
u32 HAL_GetTick(void);
void HAL_Delay(__IO u32 Delay);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);

/* LED related */
void LED_Init(void);
void LED_flash(u8 times, u16 interval);
void LED_flash_short(u8 times);
void LED_flash_fast(u8 times);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

/* 蜂鸣器 */
void BEEP_On(void);
void BEEP_Off(void);
void BEEP_Init(void);	
void BEEP_flash(u8 times, u16 ms);

/* General Timer */
void TIM7_Int_Init(u16 arr, u16 psc);

/* Timer related */
#define ADC_TIMx              TIM6

extern Boolean timer_led_enable;
extern u16 timer_led_base_counter;
extern u16 timer_led_reset_counter;

extern Boolean timer_beep_enable;
extern u16 timer_beep_base_counter;
extern u16 timer_beep_reset_counter;

#endif /*  __SPIDER_BSP_H */
