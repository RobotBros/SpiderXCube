#ifndef __SYS_H
#define __SYS_H	
#include "system_conf.h"
#include "stm32f10x.h"
#include "led.h"
#include "beep.h"
#include "delay.h"
#include "adc.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "action.h"
#include "exti.h"
#include "spi.h"
#include "24l01.h"



#define NRF24L01_LEN						32	  // NRF24l01 module RX data len

//0,不支持ucos
//1,支持ucos
#define SYSTEM_SUPPORT_UCOS		 0							//定义系统文件夹是否支持UCOS
//****************** 系统状态标识*********************//
extern u8 System_state_flag;										//system run state flag
extern u8 Old_system_state_flag;
extern u8 System_initial_state_flag;  								//system initial state flag
extern u8 Communicate_state_flag;   							  	//system 解锁 flag
extern u8 Ctrl_state_flag; 											//systerm ctrl or check state select flag
extern u8 Nrf24l01_rx_handle_flag;								//NRF 接收数据处理完成标志

extern u8 System_HW_cur_protect_flag;							//系统电流峰值保护寄存器

extern u16 Timer7_nrf_base_cnt;                                 //TIM7 nrf 2.4g时基计数器
extern u8 Timer7_nrf_enable;                                    // TIM7 NRF 2.4G 启用flag
extern u16 Timer7_base_cnt;										//TIM7 时基计数器	
extern u8 System_display_time_falg;								//TIM7 显示方式标志
extern u8 System_tim7_display_flag;								//系统状态显示定时标志
extern u8 System_fault_state;										//系统故障状态寄存器


//**********************************************************//
//************系统数据全局寄存器******************//
extern u16 Bat_val;												//battery voltage value
extern u16 Bat_cur;												//battery current value

//extern u16 Bat_adc_converted_value[CHANNEL_NUM][SAMPLE_NUM];	// 采样缓存器 [7][0]:电压 ，[7][1]:电流

extern u8 Nrf24l01_rx_buff[NRF24L01_LEN];			 				//IRQ intterupt RX data buffer
extern u8 Nrf24l01_tx_buff[NRF24L01_LEN];			 				//IRQ intterupt TX data buffer

extern u8 Joystick_left_x_axis_val,Joystick_left_y_axis_val;		//joystick X/Y value
extern u8 Joystick_right_x_axis_val,Joystick_right_y_axis_val;
extern u8 Steering_engine_L_B_E_angle;
extern u8 Steering_engine_L_B_F_angle;
extern u8 Steering_engine_L_B_M_angle;
extern u8 Steering_engine_L_M_E_angle;
extern u8 Steering_engine_L_M_M_angle;
extern u8 Steering_engine_L_M_F_angle;
extern u8 Steering_engine_L_F_E_angle;
extern u8 Steering_engine_L_F_M_angle;
extern u8 Steering_engine_L_F_F_angle;
extern u8 Steering_engine_R_B_E_angle;
extern u8 Steering_engine_R_B_M_angle;
extern u8 Steering_engine_R_B_F_angle;
extern u8 Steering_engine_R_M_E_angle;
extern u8 Steering_engine_R_M_M_angle;
extern u8 Steering_engine_R_M_F_angle;
extern u8 Steering_engine_R_F_E_angle;
extern u8 Steering_engine_R_F_M_angle;
extern u8 Steering_engine_R_F_F_angle;

extern u8 Steering_engine_DRV1_angle;
extern u8 Steering_engine_DRV2_angle;
extern u8 Steering_engine_DRV3_angle;
extern u8 Steering_engine_DRV4_angle;
//**********************************************************//

// **************无线NRF RX完成标志***************************//
#define DONE							1
#ifndef NULL
#define NULL							0
#endif

//***************无线NRF模块检测标志**************************//
#define NRF24L01_CHECK_NULL		1							//NRF24l01 module is null
#define NRF24L01_CHECK_OK     		0							//NRF24l01 module is here

//***************系统初始化标志*******************************//
#define SYSTEM_INITIALZE_NULL   	0		  					//initial done state
#define SYSTEM_INITIALZE_DONE 		1		 					//initial done state

//***************系统遥控状态下解锁标志************************//
#define SYSTEM_LOCK					0							//system lock state
#define SYSTEM_UNLOCK				1							//system unlock states

//***************系统PC控制or遥控控制标志**********************//
#define SYSTEM_CTRL_STATE			0							//system ctrl cammand state flag
#define SYSTEM_CHECK_STATE		1							//system ckeck cammand state flag

//***************系统运行状态*********************************//
#define SYSTEM_NULL					0							//system reset state 
#define SYSTEM_STAND_BY   			1							//stand by state
#define SYSTEM_NORMAL				2							//normal work state
#define SYSTEM_NORMAL_WORRING	3							//normal worring state
#define SYSTEM_CTRL					4							//ctrl work state
#define SYSTEM_CTRL_WORRING		5							//ctrl worring state
#define SYSTEM_STOP					6					  		//stop work state	

//***************系统故障号***********************************//
#define SYSTEM_FAULT_NULL					0					//system fault: null
#define SYSTEM_FAULT_VOL_SOFT				1					//system fault: over or lower voltage soft sampling
#define SYSTEM_FAULT_OVER_CUR_SOFT		2					//system fault: over current soft sampling
#define SYSTEM_FAULT_OVER_PWR			3					//system fault: over power
#define SYSTEM_FAULT_OVER_CUR_HW			4					//system fault: over current  hardwear Ipp

//****************系统故障显示间隔及清除故障时间*****************//
#define SYSTEM_FAULT_DISPLAY_GAP_SEC		5					// system fault NO. display gap ,second	
#define SYSTEM_FAULT_DISPLAY_RETURN_MS	1800				// system fault NO. display return time ,second 180S = 3min

//****************系统状态显示时间标志**************************//
#define SYSTEM_DISPLAY_200MS				0
#define SYSTEM_DISPLAY_300MS				1
#define SYSTEM_DISPLAY_500MS				2
#define SYSTEM_DISPLAY_1S					3
//**********************************************************//
	 
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
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
 
//IO口操作,只对单一的IO口!
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



void NVIC_Configuration(void);			 //中断分组
void SysInit(void);							 //系统初始化
void System_pre_sampling(void);			//预采样电流，电压
void System_moving_average_filter(void);//滑动平均值滤波
void System_state_display(void);			//系统展示
void System_state_check(void);			//系统状态转换
void System_lock_check(void);			//系统解锁判断
void System_nrf_tx_flame(void);			//系统2.4G 发送数据处理
void System_nrf_tx_handle(void);			//系统发送2.4G 数据处理
void System_fault_display(void);			//系统故障号显示
void System_fault_check(void);			//系统故障号检测

#endif
