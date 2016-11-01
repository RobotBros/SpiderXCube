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

//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_UCOS		 0							//����ϵͳ�ļ����Ƿ�֧��UCOS
//****************** ϵͳ״̬��ʶ*********************//
extern u8 System_state_flag;										//system run state flag
extern u8 Old_system_state_flag;
extern u8 System_initial_state_flag;  								//system initial state flag
extern u8 Communicate_state_flag;   							  	//system ���� flag
extern u8 Ctrl_state_flag; 											//systerm ctrl or check state select flag
extern u8 Nrf24l01_rx_handle_flag;								//NRF �������ݴ�����ɱ�־

extern u8 System_HW_cur_protect_flag;							//ϵͳ������ֵ�����Ĵ���

extern u16 Timer7_nrf_base_cnt;                                 //TIM7 nrf 2.4gʱ��������
extern u8 Timer7_nrf_enable;                                    // TIM7 NRF 2.4G ����flag
extern u16 Timer7_base_cnt;										//TIM7 ʱ��������	
extern u8 System_display_time_falg;								//TIM7 ��ʾ��ʽ��־
extern u8 System_tim7_display_flag;								//ϵͳ״̬��ʾ��ʱ��־
extern u8 System_fault_state;										//ϵͳ����״̬�Ĵ���


//**********************************************************//
//************ϵͳ����ȫ�ּĴ���******************//
extern u16 Bat_val;												//battery voltage value
extern u16 Bat_cur;												//battery current value

//extern u16 Bat_adc_converted_value[CHANNEL_NUM][SAMPLE_NUM];	// ���������� [7][0]:��ѹ ��[7][1]:����

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

// **************����NRF RX��ɱ�־***************************//
#define DONE							1
#ifndef NULL
#define NULL							0
#endif

//***************����NRFģ�����־**************************//
#define NRF24L01_CHECK_NULL		1							//NRF24l01 module is null
#define NRF24L01_CHECK_OK     		0							//NRF24l01 module is here

//***************ϵͳ��ʼ����־*******************************//
#define SYSTEM_INITIALZE_NULL   	0		  					//initial done state
#define SYSTEM_INITIALZE_DONE 		1		 					//initial done state

//***************ϵͳң��״̬�½�����־************************//
#define SYSTEM_LOCK					0							//system lock state
#define SYSTEM_UNLOCK				1							//system unlock states

//***************ϵͳPC����orң�ؿ��Ʊ�־**********************//
#define SYSTEM_CTRL_STATE			0							//system ctrl cammand state flag
#define SYSTEM_CHECK_STATE		1							//system ckeck cammand state flag

//***************ϵͳ����״̬*********************************//
#define SYSTEM_NULL					0							//system reset state 
#define SYSTEM_STAND_BY   			1							//stand by state
#define SYSTEM_NORMAL				2							//normal work state
#define SYSTEM_NORMAL_WORRING	3							//normal worring state
#define SYSTEM_CTRL					4							//ctrl work state
#define SYSTEM_CTRL_WORRING		5							//ctrl worring state
#define SYSTEM_STOP					6					  		//stop work state	

//***************ϵͳ���Ϻ�***********************************//
#define SYSTEM_FAULT_NULL					0					//system fault: null
#define SYSTEM_FAULT_VOL_SOFT				1					//system fault: over or lower voltage soft sampling
#define SYSTEM_FAULT_OVER_CUR_SOFT		2					//system fault: over current soft sampling
#define SYSTEM_FAULT_OVER_PWR			3					//system fault: over power
#define SYSTEM_FAULT_OVER_CUR_HW			4					//system fault: over current  hardwear Ipp

//****************ϵͳ������ʾ������������ʱ��*****************//
#define SYSTEM_FAULT_DISPLAY_GAP_SEC		5					// system fault NO. display gap ,second	
#define SYSTEM_FAULT_DISPLAY_RETURN_MS	1800				// system fault NO. display return time ,second 180S = 3min

//****************ϵͳ״̬��ʾʱ���־**************************//
#define SYSTEM_DISPLAY_200MS				0
#define SYSTEM_DISPLAY_300MS				1
#define SYSTEM_DISPLAY_500MS				2
#define SYSTEM_DISPLAY_1S					3
//**********************************************************//
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
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
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����



void NVIC_Configuration(void);			 //�жϷ���
void SysInit(void);							 //ϵͳ��ʼ��
void System_pre_sampling(void);			//Ԥ������������ѹ
void System_moving_average_filter(void);//����ƽ��ֵ�˲�
void System_state_display(void);			//ϵͳչʾ
void System_state_check(void);			//ϵͳ״̬ת��
void System_lock_check(void);			//ϵͳ�����ж�
void System_nrf_tx_flame(void);			//ϵͳ2.4G �������ݴ���
void System_nrf_tx_handle(void);			//ϵͳ����2.4G ���ݴ���
void System_fault_display(void);			//ϵͳ���Ϻ���ʾ
void System_fault_check(void);			//ϵͳ���Ϻż��

#endif
