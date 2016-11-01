#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
#include "led.h"
#include "beep.h"
#include "delay.h"
#include "adc.h"
#include "sys.h"
//#include "usart.h"
#include "timer.h"
#include "exti.h"
#include "spi.h"
#include "24l01.h"
//#include "comm_pc.h"
//#include "nokia5110.h" 
#include "Lcd_Driver.h"
#include "key.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "imu_motion.h"	


//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_UCOS		0				//����ϵͳ�ļ����Ƿ�֧��UCOS

#define LCD_144INCH

#define NRF42L01_LEN  				33  		//����NRF24L01�������ֽ��� 33

extern u8 System_state_flag;					//system run state flag
extern u8 System_fault_flag;					//system fault flag

extern u8 Nrf24l01_rx_handle_flag;					//NRF �������ݴ�����ɱ�־

extern u8 Nrf24l01_tx_buff[NRF42L01_LEN];				//NRF24L01 TX buffer
extern u8 Nrf24l01_rx_buff[NRF42L01_LEN];				//NRF24L01 RX buffer

extern u16 Bat_val;
extern u16 Photoresistor_val;

extern u16 Val_ladx,Val_lady,Val_radx,Val_rady;  //Joystick VAl buffer
extern u8 Bat_slave_high_val,Bat_slave_low_val;		//slave batteryval

extern u8 Pre_ladx,Pre_lady,Pre_radx,Pre_rady;				//�����ٷֱ�
extern u8 Pre_bat,Pre_photoresistor;

typedef struct  
{										    
	u8 led_ini_state;		//LED��ʼ��״̬
	u8 lcd_ini_state;			
	u8 dma_ini_state;				
	u8 adc_ini_state;				
	u8 nrf_ini_state;		
	u8 beep_ini_state;		
	u8 key_ini_state;
	u8 system_timer_ini_state;
	u8 mpu_ini_state;
}_system_ini_type; 	

typedef struct  
{										    
	u8 new_key_type;		
	u8 old_key_type;	
	u8 key_handle_flag;
}_system_key_type;

extern _system_ini_type System_ini_state;
extern _system_key_type System_key_type;

#define DONE  1
#define NOT 	  0

#define NRF24L01_CHECK_NULL		 1			//NRF24l01 module is null
#define NRF24L01_CHECK_OK     		 0			//NRF24l01 module is here
	

//---------------ϵͳ��������--------------------
#define SYSTEM_NO_FAULT				 0
#define SYSTEM_NRF_NULL				 1
#define SYSTEM_VOLTAGE_FAULT			 2			

	 
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



void NVIC_Configuration(void);				//�жϷ���
void System_variable_init(void);
void SysInit(void);									  //ϵͳ��ʼ��
void Syterm_master_nrf_tx_handle(void);
void Syterm_master_nrf_rx_handle(void);
void System_nrf_tx_timer(void);
void System_nrf_rx_timer(void);
void System_fault_clear_timer(void);
void System_fault_clear_handle(void);
void System_fault_check_timer(void);
void System_fault_check_handle(void);


#endif
