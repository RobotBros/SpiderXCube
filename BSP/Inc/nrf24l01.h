/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/nrf24l01.h
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   NRF24L01 驱动定义头文件
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


#ifndef __NRF24L01_H
#define __NRF24L01_H  

#include "spiderx_bsp.h"
#include "stm32f10x.h"

typedef enum _NRF24L01State
{
    NRF24L01StateNotAttached = 0x00,        /*<! 2.4模块未接入                */
    NRF24L01StateReady,                     /*<! 2.4模块空闲                  */
    NRF24L01StateSending,                   /*<! 2.4模块正在发送              */
    NRF24L01StateSendTimeout,               /*<! 2.4模块发送超时              */
    NRF24L01StateReceiving,                 /*<! 2.4模块正在接受              */
    NRF24L01StateReceived,                  /*<! 2.4模块接受完毕              */
} NRF24L01State;

/**
 *@brief  2.4G控制单元结构体
 */
typedef struct _NRF24L01Handler
{
   NRF24L01State state;
   u8 *txBuffer;      /* 发送缓冲区指针 */
   u8 *rxBuffer;      /* 接收缓冲区指针 */
   u16 rxBufferLen;   /* 接收缓冲区长度 */
   u16 txBufferLen;   /* 发送缓冲区长度 */
   u16 frameLen;      /* 帧长 */
} NRF24L01Handler;

/**
 *@brief NRF24L01寄存器操作命令
 */
#define READ_REG_NRF                0x00  //读配置寄存器,低5位为寄存器地址
#define WRITE_REG_NRF               0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD                 0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD                 0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX                    0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX                    0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL                 0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP                         0xFF  //空操作,可以用来读状态寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG                      0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                                          //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA                       0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR                   0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW                    0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR                  0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH                       0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP                    0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS                      0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                                          //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  	            	0x10  //达到最大发送次数中断
#define TX_OK   	            	0x20  //TX发送完成中断
#define RX_OK               		0x40  //接收到数据中断

#define OBSERVE_TX                  0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD                          0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0                  0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1                  0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2                  0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3                  0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4                  0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5                  0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR                     0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0                    0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1                    0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2                    0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3                    0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4                    0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5                    0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS             0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;

/**
 *@brief 24L01操作线
 */
#define NRF24L01_CE                PAout(5)     //24L01片选信号
#define NRF24L01_CSN               PBout(12)    //SPI片选信号	   
#define NRF24L01_IRQ               PAin(4)      //IRQ主机数据输入

/**
 *@brief 24L01发送接收数据宽度定义
 */
#define TX_ADR_WIDTH                5   	    //5字节的地址宽度
#define RX_ADR_WIDTH                5   	    //5字节的地址宽度
//#define TX_PLOAD_WIDTH              32  	    //32字节的用户数据宽度
//#define RX_PLOAD_WIDTH              32  	    //32字节的用户数据宽度

/**
 *@brief  控制帧帧头
 */
#define NRF_FRAME_PC_HEADER         0xAA        //PC控制帧头
#define NRF_FRAME_JOYSTICK_HEADER   0xAB        //遥控杆控制帧头
#define NRF_FRAME_SYS_STATUS        0xAC        //下位机至上位机系统报告帧头 
     
/**
 *@brief  IRQ handler
 */
#define EXTIx_Handler               EXTI4_Handler
#define NRF24L01_EXTI_Line          EXTI_Line4
#define NRF24L01_EXTI_IRQn          EXTI4_IRQn

/* Global varialbles **********************************************************/

extern NRF24L01Handler nrfHandler;
									   	   
/* Public functions ***********************************************************/

/**
 * @brief  初始化24L01的IO口
 * @param  rxBufferPtr:  接收数据缓冲区指针
 * @param  txBufferPtr:  发送数据缓冲区指针
 * @param  rxBufferLen:  接收缓冲区长度 
 * @param  txBufferLen:  发送缓冲区长度 
 * @param  frameLen:     帧长
 * @retval 是否初始化成功. 
 *        TRUE  - 模块已接入并初始化成功
 *        FALSE - 模块未接入, 初始化失败
 */
Boolean NRF24L01_Init(u8 *rxBufferPtr, u8 *txBufferPtr, 
                      u16 rxBufferLen, u16 txBufferLen, 
                      u16 frameLen);

/**
 *@brief  初始化NRF24L01到RX模式
 *@param  None
 *@retval None
 */
void NRF24L01_RX_Mode(void);

/**
 *@brief  初始化NRF24L01到TX模式
 *@param  None
 *@retval None
 */
void NRF24L01_TX_Mode(void);

/**
 *@brief  在指定位置写指定长度的数据
 *@param  reg:   寄存器(位置)
 *@param  *pBuf: 数据指针
 *@param  len:   数据长度
 *@retval 返回值,此次读到的状态寄存器值 
 */
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);

/**
 *@brief  在指定位置读出指定长度的数据
 *@param  reg: 寄存器(位置)
 *@param  *pBuf: 数据指针
 *@retval 返回值,此次读到的状态寄存器值 
 */
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);

/**
 *@brief  读取SPI寄存器值
 *@param  reg: 要读的寄存器
 *@retval 状态值
 */
u8 NRF24L01_Read_Reg(u8 reg);

/**
 *@brief  SPI写寄存器
 *@param  reg: 指定寄存器地址
 *@param  value: 写入的值
 *@retval 状态值
 */
u8 NRF24L01_Write_Reg(u8 reg, u8 value);

/**
 *@brief  检测24L01是否存在
 *@param  None
 *@retval 0: 存在; 1: 不存在
 */
u8 NRF24L01_Check(void);

/**
 *@brief  启动NRF24L01发送一次数据
 *@param  txbuf:待发送数据首地址
 *@retval 发送完成状况
 */
u8 NRF24L01_TxPacket(u8 *txbuf);

/**
 *@brief  启动NRF24L01接收一次数据
 *@param  txbuf:待发送数据首地址
 *@retval 发送完成状况, 0:接收完成; 其他，错误代码
 */
u8 NRF24L01_RxPacket(u8 *rxbuf);

/**
 *@brief  清空TX缓冲区
 *@param  None
 *@return None
 */
void NRF24L01_ClearTxBuffer(void);

/**
 *@brief  清空RX缓冲区
 *@param  None
 *@return None
 */
void NRF24L01_ClearRxBuffer(void);

/**
 *@biref  通过2.4G发送数据. 数据需在调用此方法前填入nrfHandler
 *@param  None
 *@return None
 */
void NRF24L01_Send_Frame(void);

/**
 *@brief  计算校验和
 *@param  *data_buf: 数据缓冲区
 *@param  len: 数据缓冲区长度
 *@retval 校验和
 */
u8 NRF24L01_Calc_Checksum(u8 *data_buf, u8 len);

/**
 *@brief  NRF RX IRQ Service routine
 */
#define NRF24L01_IRQHandler       EXTI4_IRQHandler

#endif /* __NRF24L01_H */
