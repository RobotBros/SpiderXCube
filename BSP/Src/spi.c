/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/spi.c
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   SPI相关函数定义
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

#include "spi.h"

/* Public functions ***********************************************************/

/** 
 *@brief  SPI2初始化 PB13-SCK PB14-MISO PB15-MOSI
 */
void SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);																		
    
    /* PB13/14/15上拉 */
 	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    
    /* SPI2设置为双线双线全双工 */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    
    /* SPI2设置为主机模式 */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    
    /* SPI2设置为收发为8帧模式 */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    
    /* 串行同步时钟空闲状态设置为低电平 */
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    
    /* 串行同步时钟第一个跳变沿，数据被采集 */
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    
    /* NSS使用软件控制，即I/O控制 */
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    
    /* 波特率预分频值为16 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    
    /* 数据处理从MSB开始，最高位 */
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    
    /* CRC计算设置 */
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);  																   
    
	SPI_Cmd(SPI2, ENABLE);
	SPI2_ReadWriteByte(0xff); //start TX/RX
    
    /* spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）*/
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);
}  

/**
 *@brief  SPI 速度设置函数
 *@param  SpeedSet:
 *        SPI_BaudRatePrescaler_2   2分频   
 *        SPI_BaudRatePrescaler_8   8分频   
 *        SPI_BaudRatePrescaler_16  16分频  
 *        SPI_BaudRatePrescaler_256 256分频 
 */
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1 &= 0XFFC7;
    /* 设置SPI2速度 */
	SPI2->CR1 |= SPI_BaudRatePrescaler;
	SPI_Cmd(SPI2, ENABLE); 
} 

/**
 *@brief  SPIx 读写一个字节
 *@param  TxData:要写入的字节
 *@retval 读取到的字节
 */
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry = 0;
    
    /* 检查指定的SPI标志位设置与否:发送缓存空标志位 */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) 
    {
		retry++;
		if(retry>200)return 0;
    }
    
    /* 通过外设SPIx发送一个数据 */
	SPI_I2S_SendData(SPI2, TxData);
	retry = 0;
    
    /* 检查指定的SPI标志位设置与否:接受缓存非空标志位 */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
		retry ++;
		if(retry > 200)
            return 0;
    }
    
    /* 返回通过SPIx最近接收的数据 */
	return SPI_I2S_ReceiveData(SPI2);
}
