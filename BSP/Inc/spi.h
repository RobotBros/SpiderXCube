/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/spi.h
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


#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"
			  	    
/* 初始化SPI口 */
void SPI2_Init(void);

/* 设置SPI速度 */
void SPI2_SetSpeed(u8 SpeedSet);

/* SPI总线读写一个字节 */
u8 SPI2_ReadWriteByte(u8 TxData);
		 
#endif
