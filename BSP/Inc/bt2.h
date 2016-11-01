/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/bt2.h
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   蓝牙2.0 HC-06模块驱动函数
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 RobotBros.cn</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of RobotBros.cn nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


#ifndef __BT2_H
#define __BT2_H

#include "spiderx_bsp.h"
#include "stdio.h"
#include "stdarg.h"

/* Macro definition ***********************************************************/

#define BT_USARTx_MAX_RECV_CMD_LEN	    11                  //最大接收缓存字节数

#define BT_USARTx_DMA_TX_CH			    DMA1_Channel2       //TX DMA1 通道2
#define BT_USARTx_DMA_RX_CH			    DMA1_Channel3       //RX DMA1 通道3
#define BT_USARTx_DMA_RX_TC             DMA1_FLAG_TC3       //RX Transfer complete bit
#define BT_DMA_Channel_IRQn             DMA1_Channel3_IRQn

#define BT_USARTx_FRAME_RX_LEN          4

#define BT_BAUND 	                    115200
#define BT_TX_PIN                       GPIO_Pin_10
#define BT_RX_PIN                       GPIO_Pin_11
#define BT_USARTx_PORT                  GPIOC
#define BT_USARTx                       USART3
#define BT_USARTx_IRQn                  USART3_IRQn
#define __BT_PORT_RCC_ENABLE__()        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE)
#define __BT_USART_RCC_ENABLE__()       RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE)
#define __BT_PIN_REMAP__()              GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE)
      
#define BT_USARTx_IRQHandler            USART3_IRQHandler
#define BT_DMAx_Channelx_IRQHandler     DMA1_Channel3_IRQHandler

extern u32 USARTx_RX_LEN;                                 /* 需接收的数据长度 */
extern u32 USARTx_RX_CURRENT_LEN;                         /* 已接收的数据长度 */
extern Boolean BT_DataReady;
      
/* Public functions ***********************************************************/

u8   BT_Init(u8 retry, u8 *rxBufferPtr, u8 *txBufferPtr);
void BT_SendFrame(u8 *buffer, u16 bufferLen);

void USARTx_Init_DMA(u16 bufferLen);
void USARTx_HandlerInit(void);
void USARTx_Init(u32 bound);
void USARTx_printf(char* fmt,...);
void USARTx_print(u8 *buffer, u16 bufferLen);
void BT_DMAx_Channelx_IRQHandler(void);
void USARTx_dma_rx_start(u16 len);
void USARTx_resetBuffer(void);
void BT_USARTx_IRQHandler(void);

#endif
