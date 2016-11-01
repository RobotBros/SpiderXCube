/**
  ******************************************************************************
  * @file    SpiderXCube/USART_IT_Bluetooth2_0/Src/main.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   蓝牙通信例程
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

#include "main.h"

extern u32 USARTx_RX_CURRENT_LEN;
extern u32 USARTx_RX_LEN;

/*
 *  @brief  缓冲区
 */
#define SYSTEM_TX_FRAME_BUFFER_SIZE             100
#define SYSTEM_RX_FRAME_BUFFER_SIZE             100

/* Private variables **********************************************************/
static u8 SYSTEM_RX_FRAME_BUFFER[SYSTEM_RX_FRAME_BUFFER_SIZE];
static u8 SYSTEM_TX_FRAME_BUFFER[SYSTEM_TX_FRAME_BUFFER_SIZE];

/**
 *@brief  Main function
 *@param  None
 *@retval None
 */
int main(void)
{   
    Boolean error = FALSE;
    
    LED_Init();
    
    /* Systick Init */
    HAL_InitTick(IRQ_SYSTICK_Priority, 0);
    
    /* 蓝牙初始化，重试次数=2 */
    if (!BT_Init(2, SYSTEM_RX_FRAME_BUFFER, SYSTEM_TX_FRAME_BUFFER))
        error = TRUE;
    
    while(1)
    {
        if (!error)
        {
            USARTx_RX_CURRENT_LEN = 4;
            
            /* 等待接收蓝牙帧 */
            while(USARTx_RX_LEN != USARTx_RX_CURRENT_LEN);
            
            /*接收到数据帧*/
            LED_Toggle();
            
            /* 清除缓冲区，并使能串口中断 */
            USARTx_resetBuffer();
        }
    }
}

