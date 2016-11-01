/**
  @page USART_IT_Bluetooth2_0 本例程介绍如何使用USART接口的蓝牙2.0模块HC-05
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    USART_IT_Bluetooth2_0/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   USART_IT_Bluetooth2_0 例程描述
  ******************************************************************************
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
  @endverbatim

@par 例程说明如何配置使用HC-05。例程开始初始化蓝牙模块，配置完成后，进入等待数据帧模式，
当收到数据(4个字节)后，反转LED灯，告诉上层应用接收到数据。接收到的数据在USARTx_rx_buf缓冲区内，
接收的数据长度由全局变量USARTx_RX_LEN指定。


@par 例程文件

  - SpiderXCube/BSP/Inc/bt2.h                                 蓝牙2.0 HC-05模块函数文件
  - SpiderXCube/BSP/Src/bt2.c                                 蓝牙2.0 HC-05模块函数
  - SpiderXCube/USART_IT_Bluetooth2_0/Inc/stm32f10x_conf.h    配置需使用的库模块
  - SpiderXCube/USART_IT_Bluetooth2_0/Src/stm32f1xx_it.c      中断处理函数ISR
  - SpiderXCube/USART_IT_Bluetooth2_0/Inc/stm32f1xx_it.h      中断处理函数ISR头文件
  - SpiderXCube/USART_IT_Bluetooth2_0/Src/main.c              主程序
  - SpiderXCube/USART_IT_Bluetooth2_0/Inc/main.h              主程序头文件
  - SpiderXCube/USART_IT_Bluetooth2_0/Src/system_stm32f1xx.c  STM32F1xx system source file


@par 硬件平台

  - 本例程可于SpiderX-NANO硬件平台上运行(详情:http://www.robotbros.cn/index.html#!/products/)

@par 如何使用 ? 

 - 使用你熟悉的开发工具(目前支持MDK, IAR)
 - 对于IAR如果需要在内存中运行，请在TARGET选择RAM，并重新编译项目文件.
 - 对于IAR如果需要在FLASH中运行，请在TARGET选择ROM，并重新编译项目文件.
 - 运行程序

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
