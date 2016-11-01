/**
  @page EXTI_OverCurrent 硬件电流采样检测，并在出现电流过高后，闪灯提醒用户。
  实际应用中，当软件检测到过压或过流，必须切断舵机的供电电源，以防止舵机的烧毁。
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    EXTI_OverCurrent/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   EXTI_OverCurrent 例程描述
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

@par 例程说明

本例程描述如何使用硬件电流采样，并配合外部中断提醒用户。当舵机供电电路出现电流超过
阈值，电流保护电路会产生MCU硬件中断，软件会在检测到该信号后，及时通知用户。

@par 例程文件
  - SpiderXCube/BSP/Inc/power.h                          电源管理模块头文件
  - SpiderXCube/BSP/Src/power.h                          电源管理模块函数
  - SpiderXCube/EXTI_OverCurrent/Inc/stm32f10x_conf.h    配置需使用的库模块
  - SpiderXCube/EXTI_OverCurrent/Src/stm32f1xx_it.c      中断处理函数ISR
  - SpiderXCube/EXTI_OverCurrent/Inc/stm32f1xx_it.h      中断处理函数ISR头文件
  - SpiderXCube/EXTI_OverCurrent/Src/main.c              主程序
  - SpiderXCube/EXTI_OverCurrent/Inc/main.h              主程序头文件
  - SpiderXCube/EXTI_OverCurrent/Src/system_stm32f1xx.c  STM32F1xx system source file


@par 硬件平台

  - 本例程可于SpiderX-NANO硬件平台上运行(详情:http://www.robotbros.cn/index.html#!/products/)

@par 如何使用 ? 

 - 使用你熟悉的开发工具(目前支持MDK, IAR)
 - 对于IAR如果需要在内存中运行，请在TARGET选择RAM，并重新编译项目文件.
 - 对于IAR如果需要在FLASH中运行，请在TARGET选择ROM，并重新编译项目文件.
 - 运行程序

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
