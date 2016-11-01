/**
  @page ADC_VoltageSampling 软件电压/电流采样，并在出现电压或电流过高后，
  以不同频率闪灯提醒用户。实际应用中，当软件检测到过压或过流，必须切断舵机的
  供电电源，以防止舵机的烧毁。
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    ADC_VoltageSampling/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   ADC_VoltageSampling 例程描述
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

本例程描述如何使用ADC采样，并配合软件滤波实现基本的电压/电流过压过流检测，并提醒用户。

本例程的ADC采样值会保存在一个私有变量数组中，并启用了定时器，每隔20ms出现定时器中断，
并调用软件滤波函数(移动平均值滤波)检测电压/电流是否超过了阈值。如果出现如下情况，
将闪灯提醒用户:

  - 过压          每隔500ms反转LED灯，连续闪烁10次
  - 低压警告      每隔1000ms反转LED灯，连续闪烁5次
  - 电压过低      每隔500ms反转LED灯，连续闪烁5次
  - 软件检测过流  每隔500ms反转LED灯，连续闪烁15次

@par 例程文件

  - SpiderXCube/BSP/Inc/power.h                             电源管理模块头文件
  - SpiderXCube/BSP/Src/power.h                             电源管理模块函数
  - SpiderXCube/ADC_VoltageSampling/Inc/stm32f10x_conf.h    配置需使用的库模块
  - SpiderXCube/ADC_VoltageSampling/Src/stm32f1xx_it.c      中断处理函数ISR
  - SpiderXCube/ADC_VoltageSampling/Inc/stm32f1xx_it.h      中断处理函数ISR头文件
  - SpiderXCube/ADC_VoltageSampling/Src/main.c              主程序
  - SpiderXCube/ADC_VoltageSampling/Inc/main.h              主程序头文件
  - SpiderXCube/ADC_VoltageSampling/Src/system_stm32f1xx.c  STM32F1xx system source file


@par 硬件平台

  - 本例程可于SpiderX-NANO硬件平台上运行(详情:http://www.robotbros.cn/index.html#!/products/)

@par 如何使用 ? 

 - 使用你熟悉的开发工具(目前支持MDK, IAR)
 - 对于IAR如果需要在内存中运行，请在TARGET选择RAM，并重新编译项目文件.
 - 对于IAR如果需要在FLASH中运行，请在TARGET选择ROM，并重新编译项目文件.
 - 运行程序

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
