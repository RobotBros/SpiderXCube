/**
  @page SimpleControlBLE 本例程给出一个完整的蓝牙控制舵机角度的例子，并可实时
                         查看控制板的电压，电流，功率，错误信息。
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    SimpleControlBLE/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Jun-2016
  * @brief   SimpleControlBLE 例程描述
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

本例程是一个完整的蓝牙控制舵机的例程。本例程实现了两种数据帧请求，第一个是设置18
个舵机的角度帧SystemFrameSetDegreeReq，另外一个是获取下位机状态信息帧
SystemFrameHealthReportReq，状态信息包括，软件采样电压，电流，错误信息寄存器值，
警告信息寄存器值，可以方便的在上位机获取并显示。

另外需注意的是，硬件过流保护检测必须小心修改，因为过流后，产生硬件中断，此时必须
在中断程序中关闭舵机电源。若需要修改此中断程序时，必须留意。

项目配置
--------

根据你使用的舵机的品牌和安装的需求，设置SpiderNANOConfig.h中关于舵机的初始角度，
最小允许角度范围，最大允许角度范围参数。
例如，你使用的是180度量程的舵机，初始角度90度，L_F_F的允许最小角度为60， 
最大角度为180，应按如下设置更改配置文件:

#define L_F_F_InitDegree                90
#define L_F_F_MaxDegree                180
#define L_F_F_MinDegree                 60

蓝牙帧格式:

所有长度帧均为20bytes，帧格式如下:

+----------+-----------------------------+-----------+
|  帧头H   |           帧数据D           |  校验和C  |
| (1 Byte) |          (18 bytes)         | ( 1 byte) |
+----------+-----------------------------+-----------+

校验和计算公式:

C = (( H + D[0] + D[1] + ... + D[17] ) & 0xFF ) ^ 0xFF

程序默认支持2种帧

1. 设置舵机角度请求

请求帧(设置为初始角度):  
055a1e5a5a1e5a5a1e5a5a965a5a965a5a965a000000000000000000000000a6

答复:    无

请求帧(末端E舵机设置为80角度):  
05501e50501e50501e505096505096505096500000000000000000000000001e
[ 帧头 L-F-F  L-F-M  L-F-E .... (具体定义请查看servo.h) ]

答复:    无

注意，设置的角度必须在舵机的允许范围之内，以免烧毁舵机.

2. 获取控制板状态

请求帧:  A20000000000000000000000000000000000005D
答复:    A3 00 00  00 00  00000000  00000000  000000000000    5B   
            -----  -----  --------  --------  ------------  ------
            电压   电流   错误代码  警告代码  预留          校验和

@par 例程文件

  - SpiderXCube/BSP/Inc/bt4.h                            蓝牙4.0模块函数头文件
  - SpiderXCube/BSP/Src/bt4.c                            蓝牙4.0模块函数实现
  - SpiderXCube/BSP/Inc/flash.h                          FLASH操作函数头文件
  - SpiderXCube/BSP/Src/flash.c                          FLASH操作函数实现
  - SpiderXCube/BSP/Inc/power.h                          电源采样监控函数头文件
  - SpiderXCube/BSP/Src/power.c                          电源采样监控函数实现
  - SpiderXCube/BSP/Inc/servo.h                          舵机控制模块头文件
  - SpiderXCube/BSP/Src/servo.c                          舵机控制模块函数
  - SpiderXCube/BSP/Inc/spiderx_bsp.h                    SpiderX-NANO硬件平台定义
  - SpiderXCube/BSP/Src/spiderx_bsp.c                    SpiderX-NANO硬件平台定义实现
  - SpiderXCube/SimpleControlBLE/Inc/SpiderNANOConfig.h  项目自定义配置文件 (需根据具体舵机设置)
  - SpiderXCube/SimpleControlBLE/Inc/stm32f10x_conf.h    配置需使用的库模块
  - SpiderXCube/SimpleControlBLE/Src/stm32f1xx_it.c      中断处理函数ISR
  - SpiderXCube/SimpleControlBLE/Inc/stm32f1xx_it.h      中断处理函数ISR头文件
  - SpiderXCube/SimpleControlBLE/Src/main.c              主程序
  - SpiderXCube/SimpleControlBLE/Inc/main.h              主程序头文件
  - SpiderXCube/SimpleControlBLE/Src/system_stm32f1xx.c  STM32F1xx system source file


@par 硬件平台

  - 本例程可于SpiderX-NANO硬件平台上运行(详情:http://www.robotbros.cn/index.html#!/products/)

@par 如何使用 ? 

 - 使用你熟悉的开发工具(目前支持MDK, IAR)
 - 对于IAR如果需要在内存中运行，请在TARGET选择RAM，并重新编译项目文件.
 - 对于IAR如果需要在FLASH中运行，请在TARGET选择ROM，并重新编译项目文件.
 - 运行程序

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
