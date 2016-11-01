/**
  @page SpiderXCube包含SpiderX硬件平台的组件开发例程，例如通信模块蓝牙4.0/2.0，
  2.4G NRF24L01，硬件过流保护，软件电压采样检测，TIMER PWM舵机驱动，片上FLASH读
  写例程，FreeRTOS多进程协调通信例程等。通过这个Cube快速构建SpiderX六足机器人。

  @verbatim
  ******************** (C) COPYRIGHT 2016 RobotBros.cn *************************
  * @author  RobotBros Team
  * @version V0.1.0
  * @date    12-Mar-2016
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

@par  SpiderXCube目录结构：

 1. ADC_VoltageSampling       ADC电压采样，实现过压断电保护，低压警告，欠压断电保护，防止烧毁舵机
 2. EXTI_OverCurrent          电流保护外部中断例程。硬件过流保护模块实现电流阈值检测，过流中断，并切断舵机电源
 3. FLASH_SaveData            片上FLASH读写例程。
 4. FreeRTOS_Signal           FreeRTOS实现线程之间的唤醒与等待睡眠
 5. Middlewares               本例程用到的第三方库
 7. TIM_ServoControl          定时器PWM驱动舵机例程
 8  USART_DMA_Bluetooth2_0    DMA模式下的HC-06蓝牙2.0模块的数据读写例程
 9. USART_DMA_Bluetooth4_0    DMA模式下的BT-05蓝牙4.0模块的通信读写例程
10. USART_IT_Bluetooth2_0     中断方式的HC-06蓝牙2.0模块的数据读写例程
11. USART_IT_Bluetooth4_0     中断方式的BT-05蓝牙4.0模块的通信例程
12. SpiderXCube-Bluetooth-iOS iOS APP蓝牙通信例程 （只适用于带蓝牙4.0的iPhone，iPad，即iphone4s及后续版本）
13. STM32F10x_FWLib           STM32F10x官方驱动库
14. CMSIS                     ARM CMSIS官方库
15. BSP                       SpiderX-NANO硬件依赖定义
16. SimpleControlBLE          一个完整的蓝牙4.0控制舵机转动例程，并实现了过流过压保护
17. SimpleControlNRF          一个完整的无线2.4G控制舵机转动例程，并实现了过流过压保护
18. Joystick-ROM              遥控器固件，带LCD显示屏，MPU传感器, 下位机需配合SimpleControlNRF使用。
19. Labview                   Labview PC控制程序，下位机需配合SimpleControlNRF使用
20. SCH                       舵机控制板 + 遥控板原理图，端口定义
21. Datasheets                相关原件datasheets

@note 每个例子目录下都有一个README，方便开发者快速的了解该例子完成的功能，还有一些注意事项，请务必阅读。

@par  适用硬件平台：

SpiderX-NANO系列


淘宝购买链接：https://robotbros.taobao.com

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */