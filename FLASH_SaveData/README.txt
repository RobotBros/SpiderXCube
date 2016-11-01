/**
  @page FLASH_SaveData 本例程介绍如何读写片上flash。
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    FLASH_SaveData/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   FLASH_SaveData 例程描述
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

本例程描述如何保存数据到片上flash。在开发六足机器人的时候，往往需要把一些预先编排
好的动作文件下载到机器人上，并需要机器人按顺序执行指令。这就涉及了如何保存数据的
问题。这就是本例程的目的。

另外，一般写入flash的时候，都必须写一次前擦除一次。本例程引入了数据库的事务概念，
把数据的写入分解为开启事务，擦除数据，写入1次，写入2次，写入n次，结束事务。通过
这个方法，我们就可以分批写入同一个flash页，实现更灵活的写入数据。

同时本例程与一般的flash读写方法不一样之处还有，引入了PADDING CHAR概念，支持字节写入，
而不是限制只能写入半字(half word),或一个字(word)。这种方法可以让上层应用有更大的
自由度来写入flash。

本例程初始化完成后，先启用flash写入事务，然后擦除本次事务需要用到的页，然后再写入
5个字节的数据(此处会使用到填充字节PAD)，完成后接着再写入6个字节的数据，
最后比较写入的flash数据是否与sampleData1/2相等。

@par 例程文件

  - SpiderXCube/BSP/Inc/flash.h                        flash读写函数头文件
  - SpiderXCube/BSP/Src/flash.c                        flash读写函数
  - SpiderXCube/FLASH_SaveData/Inc/stm32f10x_conf.h    配置需使用的库模块
  - SpiderXCube/FLASH_SaveData/Src/stm32f1xx_it.c      中断处理函数ISR
  - SpiderXCube/FLASH_SaveData/Inc/stm32f1xx_it.h      中断处理函数ISR头文件
  - SpiderXCube/FLASH_SaveData/Src/main.c              主程序
  - SpiderXCube/FLASH_SaveData/Inc/main.h              主程序头文件
  - SpiderXCube/FLASH_SaveData/Src/system_stm32f1xx.c  STM32F1xx system source file


@par 硬件平台

  - 本例程可于SpiderX-NANO硬件平台上运行(详情:http://www.robotbros.cn/index.html#!/products/)

@par 如何使用 ? 

 - 使用你熟悉的开发工具(目前支持MDK, IAR)
 - 对于IAR如果需要在内存中运行，请在TARGET选择RAM，并重新编译项目文件.
 - 对于IAR如果需要在FLASH中运行，请在TARGET选择ROM，并重新编译项目文件.
 - 运行程序

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
