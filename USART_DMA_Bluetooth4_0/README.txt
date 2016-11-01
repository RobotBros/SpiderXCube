/**
  @page USART_DMA_Bluetooth4_0 �����̽������ʹ��USART�ӿڵ�����4.0ģ��BT-05
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    USART_DMA_Bluetooth4_0/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   USART_DMA_Bluetooth4_0 ��������
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

@par ����˵���������ʹ��BT-05�����̿�ʼ��ʼ������ģ�飬������ɺ��л�������
����DMAģʽ��Ȼ�����ȴ�����֡ģʽ�����յ����ݺ󣬷�תLED�ƣ������ϲ�Ӧ�ý��յ�
���ݡ����յ���������USARTx_rx_buf�������ڣ����յ����ݳ�����ȫ�ֱ���USARTx_RX_LENָ����


@par �����ļ�

  - SpiderXCube/BSP/Inc/bt4.h                                  ����4.0 BT-05ģ�麯���ļ�
  - SpiderXCube/BSP/Src/bt4.c                                  ����4.0 BT-05ģ�麯��
  - SpiderXCube/USART_DMA_Bluetooth4_0/Inc/stm32f10x_conf.h    ������ʹ�õĿ�ģ��
  - SpiderXCube/USART_DMA_Bluetooth4_0/Src/stm32f1xx_it.c      �жϴ�����ISR
  - SpiderXCube/USART_DMA_Bluetooth4_0/Inc/stm32f1xx_it.h      �жϴ�����ISRͷ�ļ�
  - SpiderXCube/USART_DMA_Bluetooth4_0/Src/main.c              ������
  - SpiderXCube/USART_DMA_Bluetooth4_0/Inc/main.h              ������ͷ�ļ�
  - SpiderXCube/USART_DMA_Bluetooth4_0/Src/system_stm32f1xx.c  STM32F1xx system source file


@par Ӳ��ƽ̨

  - �����̿���SpiderX-NANOӲ��ƽ̨������(����:http://www.robotbros.cn/index.html#!/products/)

@par ���ʹ�� ? 

 - ʹ������Ϥ�Ŀ�������(Ŀǰ֧��MDK, IAR)
 - ����IAR�����Ҫ���ڴ������У�����TARGETѡ��RAM�������±�����Ŀ�ļ�.
 - ����IAR�����Ҫ��FLASH�����У�����TARGETѡ��ROM�������±�����Ŀ�ļ�.
 - ���г���

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
