/**
  @page EXTI_OverCurrent Ӳ������������⣬���ڳ��ֵ������ߺ����������û���
  ʵ��Ӧ���У��������⵽��ѹ������������ж϶���Ĺ����Դ���Է�ֹ������ջ١�
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    EXTI_OverCurrent/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   EXTI_OverCurrent ��������
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

@par ����˵��

�������������ʹ��Ӳ������������������ⲿ�ж������û�������������·���ֵ�������
��ֵ������������·�����MCUӲ���жϣ�������ڼ�⵽���źź󣬼�ʱ֪ͨ�û���

@par �����ļ�
  - SpiderXCube/BSP/Inc/power.h                          ��Դ����ģ��ͷ�ļ�
  - SpiderXCube/BSP/Src/power.h                          ��Դ����ģ�麯��
  - SpiderXCube/EXTI_OverCurrent/Inc/stm32f10x_conf.h    ������ʹ�õĿ�ģ��
  - SpiderXCube/EXTI_OverCurrent/Src/stm32f1xx_it.c      �жϴ�����ISR
  - SpiderXCube/EXTI_OverCurrent/Inc/stm32f1xx_it.h      �жϴ�����ISRͷ�ļ�
  - SpiderXCube/EXTI_OverCurrent/Src/main.c              ������
  - SpiderXCube/EXTI_OverCurrent/Inc/main.h              ������ͷ�ļ�
  - SpiderXCube/EXTI_OverCurrent/Src/system_stm32f1xx.c  STM32F1xx system source file


@par Ӳ��ƽ̨

  - �����̿���SpiderX-NANOӲ��ƽ̨������(����:http://www.robotbros.cn/index.html#!/products/)

@par ���ʹ�� ? 

 - ʹ������Ϥ�Ŀ�������(Ŀǰ֧��MDK, IAR)
 - ����IAR�����Ҫ���ڴ������У�����TARGETѡ��RAM�������±�����Ŀ�ļ�.
 - ����IAR�����Ҫ��FLASH�����У�����TARGETѡ��ROM�������±�����Ŀ�ļ�.
 - ���г���

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
