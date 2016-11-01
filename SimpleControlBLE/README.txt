/**
  @page SimpleControlBLE �����̸���һ���������������ƶ���Ƕȵ����ӣ�����ʵʱ
                         �鿴���ư�ĵ�ѹ�����������ʣ�������Ϣ��
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    SimpleControlBLE/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Jun-2016
  * @brief   SimpleControlBLE ��������
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

��������һ���������������ƶ�������̡�������ʵ������������֡���󣬵�һ��������18
������ĽǶ�֡SystemFrameSetDegreeReq������һ���ǻ�ȡ��λ��״̬��Ϣ֡
SystemFrameHealthReportReq��״̬��Ϣ���������������ѹ��������������Ϣ�Ĵ���ֵ��
������Ϣ�Ĵ���ֵ�����Է��������λ����ȡ����ʾ��

������ע����ǣ�Ӳ����������������С���޸ģ���Ϊ�����󣬲���Ӳ���жϣ���ʱ����
���жϳ����йرն����Դ������Ҫ�޸Ĵ��жϳ���ʱ���������⡣

��Ŀ����
--------

������ʹ�õĶ����Ʒ�ƺͰ�װ����������SpiderNANOConfig.h�й��ڶ���ĳ�ʼ�Ƕȣ�
��С����Ƕȷ�Χ���������Ƕȷ�Χ������
���磬��ʹ�õ���180�����̵Ķ������ʼ�Ƕ�90�ȣ�L_F_F��������С�Ƕ�Ϊ60�� 
���Ƕ�Ϊ180��Ӧ���������ø��������ļ�:

#define L_F_F_InitDegree                90
#define L_F_F_MaxDegree                180
#define L_F_F_MinDegree                 60

����֡��ʽ:

���г���֡��Ϊ20bytes��֡��ʽ����:

+----------+-----------------------------+-----------+
|  ֡ͷH   |           ֡����D           |  У���C  |
| (1 Byte) |          (18 bytes)         | ( 1 byte) |
+----------+-----------------------------+-----------+

У��ͼ��㹫ʽ:

C = (( H + D[0] + D[1] + ... + D[17] ) & 0xFF ) ^ 0xFF

����Ĭ��֧��2��֡

1. ���ö���Ƕ�����

����֡(����Ϊ��ʼ�Ƕ�):  
055a1e5a5a1e5a5a1e5a5a965a5a965a5a965a000000000000000000000000a6

��:    ��

����֡(ĩ��E�������Ϊ80�Ƕ�):  
05501e50501e50501e505096505096505096500000000000000000000000001e
[ ֡ͷ L-F-F  L-F-M  L-F-E .... (���嶨����鿴servo.h) ]

��:    ��

ע�⣬���õĽǶȱ����ڶ��������Χ֮�ڣ������ջٶ��.

2. ��ȡ���ư�״̬

����֡:  A20000000000000000000000000000000000005D
��:    A3 00 00  00 00  00000000  00000000  000000000000    5B   
            -----  -----  --------  --------  ------------  ------
            ��ѹ   ����   �������  �������  Ԥ��          У���

@par �����ļ�

  - SpiderXCube/BSP/Inc/bt4.h                            ����4.0ģ�麯��ͷ�ļ�
  - SpiderXCube/BSP/Src/bt4.c                            ����4.0ģ�麯��ʵ��
  - SpiderXCube/BSP/Inc/flash.h                          FLASH��������ͷ�ļ�
  - SpiderXCube/BSP/Src/flash.c                          FLASH��������ʵ��
  - SpiderXCube/BSP/Inc/power.h                          ��Դ������غ���ͷ�ļ�
  - SpiderXCube/BSP/Src/power.c                          ��Դ������غ���ʵ��
  - SpiderXCube/BSP/Inc/servo.h                          �������ģ��ͷ�ļ�
  - SpiderXCube/BSP/Src/servo.c                          �������ģ�麯��
  - SpiderXCube/BSP/Inc/spiderx_bsp.h                    SpiderX-NANOӲ��ƽ̨����
  - SpiderXCube/BSP/Src/spiderx_bsp.c                    SpiderX-NANOӲ��ƽ̨����ʵ��
  - SpiderXCube/SimpleControlBLE/Inc/SpiderNANOConfig.h  ��Ŀ�Զ��������ļ� (����ݾ���������)
  - SpiderXCube/SimpleControlBLE/Inc/stm32f10x_conf.h    ������ʹ�õĿ�ģ��
  - SpiderXCube/SimpleControlBLE/Src/stm32f1xx_it.c      �жϴ�����ISR
  - SpiderXCube/SimpleControlBLE/Inc/stm32f1xx_it.h      �жϴ�����ISRͷ�ļ�
  - SpiderXCube/SimpleControlBLE/Src/main.c              ������
  - SpiderXCube/SimpleControlBLE/Inc/main.h              ������ͷ�ļ�
  - SpiderXCube/SimpleControlBLE/Src/system_stm32f1xx.c  STM32F1xx system source file


@par Ӳ��ƽ̨

  - �����̿���SpiderX-NANOӲ��ƽ̨������(����:http://www.robotbros.cn/index.html#!/products/)

@par ���ʹ�� ? 

 - ʹ������Ϥ�Ŀ�������(Ŀǰ֧��MDK, IAR)
 - ����IAR�����Ҫ���ڴ������У�����TARGETѡ��RAM�������±�����Ŀ�ļ�.
 - ����IAR�����Ҫ��FLASH�����У�����TARGETѡ��ROM�������±�����Ŀ�ļ�.
 - ���г���

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
