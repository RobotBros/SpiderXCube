/**
  @page FLASH_SaveData �����̽�����ζ�дƬ��flash��
  
  @verbatim
  ******************** (C) COPYRIGHT 2015 RobotBros.cn *******************
  * @file    FLASH_SaveData/readme.txt 
  * @author  Enix Yu
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   FLASH_SaveData ��������
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

������������α������ݵ�Ƭ��flash���ڿ�����������˵�ʱ��������Ҫ��һЩԤ�ȱ���
�õĶ����ļ����ص��������ϣ�����Ҫ�����˰�˳��ִ��ָ�����漰����α������ݵ�
���⡣����Ǳ����̵�Ŀ�ġ�

���⣬һ��д��flash��ʱ�򣬶�����дһ��ǰ����һ�Ρ����������������ݿ��������
�����ݵ�д��ֽ�Ϊ�������񣬲������ݣ�д��1�Σ�д��2�Σ�д��n�Σ���������ͨ��
������������ǾͿ��Է���д��ͬһ��flashҳ��ʵ�ָ�����д�����ݡ�

ͬʱ��������һ���flash��д������һ��֮�����У�������PADDING CHAR���֧���ֽ�д�룬
����������ֻ��д�����(half word),��һ����(word)�����ַ����������ϲ�Ӧ���и����
���ɶ���д��flash��

�����̳�ʼ����ɺ�������flashд������Ȼ���������������Ҫ�õ���ҳ��Ȼ����д��
5���ֽڵ�����(�˴���ʹ�õ�����ֽ�PAD)����ɺ������д��6���ֽڵ����ݣ�
���Ƚ�д���flash�����Ƿ���sampleData1/2��ȡ�

@par �����ļ�

  - SpiderXCube/BSP/Inc/flash.h                        flash��д����ͷ�ļ�
  - SpiderXCube/BSP/Src/flash.c                        flash��д����
  - SpiderXCube/FLASH_SaveData/Inc/stm32f10x_conf.h    ������ʹ�õĿ�ģ��
  - SpiderXCube/FLASH_SaveData/Src/stm32f1xx_it.c      �жϴ�����ISR
  - SpiderXCube/FLASH_SaveData/Inc/stm32f1xx_it.h      �жϴ�����ISRͷ�ļ�
  - SpiderXCube/FLASH_SaveData/Src/main.c              ������
  - SpiderXCube/FLASH_SaveData/Inc/main.h              ������ͷ�ļ�
  - SpiderXCube/FLASH_SaveData/Src/system_stm32f1xx.c  STM32F1xx system source file


@par Ӳ��ƽ̨

  - �����̿���SpiderX-NANOӲ��ƽ̨������(����:http://www.robotbros.cn/index.html#!/products/)

@par ���ʹ�� ? 

 - ʹ������Ϥ�Ŀ�������(Ŀǰ֧��MDK, IAR)
 - ����IAR�����Ҫ���ڴ������У�����TARGETѡ��RAM�������±�����Ŀ�ļ�.
 - ����IAR�����Ҫ��FLASH�����У�����TARGETѡ��ROM�������±�����Ŀ�ļ�.
 - ���г���

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */
