/**
  @page SpiderXCube����SpiderXӲ��ƽ̨������������̣�����ͨ��ģ������4.0/2.0��
  2.4G NRF24L01��Ӳ�����������������ѹ������⣬TIMER PWM���������Ƭ��FLASH��
  д���̣�FreeRTOS�����Э��ͨ�����̵ȡ�ͨ�����Cube���ٹ���SpiderX��������ˡ�

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

@par  SpiderXCubeĿ¼�ṹ��

 1. ADC_VoltageSampling       ADC��ѹ������ʵ�ֹ�ѹ�ϵ籣������ѹ���棬Ƿѹ�ϵ籣������ֹ�ջٶ��
 2. EXTI_OverCurrent          ���������ⲿ�ж����̡�Ӳ����������ģ��ʵ�ֵ�����ֵ��⣬�����жϣ����ж϶����Դ
 3. FLASH_SaveData            Ƭ��FLASH��д���̡�
 4. FreeRTOS_Signal           FreeRTOSʵ���߳�֮��Ļ�����ȴ�˯��
 5. Middlewares               �������õ��ĵ�������
 7. TIM_ServoControl          ��ʱ��PWM�����������
 8  USART_DMA_Bluetooth2_0    DMAģʽ�µ�HC-06����2.0ģ������ݶ�д����
 9. USART_DMA_Bluetooth4_0    DMAģʽ�µ�BT-05����4.0ģ���ͨ�Ŷ�д����
10. USART_IT_Bluetooth2_0     �жϷ�ʽ��HC-06����2.0ģ������ݶ�д����
11. USART_IT_Bluetooth4_0     �жϷ�ʽ��BT-05����4.0ģ���ͨ������
12. SpiderXCube-Bluetooth-iOS iOS APP����ͨ������ ��ֻ�����ڴ�����4.0��iPhone��iPad����iphone4s�������汾��
13. STM32F10x_FWLib           STM32F10x�ٷ�������
14. CMSIS                     ARM CMSIS�ٷ���
15. BSP                       SpiderX-NANOӲ����������
16. SimpleControlBLE          һ������������4.0���ƶ��ת�����̣���ʵ���˹�����ѹ����
17. SimpleControlNRF          һ������������2.4G���ƶ��ת�����̣���ʵ���˹�����ѹ����
18. Joystick-ROM              ң�����̼�����LCD��ʾ����MPU������, ��λ�������SimpleControlNRFʹ�á�
19. Labview                   Labview PC���Ƴ�����λ�������SimpleControlNRFʹ��
20. SCH                       ������ư� + ң�ذ�ԭ��ͼ���˿ڶ���
21. Datasheets                ���ԭ��datasheets

@note ÿ������Ŀ¼�¶���һ��README�����㿪���߿��ٵ��˽��������ɵĹ��ܣ�����һЩע�����������Ķ���

@par  ����Ӳ��ƽ̨��

SpiderX-NANOϵ��


�Ա��������ӣ�https://robotbros.taobao.com

 * <h3><center>&copy; COPYRIGHT RobotBros.cn</center></h3>
 */