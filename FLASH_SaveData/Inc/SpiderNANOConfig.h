/**
  ******************************************************************************
  * @file    SpiderXCube/SimpleControlBLE/Inc/SpiderNANOConfig.h
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-AUG-2016
  * @brief   �����ļ�
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 RobotBros.cn</center></h2>
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
  */

#ifndef __SPIDER_NANO_CONFIG_H
#define __SPIDER_NANO_CONFIG_H

/**
 * @brief Servo initial degree (�����ʼ���Ƕ�)
 */
#define L_F_F_InitDegree                90   /*<!  ��ǰ��ǰ�˵��             */
#define L_F_M_InitDegree                30   /*<!  ��ǰ���м�ؽڵ��         */
#define L_F_E_InitDegree                90   /*<!  ��ǰ��ĩ�˵��             */
#define L_M_F_InitDegree                90   /*<!  ���н�ǰ�˵��             */
#define L_M_M_InitDegree                30   /*<!  ���н��м�ؽڵ��         */
#define L_M_E_InitDegree                90   /*<!  ���н�ĩ�˵��             */
#define L_B_F_InitDegree                90   /*<!  ����ǰ�˵��             */
#define L_B_M_InitDegree                30   /*<!  �����м�ؽڵ��         */
#define L_B_E_InitDegree                90   /*<!  ����ĩ�˵��             */
#define R_F_F_InitDegree                90   /*<!  ��ǰ��ǰ�˵��             */
#define R_F_M_InitDegree               150   /*<!  ��ǰ���м�ؽڵ��         */
#define R_F_E_InitDegree                90   /*<!  ��ǰ��ǰ�˵��             */
#define R_M_F_InitDegree                90   /*<!  ���н�ǰ�˵��             */
#define R_M_M_InitDegree               150   /*<!  ���н��м�ؽڵ��         */
#define R_M_E_InitDegree                90   /*<!  ���н�ĩ�˵��             */
#define R_B_F_InitDegree                90   /*<!  �Һ��ǰ�˵��             */
#define R_B_M_InitDegree               150   /*<!  �Һ���м�ؽڵ��         */
#define R_B_E_InitDegree                90   /*<!  �Һ��ĩ�˵��             */
#define DRV1_InitDegree                 90   /*<!  ��չ1·���                */
#define DRV2_InitDegree                 90   /*<!  ��չ2·���                */
#define DRV3_InitDegree                 90   /*<!  ��չ3·���                */
#define DRV4_InitDegree                 90   /*<!  ��չ4·���                */

/**
 * @brief Servo MAX degree (����������Ƕ�)
 */
#define L_F_F_MaxDegree                180   /*<!  ��ǰ��ǰ�˵��             */
#define L_F_M_MaxDegree                180   /*<!  ��ǰ���м�ؽڵ��         */
#define L_F_E_MaxDegree                180   /*<!  ��ǰ��ĩ�˵��             */
#define L_M_F_MaxDegree                180   /*<!  ���н�ǰ�˵��             */
#define L_M_M_MaxDegree                180   /*<!  ���н��м�ؽڵ��         */
#define L_M_E_MaxDegree                180   /*<!  ���н�ĩ�˵��             */
#define L_B_F_MaxDegree                180   /*<!  ����ǰ�˵��             */
#define L_B_M_MaxDegree                180   /*<!  �����м�ؽڵ��         */
#define L_B_E_MaxDegree                180   /*<!  ����ĩ�˵��             */
#define R_F_F_MaxDegree                180   /*<!  ��ǰ��ǰ�˵��             */
#define R_F_M_MaxDegree                180   /*<!  ��ǰ���м�ؽڵ��         */
#define R_F_E_MaxDegree                180   /*<!  ��ǰ��ǰ�˵��             */
#define R_M_F_MaxDegree                180   /*<!  ���н�ǰ�˵��             */
#define R_M_M_MaxDegree                180   /*<!  ���н��м�ؽڵ��         */
#define R_M_E_MaxDegree                180   /*<!  ���н�ĩ�˵��             */
#define R_B_F_MaxDegree                180   /*<!  �Һ��ǰ�˵��             */
#define R_B_M_MaxDegree                180   /*<!  �Һ���м�ؽڵ��         */
#define R_B_E_MaxDegree                180   /*<!  �Һ��ĩ�˵��             */
#define DRV1_MaxDegree                 180   /*<!  ��չ1·���                */
#define DRV2_MaxDegree                 180   /*<!  ��չ2·���                */
#define DRV3_MaxDegree                 180   /*<!  ��չ3·���                */
#define DRV4_MaxDegree                 180   /*<!  ��չ4·���                */

/**
 * @brief Servo MAX degree (����������Ƕ�)
 */
#define L_F_F_MinDegree                  0   /*<!  ��ǰ��ǰ�˵��             */
#define L_F_M_MinDegree                  0   /*<!  ��ǰ���м�ؽڵ��         */
#define L_F_E_MinDegree                  0   /*<!  ��ǰ��ĩ�˵��             */
#define L_M_F_MinDegree                  0   /*<!  ���н�ǰ�˵��             */
#define L_M_M_MinDegree                  0   /*<!  ���н��м�ؽڵ��         */
#define L_M_E_MinDegree                  0   /*<!  ���н�ĩ�˵��             */
#define L_B_F_MinDegree                  0   /*<!  ����ǰ�˵��             */
#define L_B_M_MinDegree                  0   /*<!  �����м�ؽڵ��         */
#define L_B_E_MinDegree                  0   /*<!  ����ĩ�˵��             */
#define R_F_F_MinDegree                  0   /*<!  ��ǰ��ǰ�˵��             */
#define R_F_M_MinDegree                  0   /*<!  ��ǰ���м�ؽڵ��         */
#define R_F_E_MinDegree                  0   /*<!  ��ǰ��ǰ�˵��             */
#define R_M_F_MinDegree                  0   /*<!  ���н�ǰ�˵��             */
#define R_M_M_MinDegree                  0   /*<!  ���н��м�ؽڵ��         */
#define R_M_E_MinDegree                  0   /*<!  ���н�ĩ�˵��             */
#define R_B_F_MinDegree                  0   /*<!  �Һ��ǰ�˵��             */
#define R_B_M_MinDegree                  0   /*<!  �Һ���м�ؽڵ��         */
#define R_B_E_MinDegree                  0   /*<!  �Һ��ĩ�˵��             */
#define DRV1_MinDegree                   0   /*<!  ��չ1·���                */
#define DRV2_MinDegree                   0   /*<!  ��չ2·���                */
#define DRV3_MinDegree                   0   /*<!  ��չ3·���                */
#define DRV4_MinDegree                   0   /*<!  ��չ4·���                */


/*
 * @brief ���Կ���
 *
 * @note ����Ҫ�򿪵���ģʽ������#define DEBUG��䣬�򿪺󣬿���ͨ������
 *       �鿴������Ϣ��������Ҫ������ֻ���#define DEBUG ���ע��
 *
 */
#define DEBUG

/*
 * @brief ������Ͷ���
 *
 * @note ��ʹ��7.4V�ĵ�أ���Ҫ����� BATTERY_7V4
 *       ����Ҫʹ��11.1V�ĵ�أ���Ҫ�����µ���ע��
 *
 * @example 7.4V����:
 *                 #define BATTERY_7V4
 *          11.1V����:
 *                 // #define BATTERY_7V4
 */
#define BATTERY_7V4


/*
 * @brief ����ģ��ѡ��(Ĭ�Ͽ�������ʾ����4.0ģ��)
 *
 * @note ��ʹ������4.0ģ���������ر���Ĭ��ʹ������2.0ģ��
 * ���磬ʹ������2.0ģ�飬��� BLUETOOTH_4_0 ����Ϊ 0
 *
 */
#define BLUETOOTH_4_0                     0


#endif /* __SPIDER_NANO_CONFIG_H */
