/**
  ******************************************************************************
  * @file    SpiderXCube/SimpleControlBLE/Inc/SpiderNANOConfig.h
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-AUG-2016
  * @brief   配置文件
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
 * @brief Servo initial degree (舵机初始化角度)
 */
#define L_F_F_InitDegree                90   /*<!  左前脚前端电机             */
#define L_F_M_InitDegree                30   /*<!  左前脚中间关节电机         */
#define L_F_E_InitDegree                90   /*<!  左前脚末端电机             */
#define L_M_F_InitDegree                90   /*<!  左中脚前端电机             */
#define L_M_M_InitDegree                30   /*<!  左中脚中间关节电机         */
#define L_M_E_InitDegree                90   /*<!  左中脚末端电机             */
#define L_B_F_InitDegree                90   /*<!  左后脚前端电机             */
#define L_B_M_InitDegree                30   /*<!  左后脚中间关节电机         */
#define L_B_E_InitDegree                90   /*<!  左后脚末端电机             */
#define R_F_F_InitDegree                90   /*<!  右前脚前端电机             */
#define R_F_M_InitDegree               150   /*<!  右前脚中间关节电机         */
#define R_F_E_InitDegree                90   /*<!  右前脚前端电机             */
#define R_M_F_InitDegree                90   /*<!  右中脚前端电机             */
#define R_M_M_InitDegree               150   /*<!  右中脚中间关节电机         */
#define R_M_E_InitDegree                90   /*<!  右中脚末端电机             */
#define R_B_F_InitDegree                90   /*<!  右后脚前端电机             */
#define R_B_M_InitDegree               150   /*<!  右后脚中间关节电机         */
#define R_B_E_InitDegree                90   /*<!  右后脚末端电机             */
#define DRV1_InitDegree                 90   /*<!  扩展1路电机                */
#define DRV2_InitDegree                 90   /*<!  扩展2路电机                */
#define DRV3_InitDegree                 90   /*<!  扩展3路电机                */
#define DRV4_InitDegree                 90   /*<!  扩展4路电机                */

/**
 * @brief Servo MAX degree (舵机允许最大角度)
 */
#define L_F_F_MaxDegree                180   /*<!  左前脚前端电机             */
#define L_F_M_MaxDegree                180   /*<!  左前脚中间关节电机         */
#define L_F_E_MaxDegree                180   /*<!  左前脚末端电机             */
#define L_M_F_MaxDegree                180   /*<!  左中脚前端电机             */
#define L_M_M_MaxDegree                180   /*<!  左中脚中间关节电机         */
#define L_M_E_MaxDegree                180   /*<!  左中脚末端电机             */
#define L_B_F_MaxDegree                180   /*<!  左后脚前端电机             */
#define L_B_M_MaxDegree                180   /*<!  左后脚中间关节电机         */
#define L_B_E_MaxDegree                180   /*<!  左后脚末端电机             */
#define R_F_F_MaxDegree                180   /*<!  右前脚前端电机             */
#define R_F_M_MaxDegree                180   /*<!  右前脚中间关节电机         */
#define R_F_E_MaxDegree                180   /*<!  右前脚前端电机             */
#define R_M_F_MaxDegree                180   /*<!  右中脚前端电机             */
#define R_M_M_MaxDegree                180   /*<!  右中脚中间关节电机         */
#define R_M_E_MaxDegree                180   /*<!  右中脚末端电机             */
#define R_B_F_MaxDegree                180   /*<!  右后脚前端电机             */
#define R_B_M_MaxDegree                180   /*<!  右后脚中间关节电机         */
#define R_B_E_MaxDegree                180   /*<!  右后脚末端电机             */
#define DRV1_MaxDegree                 180   /*<!  扩展1路电机                */
#define DRV2_MaxDegree                 180   /*<!  扩展2路电机                */
#define DRV3_MaxDegree                 180   /*<!  扩展3路电机                */
#define DRV4_MaxDegree                 180   /*<!  扩展4路电机                */

/**
 * @brief Servo MAX degree (舵机允许最大角度)
 */
#define L_F_F_MinDegree                  0   /*<!  左前脚前端电机             */
#define L_F_M_MinDegree                  0   /*<!  左前脚中间关节电机         */
#define L_F_E_MinDegree                  0   /*<!  左前脚末端电机             */
#define L_M_F_MinDegree                  0   /*<!  左中脚前端电机             */
#define L_M_M_MinDegree                  0   /*<!  左中脚中间关节电机         */
#define L_M_E_MinDegree                  0   /*<!  左中脚末端电机             */
#define L_B_F_MinDegree                  0   /*<!  左后脚前端电机             */
#define L_B_M_MinDegree                  0   /*<!  左后脚中间关节电机         */
#define L_B_E_MinDegree                  0   /*<!  左后脚末端电机             */
#define R_F_F_MinDegree                  0   /*<!  右前脚前端电机             */
#define R_F_M_MinDegree                  0   /*<!  右前脚中间关节电机         */
#define R_F_E_MinDegree                  0   /*<!  右前脚前端电机             */
#define R_M_F_MinDegree                  0   /*<!  右中脚前端电机             */
#define R_M_M_MinDegree                  0   /*<!  右中脚中间关节电机         */
#define R_M_E_MinDegree                  0   /*<!  右中脚末端电机             */
#define R_B_F_MinDegree                  0   /*<!  右后脚前端电机             */
#define R_B_M_MinDegree                  0   /*<!  右后脚中间关节电机         */
#define R_B_E_MinDegree                  0   /*<!  右后脚末端电机             */
#define DRV1_MinDegree                   0   /*<!  扩展1路电机                */
#define DRV2_MinDegree                   0   /*<!  扩展2路电机                */
#define DRV3_MinDegree                   0   /*<!  扩展3路电机                */
#define DRV4_MinDegree                   0   /*<!  扩展4路电机                */


/*
 * @brief 调试开关
 *
 * @note 若需要打开调试模式，则保留#define DEBUG语句，打开后，可以通过串口
 *       查看调试信息；若不需要开启，只需把#define DEBUG 语句注释
 *
 */
#define DEBUG

/*
 * @brief 电池类型定义
 *
 * @note 若使用7.4V的电池，需要定义打开 BATTERY_7V4
 *       若需要使用11.1V的电池，需要把以下的行注释
 *
 * @example 7.4V例子:
 *                 #define BATTERY_7V4
 *          11.1V例子:
 *                 // #define BATTERY_7V4
 */
#define BATTERY_7V4


/*
 * @brief 蓝牙模块选择(默认开启，表示蓝牙4.0模块)
 *
 * @note 若使用蓝牙4.0模块则开启，关闭则默认使用蓝牙2.0模块
 * 例如，使用蓝牙2.0模块，则把 BLUETOOTH_4_0 定义为 0
 *
 */
#define BLUETOOTH_4_0                     0


#endif /* __SPIDER_NANO_CONFIG_H */
