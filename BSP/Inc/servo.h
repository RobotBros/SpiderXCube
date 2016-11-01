/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/servo.h
  * @author  Fishcan, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   ������ƺ���
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, ROBOTBROS.  SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 Robotbros.Inc </center></h2>
  ******************************************************************************
  */

#ifndef __SERVO_H
#define __SERVO_H

#include "spiderx_bsp.h"


/* Public typedef *************************************************************/

#define __SERVO_POWER_RCC_ENABLE__()    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE)
#define SERVO_POWER_PORT                GPIOC
#define SERVO_POWER_PIN                 GPIO_Pin_15

#define SERVO_SWITCH_ON                 1
#define SERVO_SWITCH_OFF                0

#define SERVO_IS_VALID_SWITCH(x)        (( x ) == SERVO_SWITCH_ON || \
                                         ( x ) == SERVO_SWITCH_OFF )

#define SERVO_1_DEGREE_MS               4  /*<! ת��1����Ҫ��ʱ��(ms),��ֵ��Ҫ���ݶ���������� */

/**
 *@brief  ������
 *
 *   �����д L: Left;  R: Right;  F: Front;  M: Middle;  E: End
 *
 *
 *                                   +--------+
 *                                   |  HEAD  |
 *                                   +--------+
 *       2           1           0   |        |   9           10         11 
 *   +-------+   +-------+   +-------+        +-------+   +-------+   +-------+  
 *   | L_F_E |---| L_F_M |---| L_F_F |        | R_F_F |---| R_F_M |---| R_F_E |
 *   +-------+   +-------+   +-------+        +-------+   +-------+   +-------+   
 *       5           4           3   |        |   12          13         14    
 *   +-------+   +-------+   +-------+        +-------+   +-------+   +-------+  
 *   | L_M_E |---| L_M_M |---| L_M_F |        | R_M_F |---| R_M_M |---| R_M_E |
 *   +-------+   +-------+   +-------+        +-------+   +-------+   +-------+  
 *       8           7           6   |        |   15          16         17  
 *   +-------+   +-------+   +-------+        +-------+   +-------+   +-------+  
 *   | L_B_E |---| L_B_M |---| L_B_F |        | R_B_F |---| R_B_M |---| R_B_E |
 *   +-------+   +-------+   +-------+        +-------+   +-------+   +-------+  
 *                                   |        |
 *                                   |        |
 *                     18          19          20          21   
 *                 +========+  +========+  +========+  +========+
 *                 |  DRV1  |  |  DRV2  |  |  DRV3  |  |  DRV4  |
 *                 +========+  +========+  +========+  +========+
 *
 */
#define L_F_F                         0x00   /*<!  ��ǰ��ǰ�˵��             */
#define L_F_M                         0x01   /*<!  ��ǰ���м�ؽڵ��         */
#define L_F_E                         0x02   /*<!  ��ǰ��ĩ�˵��             */

#define L_M_F                         0x03   /*<!  ���н�ǰ�˵��             */
#define L_M_M                         0x04   /*<!  ���н��м�ؽڵ��         */
#define L_M_E                         0x05   /*<!  ���н�ĩ�˵��             */
    
#define L_B_F                         0x06   /*<!  ����ǰ�˵��             */
#define L_B_M                         0x07   /*<!  �����м�ؽڵ��         */
#define L_B_E                         0x08   /*<!  ����ĩ�˵��             */
    
#define R_F_F                         0x09   /*<!  ��ǰ��ǰ�˵��             */
#define R_F_M                         0x0A   /*<!  ��ǰ���м�ؽڵ��         */
#define R_F_E                         0x0B   /*<!  ��ǰ��ǰ�˵��             */
    
#define R_M_F                         0x0C   /*<!  ���н�ǰ�˵��             */
#define R_M_M                         0x0D   /*<!  ���н��м�ؽڵ��         */
#define R_M_E                         0x0E   /*<!  ���н�ĩ�˵��             */
    
#define R_B_F                         0x0F   /*<!  �Һ��ǰ�˵��             */
#define R_B_M                         0x10   /*<!  �Һ���м�ؽڵ��         */
#define R_B_E                         0x11   /*<!  �Һ��ĩ�˵��             */
     
#define DRV1                          0x12   /*<!  ��չ1·���                */
#define DRV2                          0x13   /*<!  ��չ2·���                */
#define DRV3                          0x14   /*<!  ��չ3·���                */
#define DRV4                          0x15   /*<!  ��չ4���                  */
     
/**
 *@brief  ��Դ�ܽ�
 */
#define SERVO_POWER_SWITCH            PCout(15)

/* Public macro ***************************************************************/

/**
 *@brief  ����Ƕ����ú����궨��
 */
#define Servo_L_F_F_SetAngle(counter)     TIM_SetCompare3(TIM3, (counter))
#define Servo_L_F_M_SetAngle(counter)     TIM_SetCompare4(TIM3, (counter))
#define Servo_L_F_E_SetAngle(counter)     TIM_SetCompare3(TIM2, (counter))

#define Servo_L_M_F_SetAngle(counter)     TIM_SetCompare4(TIM5, (counter))
#define Servo_L_M_M_SetAngle(counter)     TIM_SetCompare1(TIM3, (counter))
#define Servo_L_M_E_SetAngle(counter)     TIM_SetCompare2(TIM3, (counter))

#define Servo_L_B_F_SetAngle(counter)     TIM_SetCompare1(TIM5, (counter))
#define Servo_L_B_M_SetAngle(counter)     TIM_SetCompare2(TIM5, (counter))
#define Servo_L_B_E_SetAngle(counter)     TIM_SetCompare3(TIM5, (counter))

#define Servo_R_F_F_SetAngle(counter)     TIM_SetCompare2(TIM8, (counter))
#define Servo_R_F_M_SetAngle(counter)     TIM_SetCompare1(TIM8, (counter))
#define Servo_R_F_E_SetAngle(counter)     TIM_SetCompare4(TIM2, (counter))

#define Servo_R_M_F_SetAngle(counter)     TIM_SetCompare1(TIM1, (counter))
#define Servo_R_M_M_SetAngle(counter)     TIM_SetCompare4(TIM8, (counter))
#define Servo_R_M_E_SetAngle(counter)     TIM_SetCompare3(TIM8, (counter))

#define Servo_R_B_F_SetAngle(counter)     TIM_SetCompare4(TIM1, (counter))
#define Servo_R_B_M_SetAngle(counter)     TIM_SetCompare3(TIM1, (counter))
#define Servo_R_B_E_SetAngle(counter)     TIM_SetCompare2(TIM1, (counter))

#define Servo_DRV1_SetAngle(counter)      TIM_SetCompare1(TIM4, (counter))
#define Servo_DRV2_SetAngle(counter)      TIM_SetCompare2(TIM4, (counter))
#define Servo_DRV3_SetAngle(counter)      TIM_SetCompare3(TIM4, (counter))
#define Servo_DRV4_SetAngle(counter)      TIM_SetCompare4(TIM4, (counter))

#define PWM_DUTY_ARR                      54999         /*<! PWM���� = 55000*25/72000 = 19.0972ms               */
#define PWM_DUTY_DIV                      24            /*<! PWMƵ��DIVϵ��25                                   */
#define PWM_DUTY_MAX                      7200
#define PWM_DUTY_ANG_180                  7200          /*<! PWM Duty                                           */
#define PWM_DUTY_ANG_0                    1440          /*<! PWM Duty                                           */
#define PWM_DUTY_STEP                     32            /*<! PWM 180�� /(7200 - 1440) = 0.03125�� / ��ʱ * 32 
                                                             Լ���� ÿ��1�ȣ�so n�� counter = angle*32+1440     */

#define SERVO_TOTAL_COUNT                 22            /*<! ������� */

/* Public variables ***********************************************************/
/* Public functions ***********************************************************/

/**
 *@brief  ��ʼ�����Ӳ���ײ�����
 *@param  None
 *@retval None
 */
void Servo_MspInit(void);

/**
 *@brief  �����ʼλ�ö�����λ
 *@param  None
 *@retval None
 */
void Servo_StateReset(void);

/**
 *@brief  Servo Handler init
 */
void Servo_HandlerInit(void);

/**
 *@brief  ����Ƕ�����
 *@param  ������
 *@param  ת���Ƕ�
 *@param  �Ƿ��ǳ�ʼ������
 *@retval None
 */
void Servo_SetServoAngle(u8 servoId, u8 angle, Boolean isReset);

/**
 *@brief  ����ܵ�Դ����
 *@param  switch:�����Դ����
 *      @arg SERVO_SWITCH_ON: ��
 *      @arg SERVO_SWITCH_OFF: �ر�
 *@retval None
 */
void Servo_PWRSwitchSet(u8 switchState);

#endif /* __SERVO_H */
