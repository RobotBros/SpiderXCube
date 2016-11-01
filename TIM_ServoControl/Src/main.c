/**
  ******************************************************************************
  * @file    SpiderXCube/TIM_ServoControl/Src/main.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   舵机控制例程
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

#include "main.h"

/**
 *@brief  Main function
 *@param  None
 *@retval None
 */
int main(void)
{   
    LED_Init();
    
    /* Systick Init */
    HAL_InitTick(IRQ_SYSTICK_Priority, 0);
    
    /* 舵机PWM初始化 */
    Servo_MspInit();
    
    while(1)
    {
        /*闪烁LED5次，间隔500ms */
        LED_flash(5, 500);
        
        /* 转动L_F_M 舵机 至30度位置 */
        Servo_SetServoAngle(L_F_M, 30, FALSE);
        
        /* 转动 R_B_M 舵机 至30度位置 */
        Servo_SetServoAngle(R_B_M, 30, FALSE);
        
        /* 闪烁LED5次，间隔1s */
        LED_flash(5, 1000);
        
        /* 转动L_F_M 舵机 至90度位置 */
        Servo_SetServoAngle(L_F_M, 90, FALSE);
        
        /* 转动 R_B_M 舵机 至90度位置 */
        Servo_SetServoAngle(R_B_M, 90, FALSE);
    }
}

