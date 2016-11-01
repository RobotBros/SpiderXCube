/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/servo.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   舵机控制函数
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

#include "servo.h"
#include "SpiderNANOConfig.h"
    
/* Private variables **********************************************************/

/* 舵机控制角度缓冲区 */
static u8 servoNewAngleBuffer[SERVO_TOTAL_COUNT];  /*<! 新的角度值            */
static u8 servoOldAngleBuffer[SERVO_TOTAL_COUNT];  /*<! 前一次的角度值        */

/* 允许最小角度缓冲区 */
static u8 servoMinDegreeBuffer[] = {L_F_F_MinDegree, L_F_M_MinDegree, L_F_E_MinDegree,
                                    L_M_F_MinDegree, L_M_M_MinDegree, L_M_E_MinDegree,
                                    L_B_F_MinDegree, L_B_M_MinDegree, L_B_E_MinDegree,
                                    R_F_F_MinDegree, R_F_M_MinDegree, R_F_E_MinDegree,
                                    R_M_F_MinDegree, R_M_M_MinDegree, R_M_E_MinDegree,
                                    R_B_F_MinDegree, R_B_M_MinDegree, R_B_E_MinDegree,
                                    DRV1_MinDegree, DRV2_MinDegree, DRV3_MinDegree,
                                    DRV4_MinDegree};

/* 允许最大角度缓冲区 */
static u8 servoMaxDegreeBuffer[] = {L_F_F_MaxDegree, L_F_M_MaxDegree, L_F_E_MaxDegree,
                                    L_M_F_MaxDegree, L_M_M_MaxDegree, L_M_E_MaxDegree,
                                    L_B_F_MaxDegree, L_B_M_MaxDegree, L_B_E_MaxDegree,
                                    R_F_F_MaxDegree, R_F_M_MaxDegree, R_F_E_MaxDegree,
                                    R_M_F_MaxDegree, R_M_M_MaxDegree, R_M_E_MaxDegree,
                                    R_B_F_MaxDegree, R_B_M_MaxDegree, R_B_E_MaxDegree,
                                    DRV1_MaxDegree, DRV2_MaxDegree, DRV3_MaxDegree,
                                    DRV4_MaxDegree};

/* Global variables ***********************************************************/
/* Private function prototypes ************************************************/

/**
 *@brief  PWM初始化
 */
static void TIM5_PWM_Init(u16 arr, u16 psc);    
static void TIM3_PWM_Init(u16 arr, u16 psc);
static void TIM2_PWM_Init(u16 arr, u16 psc);
static void TIM8_PWM_Init(u16 arr, u16 psc);
static void TIM1_PWM_Init(u16 arr, u16 psc);
static void TIM4_PWM_Init(u16 arr, u16 psc);

static u16 Servo_AngleToCounter(u8 ang);
static Boolean Servo_isValidDegree(u8 servoId, u8 degree);

/* Public functions ***********************************************************/

/**
 *@brief  初始化舵机硬件底层驱动
 *@param  None
 *@retval None
 */
void Servo_MspInit(void)
{
    /* 舵机总开关电源时钟 */
    GPIO_InitTypeDef GPIO_InitStructure;
    __SERVO_POWER_RCC_ENABLE__();
    GPIO_InitStructure.GPIO_Pin = SERVO_POWER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_POWER_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(SERVO_POWER_PORT, SERVO_POWER_PIN);
    
    /* 舵机PWM初始化 50Hz(20ms) */
    TIM5_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM3_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV); 
    TIM2_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM8_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM1_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM4_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
}

/**
 *@brief  舵机初始位置动作复位
 *@param  None
 *@retval None
 */
void Servo_StateReset(void)
{
    u8 i;
    for (i = 0; i < SERVO_TOTAL_COUNT; i ++)
        servoNewAngleBuffer[i] = 0;
        servoOldAngleBuffer[i] = 0;

    Servo_SetServoAngle(L_F_F, L_F_F_InitDegree, TRUE);
    Servo_SetServoAngle(L_F_M, L_F_M_InitDegree, TRUE);
    Servo_SetServoAngle(L_F_E, L_F_E_InitDegree, TRUE);
    
    Servo_SetServoAngle(L_M_F, L_M_F_InitDegree, TRUE);
    Servo_SetServoAngle(L_M_M, L_M_M_InitDegree, TRUE);
    Servo_SetServoAngle(L_M_E, L_M_E_InitDegree, TRUE);
    
    Servo_SetServoAngle(L_B_F, L_B_F_InitDegree, TRUE);
    Servo_SetServoAngle(L_B_M, L_B_M_InitDegree, TRUE);
    Servo_SetServoAngle(L_B_E, L_B_E_InitDegree, TRUE);
    
    Servo_SetServoAngle(R_F_F, R_F_F_InitDegree, TRUE);
    Servo_SetServoAngle(R_F_M, R_F_M_InitDegree, TRUE);
    Servo_SetServoAngle(R_F_E, R_F_E_InitDegree, TRUE);
    
    Servo_SetServoAngle(R_M_F, R_M_F_InitDegree, TRUE);
    Servo_SetServoAngle(R_M_M, R_M_M_InitDegree, TRUE);
    Servo_SetServoAngle(R_M_E, R_M_E_InitDegree, TRUE);
    
    Servo_SetServoAngle(R_B_F, R_B_F_InitDegree, TRUE);
    Servo_SetServoAngle(R_B_M, R_B_M_InitDegree, TRUE);
    Servo_SetServoAngle(R_B_E, R_B_E_InitDegree, TRUE);
    
    Servo_SetServoAngle(DRV1, DRV1_InitDegree, TRUE);
    Servo_SetServoAngle(DRV2, DRV2_InitDegree, TRUE);
    Servo_SetServoAngle(DRV3, DRV3_InitDegree, TRUE);
    Servo_SetServoAngle(DRV4, DRV4_InitDegree, TRUE);
}

/**
 *@brief  舵机总电源开关
 *@param  switchState:舵机电源开关
 *      @arg SERVO_SWITCH_ON: 打开
 *      @arg SERVO_SWITCH_OFF: 关闭
 *@retval None
 */
void Servo_PWRSwitchSet(u8 switchState)
{
    if ( SERVO_IS_VALID_SWITCH(switchState) )
    {
        if (switchState)
        {
            /* 使能TIMER PWM */
            TIM_Cmd(TIM5, ENABLE);
            TIM_Cmd(TIM3, ENABLE);
            TIM_Cmd(TIM2, ENABLE);
            TIM_Cmd(TIM8, ENABLE);
            TIM_CtrlPWMOutputs(TIM8, ENABLE);
            TIM_Cmd(TIM1, ENABLE);
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
            TIM_Cmd(TIM4, ENABLE);
    
            /* 打开舵机电源 */
            GPIO_WriteBit(SERVO_POWER_PORT, SERVO_POWER_PIN, Bit_SET);
        }
        else
        {
            /* 关闭舵机电源 */
            GPIO_WriteBit(SERVO_POWER_PORT, SERVO_POWER_PIN, Bit_RESET);
            
            /* 关闭TIMER PWM */
            TIM_Cmd(TIM5, DISABLE);
            TIM_Cmd(TIM3, DISABLE);
            TIM_Cmd(TIM2, DISABLE);
            TIM_Cmd(TIM8, DISABLE);
            TIM_CtrlPWMOutputs(TIM8, DISABLE);
            TIM_Cmd(TIM1, DISABLE);
            TIM_CtrlPWMOutputs(TIM1, DISABLE);
            TIM_Cmd(TIM4, DISABLE);
        }
    }
}

/**
 *@brief  舵机角度转换，控制角度为1度，回中为90度,返回定时器计数值
 *@param  角度
 *@retval 参数错误返回0; 正常返回900~4500计数值，代表0度~180度
 */
u16 Servo_AngleToCounter(u8 ang)	  
{
    u16 counter;
    if ( ang > 180 )
    {
   		return 0;
    }
    else
    {
   		counter = ang * PWM_DUTY_STEP + PWM_DUTY_ANG_0;
		return counter;
    }
}
       
/**
 *@brief  设置舵机转动的角度
 *@param  servo: 舵机编号, 取值必须为 (0-SERVO_TOTAL_COUNT - 1)
 *@param  angle: 转动角度，单位度
 *@param  isReset: 是否是初始化操作. TRUE/FALSE.
 *@retval None
 */
void Servo_SetServoAngle(u8 servoId, u8 angle, Boolean isReset)
{
    u8 totalSteps, step = 1;
    s8 sign;
    u16 counter;
    
    if (!Servo_isValidDegree(servoId, angle))
        /* 角度超过允许范围 */
        return;

    /* 计算步进角度*/
    if ( isReset )
    {
        step = angle;
        sign = 1;
        totalSteps = angle + 1;
        servoOldAngleBuffer[servoId] = 0;
        servoNewAngleBuffer[servoId] = angle;
    }
    else
    {
        servoOldAngleBuffer[servoId] = servoNewAngleBuffer[servoId];
        servoNewAngleBuffer[servoId] = angle;
        step = 1;
        sign = servoOldAngleBuffer[servoId] > servoNewAngleBuffer[servoId] ? -1 : 1;
        totalSteps = (servoNewAngleBuffer[servoId] - servoOldAngleBuffer[servoId]) * sign;
    }
    
    for ( ; ; )
    {
        counter = Servo_AngleToCounter(servoOldAngleBuffer[servoId] +  sign * step);
        switch(servoId)
        {
            case L_F_F:
                Servo_L_F_F_SetAngle(counter);
                break;
            case L_F_M:
                Servo_L_F_M_SetAngle(counter);
                break;
            case L_F_E:
                Servo_L_F_E_SetAngle(counter);
                break;
            case L_M_F:
                Servo_L_M_F_SetAngle(counter);
                break;
            case L_M_M:
                Servo_L_M_M_SetAngle(counter);
                break;
            case L_M_E:
                Servo_L_M_E_SetAngle(counter);
                break;
            case L_B_F:
                Servo_L_B_F_SetAngle(counter);
                break;
            case L_B_M:
                Servo_L_B_M_SetAngle(counter);
                break;
            case L_B_E:
                Servo_L_B_E_SetAngle(counter);
                break;  
            case R_F_F:
                Servo_R_F_F_SetAngle(counter);
                break;
            case R_F_M:
                Servo_R_F_M_SetAngle(counter);
                break;
            case R_F_E:
                Servo_R_F_E_SetAngle(counter);
                break;
            case R_M_F:
                Servo_R_M_F_SetAngle(counter);
                break;
            case R_M_M:
                Servo_R_M_M_SetAngle(counter);
                break;
            case R_M_E:
                Servo_R_M_E_SetAngle(counter);
                break;
            case R_B_F:
                Servo_R_B_F_SetAngle(counter);
                break;
            case R_B_M:
                Servo_R_B_M_SetAngle(counter);
                break;
            case R_B_E:
                Servo_R_B_E_SetAngle(counter);
                break;
            case DRV1:
                Servo_DRV1_SetAngle(counter);
                break;
            case DRV2:
                Servo_DRV2_SetAngle(counter);
                break;
            case DRV3:
                Servo_DRV3_SetAngle(counter);
                break;
            case DRV4:
                Servo_DRV4_SetAngle(counter);
                break;
            default:
                return;
        }
        
        step ++;
        
        if (step >= totalSteps)
            break;
        
        HAL_Delay(SERVO_1_DEGREE_MS);
    }
}

/* Private functions **********************************************************/

/**
 * @brief  判断舵机角度是否在允许范围
 * @brief  servoId: 舵机id
 * @brief  degree: 需转动到的角度
 * @return TRUE - 合法, FALSE - 超过允许范围
 */
inline static Boolean Servo_isValidDegree(u8 servoId, u8 degree)
{
    return (degree < servoMinDegreeBuffer[servoId] || 
            degree > servoMaxDegreeBuffer[servoId] ) ? FALSE : TRUE; 
}

/**
 *@brief  TIM4 PWM部分初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 */
static void TIM4_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    /* 使能GPIO外设和AFIO复用功能模块时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //使能定时器5时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);
	
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//部分重映射

 	/* 扩展4路 ，I/O初始化 */
    /* TIM4_CH1/CH2/CH3/CH4 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* 初始化TIM4，频率设置 */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	/* 初始化TIM4  PWM模式设置	
       选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低 */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   //输出极性:TIM输出比较极性高
	
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM4, ENABLE);
}

/**
 *@brief  TIM5 PWM部分初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 *@reval  None
 */
static void TIM5_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);
	
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //部分重映射

 	/* L_B_F_DRV ，L_B_M_DRV，L_B_E_DRV，L_M_F_DRV     I/O初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* 初始化TIM5，频率设置 */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	/* 初始化TIM5  PWM模式设置
       选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低 */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    /* 初始化外设TIM5 Channel */
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);

    /* 使能TIM5 Channelx在CCRx上的预装载寄存器 */
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); 						
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM5, ENABLE);
}

/**
 *@brief  TIM3 PWM部分初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 *@reval  None
 */
static void TIM3_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);						 																		 //使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);
	
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//部分重映射

 	/* L_M_M_DRV ，L_M_E_DRV，L_F_F_DRV，L_F_M_DRV	I/O初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* 初始化TIM3，频率设置 */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* 初始化TIM3  PWM模式设置	
       选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低 */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   //输出极性:TIM输出比较极性高
	
    /* 初始化外设TIM3_CHx */
    TIM_OC4Init(TIM3, &TIM_OCInitStructure); 													
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    /* 使能TIM3 CHx在CCRx上的预装载寄存器 */
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_Cmd(TIM3, ENABLE); 
}

/**
 *@brief  TIM2 PWM部分初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 *@reval  None
 */
static void TIM2_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);   //部分重映射

 	/* L_F_E_DRV ，R_F_E_DRV	I/O初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;    //TIM2_CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* 初始化TIM2，频率设置 */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* 初始化TIM2  PWM模式设置	
       选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低 */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);

    /* 使能TIM3 CH4在CCR4上的预装载寄存器 */
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_Cmd(TIM2, ENABLE);
}

/**
 *@brief  TIM8 PWM部分初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 *@reval  None
 */
static void TIM8_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

 	/* R_F_M_DRV ，R_F_F_DRV，R_M_E_DRV，R_M_M_DRV	I/O初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;    //TIM8_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
 
    /* 初始化TIM8，频率设置 */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	/* 初始化TIM8  PWM模式设置 
       选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低 */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    /* 关闭互补输出，防止SPI1_MOSI干扰 */
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; 
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);

    /* 使能TIM8 CHx在CCRx上的预装载寄存器 */
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM8, ENABLE); 
	TIM_Cmd(TIM8, ENABLE);
}

/**
 *@brief  TIM1 PWM部分初始化 
 *        PWM输出初始化
 *@param  arr:自动重装值
 *@param  psc:时钟预分频数
 *@reval  None
 */
static void TIM1_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);    //部分重映射

 	/* R_M_F_DRV ，R_B_E_DRV，R_B_M_DRV，R_B_F_DRV	I/O初始化 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;  //TIM1_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 初始化TIM1，频率设置 */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* 初始化TIM1  PWM模式设置	
       选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低 */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //关闭互补输出，防止干扰
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM1, ENABLE); 
	TIM_Cmd(TIM1, ENABLE);
}
