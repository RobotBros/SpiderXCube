/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/servo.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   ������ƺ���
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

/* ������ƽǶȻ����� */
static u8 servoNewAngleBuffer[SERVO_TOTAL_COUNT];  /*<! �µĽǶ�ֵ            */
static u8 servoOldAngleBuffer[SERVO_TOTAL_COUNT];  /*<! ǰһ�εĽǶ�ֵ        */

/* ������С�ǶȻ����� */
static u8 servoMinDegreeBuffer[] = {L_F_F_MinDegree, L_F_M_MinDegree, L_F_E_MinDegree,
                                    L_M_F_MinDegree, L_M_M_MinDegree, L_M_E_MinDegree,
                                    L_B_F_MinDegree, L_B_M_MinDegree, L_B_E_MinDegree,
                                    R_F_F_MinDegree, R_F_M_MinDegree, R_F_E_MinDegree,
                                    R_M_F_MinDegree, R_M_M_MinDegree, R_M_E_MinDegree,
                                    R_B_F_MinDegree, R_B_M_MinDegree, R_B_E_MinDegree,
                                    DRV1_MinDegree, DRV2_MinDegree, DRV3_MinDegree,
                                    DRV4_MinDegree};

/* �������ǶȻ����� */
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
 *@brief  PWM��ʼ��
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
 *@brief  ��ʼ�����Ӳ���ײ�����
 *@param  None
 *@retval None
 */
void Servo_MspInit(void)
{
    /* ����ܿ��ص�Դʱ�� */
    GPIO_InitTypeDef GPIO_InitStructure;
    __SERVO_POWER_RCC_ENABLE__();
    GPIO_InitStructure.GPIO_Pin = SERVO_POWER_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SERVO_POWER_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(SERVO_POWER_PORT, SERVO_POWER_PIN);
    
    /* ���PWM��ʼ�� 50Hz(20ms) */
    TIM5_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM3_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV); 
    TIM2_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM8_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM1_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
    TIM4_PWM_Init(PWM_DUTY_ARR, PWM_DUTY_DIV);
}

/**
 *@brief  �����ʼλ�ö�����λ
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
 *@brief  ����ܵ�Դ����
 *@param  switchState:�����Դ����
 *      @arg SERVO_SWITCH_ON: ��
 *      @arg SERVO_SWITCH_OFF: �ر�
 *@retval None
 */
void Servo_PWRSwitchSet(u8 switchState)
{
    if ( SERVO_IS_VALID_SWITCH(switchState) )
    {
        if (switchState)
        {
            /* ʹ��TIMER PWM */
            TIM_Cmd(TIM5, ENABLE);
            TIM_Cmd(TIM3, ENABLE);
            TIM_Cmd(TIM2, ENABLE);
            TIM_Cmd(TIM8, ENABLE);
            TIM_CtrlPWMOutputs(TIM8, ENABLE);
            TIM_Cmd(TIM1, ENABLE);
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
            TIM_Cmd(TIM4, ENABLE);
    
            /* �򿪶����Դ */
            GPIO_WriteBit(SERVO_POWER_PORT, SERVO_POWER_PIN, Bit_SET);
        }
        else
        {
            /* �رն����Դ */
            GPIO_WriteBit(SERVO_POWER_PORT, SERVO_POWER_PIN, Bit_RESET);
            
            /* �ر�TIMER PWM */
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
 *@brief  ����Ƕ�ת�������ƽǶ�Ϊ1�ȣ�����Ϊ90��,���ض�ʱ������ֵ
 *@param  �Ƕ�
 *@retval �������󷵻�0; ��������900~4500����ֵ������0��~180��
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
 *@brief  ���ö��ת���ĽǶ�
 *@param  servo: ������, ȡֵ����Ϊ (0-SERVO_TOTAL_COUNT - 1)
 *@param  angle: ת���Ƕȣ���λ��
 *@param  isReset: �Ƿ��ǳ�ʼ������. TRUE/FALSE.
 *@retval None
 */
void Servo_SetServoAngle(u8 servoId, u8 angle, Boolean isReset)
{
    u8 totalSteps, step = 1;
    s8 sign;
    u16 counter;
    
    if (!Servo_isValidDegree(servoId, angle))
        /* �Ƕȳ�������Χ */
        return;

    /* ���㲽���Ƕ�*/
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
 * @brief  �ж϶���Ƕ��Ƿ�������Χ
 * @brief  servoId: ���id
 * @brief  degree: ��ת�����ĽǶ�
 * @return TRUE - �Ϸ�, FALSE - ��������Χ
 */
inline static Boolean Servo_isValidDegree(u8 servoId, u8 degree)
{
    return (degree < servoMinDegreeBuffer[servoId] || 
            degree > servoMaxDegreeBuffer[servoId] ) ? FALSE : TRUE; 
}

/**
 *@brief  TIM4 PWM���ֳ�ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 */
static void TIM4_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    /* ʹ��GPIO�����AFIO���ù���ģ��ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //ʹ�ܶ�ʱ��5ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);
	
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//������ӳ��

 	/* ��չ4· ��I/O��ʼ�� */
    /* TIM4_CH1/CH2/CH3/CH4 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* ��ʼ��TIM4��Ƶ������ */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	/* ��ʼ��TIM4  PWMģʽ����	
       ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ������������� */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   //�������:TIM����Ƚϼ��Ը�
	
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
 *@brief  TIM5 PWM���ֳ�ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 *@reval  None
 */
static void TIM5_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);
	
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //������ӳ��

 	/* L_B_F_DRV ��L_B_M_DRV��L_B_E_DRV��L_M_F_DRV     I/O��ʼ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* ��ʼ��TIM5��Ƶ������ */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	/* ��ʼ��TIM5  PWMģʽ����
       ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ������������� */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    /* ��ʼ������TIM5 Channel */
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);

    /* ʹ��TIM5 Channelx��CCRx�ϵ�Ԥװ�ؼĴ��� */
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); 						
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_Cmd(TIM5, ENABLE);
}

/**
 *@brief  TIM3 PWM���ֳ�ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 *@reval  None
 */
static void TIM3_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);						 																		 //ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);
	
    //GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//������ӳ��

 	/* L_M_M_DRV ��L_M_E_DRV��L_F_F_DRV��L_F_M_DRV	I/O��ʼ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* ��ʼ��TIM3��Ƶ������ */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* ��ʼ��TIM3  PWMģʽ����	
       ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ������������� */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;   //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   //�������:TIM����Ƚϼ��Ը�
	
    /* ��ʼ������TIM3_CHx */
    TIM_OC4Init(TIM3, &TIM_OCInitStructure); 													
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    /* ʹ��TIM3 CHx��CCRx�ϵ�Ԥװ�ؼĴ��� */
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_Cmd(TIM3, ENABLE); 
}

/**
 *@brief  TIM2 PWM���ֳ�ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 *@reval  None
 */
static void TIM2_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);   //������ӳ��

 	/* L_F_E_DRV ��R_F_E_DRV	I/O��ʼ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;    //TIM2_CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
    /* ��ʼ��TIM2��Ƶ������ */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	/* ��ʼ��TIM2  PWMģʽ����	
       ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ������������� */
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

    /* ʹ��TIM3 CH4��CCR4�ϵ�Ԥװ�ؼĴ��� */
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_Cmd(TIM2, ENABLE);
}

/**
 *@brief  TIM8 PWM���ֳ�ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 *@reval  None
 */
static void TIM8_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

 	/* R_F_M_DRV ��R_F_F_DRV��R_M_E_DRV��R_M_M_DRV	I/O��ʼ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;    //TIM8_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
 
    /* ��ʼ��TIM8��Ƶ������ */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	/* ��ʼ��TIM8  PWMģʽ���� 
       ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ������������� */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    /* �رջ����������ֹSPI1_MOSI���� */
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; 
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);

    /* ʹ��TIM8 CHx��CCRx�ϵ�Ԥװ�ؼĴ��� */
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM8, ENABLE); 
	TIM_Cmd(TIM8, ENABLE);
}

/**
 *@brief  TIM1 PWM���ֳ�ʼ�� 
 *        PWM�����ʼ��
 *@param  arr:�Զ���װֵ
 *@param  psc:ʱ��Ԥ��Ƶ��
 *@reval  None
 */
static void TIM1_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);    //������ӳ��

 	/* R_M_F_DRV ��R_B_E_DRV��R_B_M_DRV��R_B_F_DRV	I/O��ʼ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;  //TIM1_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ��ʼ��TIM1��Ƶ������ */
	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* ��ʼ��TIM1  PWMģʽ����	
       ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ������������� */
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable; //�رջ����������ֹ����
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
