/**
  ******************************************************************************
  * @file    SpiderXCube/SimpleControlBLE/Src/system.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Jun-2016
  * @brief   ����ϵͳ
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

#include "system.h"

/* Private variables **********************************************************/
static u8 SYSTEM_RX_FRAME_BUFFER[SYSTEM_RX_FRAME_BUFFER_SIZE];
static u8 SYSTEM_TX_FRAME_BUFFER[SYSTEM_TX_FRAME_BUFFER_SIZE];
static SystemState previousState;

/* Global variables ***********************************************************/
SystemHandler systemHandler;

/* Private functions **********************************************************/
static u8 System_calcFrameChecksum(SystemFrame *frame);
static void System_SetServoDegree(u8 *servoDegreeBuffer);
static void System_SendErrorFrame(SystemFrameErrorType errorType);
static void System_GetHealthReport(void);

/**
 *  @brief   ��ʼ������
 */
void System_Init(void)
{
    /* ��ʼ��systemHandler */
    static SystemFrame rxFrame;
    static SystemFrame txFrame;
    
    systemHandler.state = SystemStatePowerUp;
    previousState = SystemStatePowerUp;
    systemHandler.txFrame = &txFrame;
    systemHandler.rxFrame = &rxFrame;
    
    /* ��ʼ��֡�ṹ�� */
    systemHandler.txFrame->frameType = (SystemFrameType *)SYSTEM_TX_FRAME_BUFFER;
    systemHandler.txFrame->payload = SYSTEM_TX_FRAME_BUFFER + 1;
    systemHandler.txFrame->payloadLen = 0;
    systemHandler.txFrame->checksum = SYSTEM_TX_FRAME_BUFFER + SYSTEM_FRAME_LEN - 1;
    
    systemHandler.rxFrame->frameType = (SystemFrameType *)SYSTEM_RX_FRAME_BUFFER;
    systemHandler.rxFrame->payload = SYSTEM_RX_FRAME_BUFFER + 1;
    systemHandler.rxFrame->checksum = SYSTEM_RX_FRAME_BUFFER + SYSTEM_FRAME_LEN - 1;
    systemHandler.rxFrame->payloadLen = 0;
    
    systemHandler.errorCode = 0;
    systemHandler.warningCode = 0;
    
    LED_Init();
    
    /* �������˿ڳ�ʼ�� */
    //BEEP_Init();	
    
    LED_On();
    
    /* 100ms system display */
	TIM7_Int_Init(7199, 999);
    
    /* Systick Init */
    HAL_InitTick(IRQ_SYSTICK_Priority, 0);
    
    /*��Դ������ʼ��*/
    Adc_Init();
    Adc_DMA_Init();
    /*20 ms  DIV=1 AHB1 = 36M*2  7200*200/72(us)=20ms   	
      battery voltage and current sampling             */
	Adc_Timer_Init(7199, 199);
    
    /* ���PWM��ʼ�� */
    Servo_MspInit();
    
    /* �����λ */
    Servo_StateReset();
    
    /* ������ʼ�� */
    if (BT_Init(2, SYSTEM_RX_FRAME_BUFFER, SYSTEM_TX_FRAME_BUFFER))
    {   
        /* �򿪶�������ܵ�Դ */
        Servo_PWRSwitchSet(SERVO_SWITCH_ON);
        
        /* beep 1 time ,gap 240ms */
        BEEP_flash(1,120);
        
        systemHandler.state = SystemStateInitSuccess;
    }
}

/**
 *  @brief   ����֡������
 */
void System_FrameHandler(void)
{
    /* ���ý���֡֡�� */
    USARTx_RX_LEN = SYSTEM_FRAME_LEN;
    
    /* ����Ƿ����ݽ������ */
    if (BT_DataReady)
    {
        /* ����֡���ݳ��� */
        systemHandler.rxFrame->payloadLen = USARTx_RX_CURRENT_LEN - 2;
        
        if ( !System_isValidFrameRequest(*(systemHandler.rxFrame->frameType)) )
        {
            /* δ֪���� */
            System_SendErrorFrame(SystemFrameInvalidRequest);
            
            /* ���û����� */
            USARTx_resetBuffer();
            
            return;
        }
            
        if ( System_calcFrameChecksum(systemHandler.rxFrame) != *(systemHandler.rxFrame->checksum) )
        {
            /* У��ʧ�� */
            System_SendErrorFrame(SystemFrameInvalidChecksum);
            
            /* ���û����� */
            USARTx_resetBuffer();
            
            return;
        }
        
        if ( *(systemHandler.rxFrame->frameType) == RFFrameTypeConsoleReq )
        {
            /*
             * ���ö���Ƕ�����
             */
            System_SetServoDegree(systemHandler.rxFrame->payload);
        }
        
        if ( *(systemHandler.rxFrame->frameType) == RFFrameTypeStateReq )
        {
            /*
             * ��ȡ������������
             */
            System_GetHealthReport();
        }
        
        /* ���û����� */
        USARTx_resetBuffer();
    }
}

/** 
 *  @brief  ������鴦����
 */
void System_HealthCheckHandler(void)
{
    /*--------- ��ѹ��� -----------*/
    if ( voltage < POWER_WARN_VOLTAGE )
    {
        /* ��ѹ���� */
        Mask_On(systemHandler.warningCode, SYSTEM_WARN_VOLTAGE_LOW);
        
        systemHandler.state = SystemStateLowVoltageWarning;
    }
    else if ( voltage < POWER_LOW_VOLTAGE )
    {
        /* ��ѹ���� */
        Mask_On(systemHandler.errorCode, SYSTEM_ERROR_VOLOAGE_LOW);
        
        systemHandler.state = SystemStateLowVoltageError;
    }
    
    /*----------- ������� ---------*/
    if (overCurrentFlag)
    {
        /* Ӳ�������� */
        Mask_On(systemHandler.errorCode, SYSTEM_ERROR_CURRENT_HIGH_HW);
        
        systemHandler.state = SystemStateOverCurrentHwError;
    }
    else if ( current > POWER_OVER_CURRENT )
    {
        /* ��������� */
        Mask_On(systemHandler.errorCode, SYSTEM_ERROR_CURRENT_HIGH);
        
        systemHandler.state = SystemStateOverCurrentError;
    }
}

/**
 *  @brief  ״̬��ʾ
 *  @param  None
 *  @retval None
 */
void System_StateDisplay(void)
{
    if (systemHandler.state != previousState)
    {
        if ( systemHandler.state == SystemStateInitSuccess )
        {
            /* ϵͳ���� */
            timer_led_reset_counter = TIM7_TIME_1S;
            timer_led_enable = TRUE;
            timer_beep_enable = FALSE;
        }
        else if ( systemHandler.state == SystemStateInitFailed )
        {
            /* ��ʼ��ʧ�� */
            timer_led_reset_counter = TIM7_TIME_2S;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateLowVoltageWarning )
        {
            /* ��ѹ���� */
            timer_led_reset_counter = TIM7_TIME_500MS;
            timer_beep_reset_counter = TIM7_TIME_500MS;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateLowVoltageError )
        {
            /* ��ѹ���� */
            timer_led_reset_counter = TIM7_TIME_1S;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateHighVoltageError )
        {
            /* ��ѹ���� */
            timer_led_reset_counter = TIM7_TIME_500MS;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateOverCurrentError )
        {
            /* ��������(������) */
            timer_led_reset_counter = TIM7_TIME_500MS;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateOverCurrentHwError )
        {
            /* ��������(Ӳ�����) */
            timer_led_reset_counter = TIM7_TIME_200MS;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
            
        previousState = systemHandler.state;
    }
}

/** 
 *  @brief  ����֡У����
 *  @param  frame: ֡�ṹ��
 *  @retval У����
 */
static u8 System_calcFrameChecksum(SystemFrame *frame)
{
   u8 checksum = 0;
   u8 i;
   
   checksum += *(frame->frameType);
   
   for ( i = 0; i < frame->payloadLen ; i ++ )
   {
        checksum += frame->payload[i];
   }
   
   checksum ^= 0xff;
   
   return checksum;
}

/**
 *  @brief  ���ö���Ƕ�
 *  @param  servoDegreeBuffer: ����ǶȻ�����
 *  @retval None
 */
static void System_SetServoDegree(u8 *servoDegreeBuffer)
{
    u8 i ;
    
    for ( i = 0; i < SERVO_TOTAL_COUNT; i ++ )
        Servo_SetServoAngle(i, servoDegreeBuffer[i], FALSE);
}

/**
 *  @brief  ���ʹ�������֡
 *  @param  errorType: ��������
 *  @retval None
 */
static void System_SendErrorFrame(SystemFrameErrorType errorType)
{
    u8 i;
    *(systemHandler.txFrame->frameType) = RFFrameTypeInvalidRsp;
    systemHandler.txFrame->payloadLen   = SYSTEM_FRAME_LEN - 2;
    
    *(systemHandler.txFrame->payload)   = (u8)errorType;
    for ( i = 1; i < systemHandler.txFrame->payloadLen; i ++ )
        systemHandler.txFrame->payload[i] = 0x00;
    
    *(systemHandler.txFrame->checksum) = System_calcFrameChecksum(systemHandler.txFrame);
    
    /* �������� */
    USARTx_print(SYSTEM_TX_FRAME_BUFFER, SYSTEM_FRAME_LEN);
}

/**
 *  @brief  ��ȡ��������
 *  @param  None
 *  @retval None
 */
static void System_GetHealthReport(void)
{
    u8 i;
    *(systemHandler.txFrame->frameType) = RFFrameTypeStateRsp;
    systemHandler.txFrame->payloadLen   = SYSTEM_FRAME_LEN - 2;
    
    *(systemHandler.txFrame->payload)     = (voltage & 0xff00) >> 8;
    *(systemHandler.txFrame->payload + 1) = (voltage & 0x00ff);
    *(systemHandler.txFrame->payload + 2) = (current & 0xff00) >> 8;
    *(systemHandler.txFrame->payload + 3) = (current & 0x00ff);
    
    *(systemHandler.txFrame->payload + 4)  = (systemHandler.errorCode & 0xff000000) >> 24;
    *(systemHandler.txFrame->payload + 5)  = (systemHandler.errorCode & 0x00ff0000) >> 16;
    *(systemHandler.txFrame->payload + 6)  = (systemHandler.errorCode & 0x0000ff00) >> 8;
    *(systemHandler.txFrame->payload + 7)  = (systemHandler.errorCode & 0x000000ff);
    
    *(systemHandler.txFrame->payload + 8)  = (systemHandler.warningCode & 0xff000000) >> 24;
    *(systemHandler.txFrame->payload + 9)  = (systemHandler.warningCode & 0x00ff0000) >> 16;
    *(systemHandler.txFrame->payload + 10) = (systemHandler.warningCode & 0x0000ff00) >> 8;
    *(systemHandler.txFrame->payload + 11) = (systemHandler.warningCode & 0x000000ff);
    
    for ( i = 12; i < systemHandler.txFrame->payloadLen; i ++ )
        systemHandler.txFrame->payload[i] = 0x00;
    
    *(systemHandler.txFrame->checksum) = System_calcFrameChecksum(systemHandler.txFrame);
    
    /* �������� */
    USARTx_print(SYSTEM_TX_FRAME_BUFFER, SYSTEM_FRAME_LEN);
}
