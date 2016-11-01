/**
  ******************************************************************************
  * @file    SpiderXCube/SimpleControlBLE/Src/system.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Jun-2016
  * @brief   控制系统
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
 *  @brief   初始化函数
 */
void System_Init(void)
{
    /* 初始化systemHandler */
    static SystemFrame rxFrame;
    static SystemFrame txFrame;
    
    systemHandler.state = SystemStatePowerUp;
    previousState = SystemStatePowerUp;
    systemHandler.txFrame = &txFrame;
    systemHandler.rxFrame = &rxFrame;
    
    /* 初始化帧结构体 */
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
    
    /* 蜂鸣器端口初始化 */
    //BEEP_Init();	
    
    LED_On();
    
    /* 100ms system display */
	TIM7_Int_Init(7199, 999);
    
    /* Systick Init */
    HAL_InitTick(IRQ_SYSTICK_Priority, 0);
    
    /*电源采样初始化*/
    Adc_Init();
    Adc_DMA_Init();
    /*20 ms  DIV=1 AHB1 = 36M*2  7200*200/72(us)=20ms   	
      battery voltage and current sampling             */
	Adc_Timer_Init(7199, 199);
    
    /* 舵机PWM初始化 */
    Servo_MspInit();
    
    /* 舵机复位 */
    Servo_StateReset();
    
    /* 蓝牙初始化 */
    if (BT_Init(2, SYSTEM_RX_FRAME_BUFFER, SYSTEM_TX_FRAME_BUFFER))
    {   
        /* 打开舵机开关总电源 */
        Servo_PWRSwitchSet(SERVO_SWITCH_ON);
        
        /* beep 1 time ,gap 240ms */
        BEEP_flash(1,120);
        
        systemHandler.state = SystemStateInitSuccess;
    }
}

/**
 *  @brief   蓝牙帧处理函数
 */
void System_FrameHandler(void)
{
    /* 设置接收帧帧长 */
    USARTx_RX_LEN = SYSTEM_FRAME_LEN;
    
    /* 检查是否数据接收完毕 */
    if (BT_DataReady)
    {
        /* 设置帧数据长度 */
        systemHandler.rxFrame->payloadLen = USARTx_RX_CURRENT_LEN - 2;
        
        if ( !System_isValidFrameRequest(*(systemHandler.rxFrame->frameType)) )
        {
            /* 未知请求 */
            System_SendErrorFrame(SystemFrameInvalidRequest);
            
            /* 重置缓冲区 */
            USARTx_resetBuffer();
            
            return;
        }
            
        if ( System_calcFrameChecksum(systemHandler.rxFrame) != *(systemHandler.rxFrame->checksum) )
        {
            /* 校验失败 */
            System_SendErrorFrame(SystemFrameInvalidChecksum);
            
            /* 重置缓冲区 */
            USARTx_resetBuffer();
            
            return;
        }
        
        if ( *(systemHandler.rxFrame->frameType) == RFFrameTypeConsoleReq )
        {
            /*
             * 设置舵机角度请求
             */
            System_SetServoDegree(systemHandler.rxFrame->payload);
        }
        
        if ( *(systemHandler.rxFrame->frameType) == RFFrameTypeStateReq )
        {
            /*
             * 获取健康报告请求
             */
            System_GetHealthReport();
        }
        
        /* 重置缓冲区 */
        USARTx_resetBuffer();
    }
}

/** 
 *  @brief  健康检查处理函数
 */
void System_HealthCheckHandler(void)
{
    /*--------- 电压检测 -----------*/
    if ( voltage < POWER_WARN_VOLTAGE )
    {
        /* 低压警告 */
        Mask_On(systemHandler.warningCode, SYSTEM_WARN_VOLTAGE_LOW);
        
        systemHandler.state = SystemStateLowVoltageWarning;
    }
    else if ( voltage < POWER_LOW_VOLTAGE )
    {
        /* 电压过低 */
        Mask_On(systemHandler.errorCode, SYSTEM_ERROR_VOLOAGE_LOW);
        
        systemHandler.state = SystemStateLowVoltageError;
    }
    
    /*----------- 电流检测 ---------*/
    if (overCurrentFlag)
    {
        /* 硬件检测过流 */
        Mask_On(systemHandler.errorCode, SYSTEM_ERROR_CURRENT_HIGH_HW);
        
        systemHandler.state = SystemStateOverCurrentHwError;
    }
    else if ( current > POWER_OVER_CURRENT )
    {
        /* 软件检测过流 */
        Mask_On(systemHandler.errorCode, SYSTEM_ERROR_CURRENT_HIGH);
        
        systemHandler.state = SystemStateOverCurrentError;
    }
}

/**
 *  @brief  状态显示
 *  @param  None
 *  @retval None
 */
void System_StateDisplay(void)
{
    if (systemHandler.state != previousState)
    {
        if ( systemHandler.state == SystemStateInitSuccess )
        {
            /* 系统正常 */
            timer_led_reset_counter = TIM7_TIME_1S;
            timer_led_enable = TRUE;
            timer_beep_enable = FALSE;
        }
        else if ( systemHandler.state == SystemStateInitFailed )
        {
            /* 初始化失败 */
            timer_led_reset_counter = TIM7_TIME_2S;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateLowVoltageWarning )
        {
            /* 低压警告 */
            timer_led_reset_counter = TIM7_TIME_500MS;
            timer_beep_reset_counter = TIM7_TIME_500MS;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateLowVoltageError )
        {
            /* 低压错误 */
            timer_led_reset_counter = TIM7_TIME_1S;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateHighVoltageError )
        {
            /* 过压错误 */
            timer_led_reset_counter = TIM7_TIME_500MS;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateOverCurrentError )
        {
            /* 电流过大(软件检测) */
            timer_led_reset_counter = TIM7_TIME_500MS;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
        else if ( systemHandler.state == SystemStateOverCurrentHwError )
        {
            /* 电流过大(硬件检测) */
            timer_led_reset_counter = TIM7_TIME_200MS;
            timer_beep_reset_counter = TIM7_TIME_2S;
            
            timer_led_enable = TRUE;
            timer_beep_enable = TRUE;
        }
            
        previousState = systemHandler.state;
    }
}

/** 
 *  @brief  计算帧校验码
 *  @param  frame: 帧结构体
 *  @retval 校验码
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
 *  @brief  设置舵机角度
 *  @param  servoDegreeBuffer: 舵机角度缓冲区
 *  @retval None
 */
static void System_SetServoDegree(u8 *servoDegreeBuffer)
{
    u8 i ;
    
    for ( i = 0; i < SERVO_TOTAL_COUNT; i ++ )
        Servo_SetServoAngle(i, servoDegreeBuffer[i], FALSE);
}

/**
 *  @brief  发送错误命令帧
 *  @param  errorType: 错误类型
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
    
    /* 发送数据 */
    USARTx_print(SYSTEM_TX_FRAME_BUFFER, SYSTEM_FRAME_LEN);
}

/**
 *  @brief  获取健康报告
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
    
    /* 发送数据 */
    USARTx_print(SYSTEM_TX_FRAME_BUFFER, SYSTEM_FRAME_LEN);
}
