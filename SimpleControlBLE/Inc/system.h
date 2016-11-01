/**
  ******************************************************************************
  * @file    SpiderXCube/SimpleControlBLE/Src/system.h
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

#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "spiderx_bsp.h"


#if (BLUETOOTH_4_0 == 1)
    #include "bt4.h"
#else
    #include "bt2.h"
#endif

#include "power.h"
#include "servo.h"

/*
 *  @brief  缓冲区
 */
#define SYSTEM_TX_FRAME_BUFFER_SIZE             100
#define SYSTEM_RX_FRAME_BUFFER_SIZE             100

/**
 *  @brief  帧长
 */
#define SYSTEM_FRAME_LEN                        32

/**
 *  @brief  帧头
 */
typedef enum _SystemFrameType
{
    RFFrameTypeInvalidRsp         = 0x00,            /*!< 未知命令答复                */
    RFFrameTypeJoystickSimple     = 0x03,            /*!< 摇杆简单控制数据帧          */
    RFFrameTypeConsoleReq         = 0x05,            /*!< 控制台Console同步电机角度帧 */
    RFFrameTypeStateReq           = 0x0A,            /*!< 获取系统当前状态请求        */
    RFFrameTypeStateRsp           = 0x0B,            /*!< 获取系统当前状态答复        */
    RFFrameTypeCmdRsp             = 0x0E,            /*!< 命令请求答复                */
} SystemFrameType;

#define System_isValidFrameRequest(frameType)   ( ( frameType ) == RFFrameTypeJoystickSimple    || \
                                                  ( frameType ) == RFFrameTypeConsoleReq  || \
                                                  ( frameType ) == RFFrameTypeStateReq )
/**
 *  @brief  帧错误类型 
 */
typedef enum _SystemFrameErrorType
{
    SystemFrameInvalidRequest  = 0x01,          /* 未知命令错误 */
    SystemFrameInvalidChecksum = 0x02,          /* 校验错误     */
} SystemFrameErrorType;

/**
 *  @brief  帧结构体
 */
typedef struct _SystemFrame
{
    SystemFrameType *frameType;                 /* 帧头       */
    u8              *payload;                   /* 帧数据     */
    u8              *checksum;                  /* 帧校验码   */
    u32              payloadLen;                /* 帧数据长度 */   
} SystemFrame;

typedef enum _SystemState
{
    SystemStatePowerUp,
    SystemStateInitSuccess,
    SystemStateInitFailed,
    SystemStateLowVoltageWarning,
    SystemStateLowVoltageError,
    SystemStateHighVoltageError,
    SystemStateOverCurrentError,
    SystemStateOverCurrentHwError,
} SystemState;

typedef struct _SystemHandler
{
    SystemFrame *rxFrame;                    /* 接收帧     */
    SystemFrame *txFrame;                    /* 发送帧     */
    u32 errorCode;                           /* 错误寄存器 */
    u32 warningCode;                         /* 警告寄存器 */
    SystemState state;                       /* 系统状态   */
} SystemHandler;

/**
 *  @brief  错误代码
 */
#define SYSTEM_ERROR_VOLOAGE_LOW             0x00000001   /*!< 电压过低                 */
#define SYSTEM_ERROR_VOLOAGE_HIGH            0x00000002   /*!< 电压过大                 */
#define SYSTEM_ERROR_CURRENT_HIGH            0x00000004   /*!< 电流过大(软件检测)       */
#define SYSTEM_ERROR_CURRENT_HIGH_HW         0x00000008   /*!< 电流过大(硬件检测)       */

/**
 *  @brief  警告代码
 */
#define SYSTEM_WARN_VOLTAGE_LOW              0x00000001
     
#define Mask_On(code, mask)                 (( code ) |= ( mask ))

extern SystemHandler systemHandler;

void System_Init(void);                    /* 初始化函数         */
void System_FrameHandler(void);            /* 蓝牙帧处理函数     */
void System_HealthCheckHandler(void);      /* 健康检查处理函数   */
void System_StateDisplay(void);            /* 状态显示           */

#endif /* __SYSTEM_H */
