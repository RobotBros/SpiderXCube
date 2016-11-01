/**
  ******************************************************************************
  * @file    SpiderXCube/SimpleControlBLE/Src/system.h
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
 *  @brief  ������
 */
#define SYSTEM_TX_FRAME_BUFFER_SIZE             100
#define SYSTEM_RX_FRAME_BUFFER_SIZE             100

/**
 *  @brief  ֡��
 */
#define SYSTEM_FRAME_LEN                        32

/**
 *  @brief  ֡ͷ
 */
typedef enum _SystemFrameType
{
    RFFrameTypeInvalidRsp         = 0x00,            /*!< δ֪�����                */
    RFFrameTypeJoystickSimple     = 0x03,            /*!< ҡ�˼򵥿�������֡          */
    RFFrameTypeConsoleReq         = 0x05,            /*!< ����̨Consoleͬ������Ƕ�֡ */
    RFFrameTypeStateReq           = 0x0A,            /*!< ��ȡϵͳ��ǰ״̬����        */
    RFFrameTypeStateRsp           = 0x0B,            /*!< ��ȡϵͳ��ǰ״̬��        */
    RFFrameTypeCmdRsp             = 0x0E,            /*!< ���������                */
} SystemFrameType;

#define System_isValidFrameRequest(frameType)   ( ( frameType ) == RFFrameTypeJoystickSimple    || \
                                                  ( frameType ) == RFFrameTypeConsoleReq  || \
                                                  ( frameType ) == RFFrameTypeStateReq )
/**
 *  @brief  ֡�������� 
 */
typedef enum _SystemFrameErrorType
{
    SystemFrameInvalidRequest  = 0x01,          /* δ֪������� */
    SystemFrameInvalidChecksum = 0x02,          /* У�����     */
} SystemFrameErrorType;

/**
 *  @brief  ֡�ṹ��
 */
typedef struct _SystemFrame
{
    SystemFrameType *frameType;                 /* ֡ͷ       */
    u8              *payload;                   /* ֡����     */
    u8              *checksum;                  /* ֡У����   */
    u32              payloadLen;                /* ֡���ݳ��� */   
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
    SystemFrame *rxFrame;                    /* ����֡     */
    SystemFrame *txFrame;                    /* ����֡     */
    u32 errorCode;                           /* ����Ĵ��� */
    u32 warningCode;                         /* ����Ĵ��� */
    SystemState state;                       /* ϵͳ״̬   */
} SystemHandler;

/**
 *  @brief  �������
 */
#define SYSTEM_ERROR_VOLOAGE_LOW             0x00000001   /*!< ��ѹ����                 */
#define SYSTEM_ERROR_VOLOAGE_HIGH            0x00000002   /*!< ��ѹ����                 */
#define SYSTEM_ERROR_CURRENT_HIGH            0x00000004   /*!< ��������(������)       */
#define SYSTEM_ERROR_CURRENT_HIGH_HW         0x00000008   /*!< ��������(Ӳ�����)       */

/**
 *  @brief  �������
 */
#define SYSTEM_WARN_VOLTAGE_LOW              0x00000001
     
#define Mask_On(code, mask)                 (( code ) |= ( mask ))

extern SystemHandler systemHandler;

void System_Init(void);                    /* ��ʼ������         */
void System_FrameHandler(void);            /* ����֡������     */
void System_HealthCheckHandler(void);      /* ������鴦����   */
void System_StateDisplay(void);            /* ״̬��ʾ           */

#endif /* __SYSTEM_H */
