/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/nrf24l01.h
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   NRF24L01 ��������ͷ�ļ�
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


#ifndef __NRF24L01_H
#define __NRF24L01_H  

#include "spiderx_bsp.h"
#include "stm32f10x.h"

typedef enum _NRF24L01State
{
    NRF24L01StateNotAttached = 0x00,        /*<! 2.4ģ��δ����                */
    NRF24L01StateReady,                     /*<! 2.4ģ�����                  */
    NRF24L01StateSending,                   /*<! 2.4ģ�����ڷ���              */
    NRF24L01StateSendTimeout,               /*<! 2.4ģ�鷢�ͳ�ʱ              */
    NRF24L01StateReceiving,                 /*<! 2.4ģ�����ڽ���              */
    NRF24L01StateReceived,                  /*<! 2.4ģ��������              */
} NRF24L01State;

/**
 *@brief  2.4G���Ƶ�Ԫ�ṹ��
 */
typedef struct _NRF24L01Handler
{
   NRF24L01State state;
   u8 *txBuffer;      /* ���ͻ�����ָ�� */
   u8 *rxBuffer;      /* ���ջ�����ָ�� */
   u16 rxBufferLen;   /* ���ջ��������� */
   u16 txBufferLen;   /* ���ͻ��������� */
   u16 frameLen;      /* ֡�� */
} NRF24L01Handler;

/**
 *@brief NRF24L01�Ĵ�����������
 */
#define READ_REG_NRF                0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define WRITE_REG_NRF               0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD                 0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD                 0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX                    0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX                    0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL                 0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP                         0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG                      0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                                          //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA                       0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR                   0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW                    0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR                  0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH                       0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP                    0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS                      0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                                          //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX  	            	0x10  //�ﵽ����ʹ����ж�
#define TX_OK   	            	0x20  //TX��������ж�
#define RX_OK               		0x40  //���յ������ж�

#define OBSERVE_TX                  0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD                          0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0                  0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1                  0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2                  0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3                  0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4                  0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5                  0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR                     0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0                    0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1                    0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2                    0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3                    0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4                    0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5                    0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define NRF_FIFO_STATUS             0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;

/**
 *@brief 24L01������
 */
#define NRF24L01_CE                PAout(5)     //24L01Ƭѡ�ź�
#define NRF24L01_CSN               PBout(12)    //SPIƬѡ�ź�	   
#define NRF24L01_IRQ               PAin(4)      //IRQ������������

/**
 *@brief 24L01���ͽ������ݿ�ȶ���
 */
#define TX_ADR_WIDTH                5   	    //5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH                5   	    //5�ֽڵĵ�ַ���
//#define TX_PLOAD_WIDTH              32  	    //32�ֽڵ��û����ݿ��
//#define RX_PLOAD_WIDTH              32  	    //32�ֽڵ��û����ݿ��

/**
 *@brief  ����֡֡ͷ
 */
#define NRF_FRAME_PC_HEADER         0xAA        //PC����֡ͷ
#define NRF_FRAME_JOYSTICK_HEADER   0xAB        //ң�ظ˿���֡ͷ
#define NRF_FRAME_SYS_STATUS        0xAC        //��λ������λ��ϵͳ����֡ͷ 
     
/**
 *@brief  IRQ handler
 */
#define EXTIx_Handler               EXTI4_Handler
#define NRF24L01_EXTI_Line          EXTI_Line4
#define NRF24L01_EXTI_IRQn          EXTI4_IRQn

/* Global varialbles **********************************************************/

extern NRF24L01Handler nrfHandler;
									   	   
/* Public functions ***********************************************************/

/**
 * @brief  ��ʼ��24L01��IO��
 * @param  rxBufferPtr:  �������ݻ�����ָ��
 * @param  txBufferPtr:  �������ݻ�����ָ��
 * @param  rxBufferLen:  ���ջ��������� 
 * @param  txBufferLen:  ���ͻ��������� 
 * @param  frameLen:     ֡��
 * @retval �Ƿ��ʼ���ɹ�. 
 *        TRUE  - ģ���ѽ��벢��ʼ���ɹ�
 *        FALSE - ģ��δ����, ��ʼ��ʧ��
 */
Boolean NRF24L01_Init(u8 *rxBufferPtr, u8 *txBufferPtr, 
                      u16 rxBufferLen, u16 txBufferLen, 
                      u16 frameLen);

/**
 *@brief  ��ʼ��NRF24L01��RXģʽ
 *@param  None
 *@retval None
 */
void NRF24L01_RX_Mode(void);

/**
 *@brief  ��ʼ��NRF24L01��TXģʽ
 *@param  None
 *@retval None
 */
void NRF24L01_TX_Mode(void);

/**
 *@brief  ��ָ��λ��дָ�����ȵ�����
 *@param  reg:   �Ĵ���(λ��)
 *@param  *pBuf: ����ָ��
 *@param  len:   ���ݳ���
 *@retval ����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
 */
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 u8s);

/**
 *@brief  ��ָ��λ�ö���ָ�����ȵ�����
 *@param  reg: �Ĵ���(λ��)
 *@param  *pBuf: ����ָ��
 *@retval ����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
 */
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 u8s);

/**
 *@brief  ��ȡSPI�Ĵ���ֵ
 *@param  reg: Ҫ���ļĴ���
 *@retval ״ֵ̬
 */
u8 NRF24L01_Read_Reg(u8 reg);

/**
 *@brief  SPIд�Ĵ���
 *@param  reg: ָ���Ĵ�����ַ
 *@param  value: д���ֵ
 *@retval ״ֵ̬
 */
u8 NRF24L01_Write_Reg(u8 reg, u8 value);

/**
 *@brief  ���24L01�Ƿ����
 *@param  None
 *@retval 0: ����; 1: ������
 */
u8 NRF24L01_Check(void);

/**
 *@brief  ����NRF24L01����һ������
 *@param  txbuf:�����������׵�ַ
 *@retval �������״��
 */
u8 NRF24L01_TxPacket(u8 *txbuf);

/**
 *@brief  ����NRF24L01����һ������
 *@param  txbuf:�����������׵�ַ
 *@retval �������״��, 0:�������; �������������
 */
u8 NRF24L01_RxPacket(u8 *rxbuf);

/**
 *@brief  ���TX������
 *@param  None
 *@return None
 */
void NRF24L01_ClearTxBuffer(void);

/**
 *@brief  ���RX������
 *@param  None
 *@return None
 */
void NRF24L01_ClearRxBuffer(void);

/**
 *@biref  ͨ��2.4G��������. �������ڵ��ô˷���ǰ����nrfHandler
 *@param  None
 *@return None
 */
void NRF24L01_Send_Frame(void);

/**
 *@brief  ����У���
 *@param  *data_buf: ���ݻ�����
 *@param  len: ���ݻ���������
 *@retval У���
 */
u8 NRF24L01_Calc_Checksum(u8 *data_buf, u8 len);

/**
 *@brief  NRF RX IRQ Service routine
 */
#define NRF24L01_IRQHandler       EXTI4_IRQHandler

#endif /* __NRF24L01_H */
