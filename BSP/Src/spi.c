/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/spi.c
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   SPI��غ�������
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

#include "spi.h"

/* Public functions ***********************************************************/

/** 
 *@brief  SPI2��ʼ�� PB13-SCK PB14-MISO PB15-MOSI
 */
void SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);																		
    
    /* PB13/14/15���� */
 	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    
    /* SPI2����Ϊ˫��˫��ȫ˫�� */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    
    /* SPI2����Ϊ����ģʽ */
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    
    /* SPI2����Ϊ�շ�Ϊ8֡ģʽ */
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    
    /* ����ͬ��ʱ�ӿ���״̬����Ϊ�͵�ƽ */
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    
    /* ����ͬ��ʱ�ӵ�һ�������أ����ݱ��ɼ� */
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    
    /* NSSʹ��������ƣ���I/O���� */
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    
    /* ������Ԥ��ƵֵΪ16 */
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    
    /* ���ݴ����MSB��ʼ�����λ */
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    
    /* CRC�������� */
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);  																   
    
	SPI_Cmd(SPI2, ENABLE);
	SPI2_ReadWriteByte(0xff); //start TX/RX
    
    /* spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��*/
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);
}  

/**
 *@brief  SPI �ٶ����ú���
 *@param  SpeedSet:
 *        SPI_BaudRatePrescaler_2   2��Ƶ   
 *        SPI_BaudRatePrescaler_8   8��Ƶ   
 *        SPI_BaudRatePrescaler_16  16��Ƶ  
 *        SPI_BaudRatePrescaler_256 256��Ƶ 
 */
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1 &= 0XFFC7;
    /* ����SPI2�ٶ� */
	SPI2->CR1 |= SPI_BaudRatePrescaler;
	SPI_Cmd(SPI2, ENABLE); 
} 

/**
 *@brief  SPIx ��дһ���ֽ�
 *@param  TxData:Ҫд����ֽ�
 *@retval ��ȡ�����ֽ�
 */
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry = 0;
    
    /* ���ָ����SPI��־λ�������:���ͻ���ձ�־λ */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) 
    {
		retry++;
		if(retry>200)return 0;
    }
    
    /* ͨ������SPIx����һ������ */
	SPI_I2S_SendData(SPI2, TxData);
	retry = 0;
    
    /* ���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
		retry ++;
		if(retry > 200)
            return 0;
    }
    
    /* ����ͨ��SPIx������յ����� */
	return SPI_I2S_ReceiveData(SPI2);
}
