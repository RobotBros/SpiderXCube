#include "spi.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01 20150814  fishcan
//SPI2��ʼ�� ���ó�����ģʽ
//////////////////////////////////////////////////////////////////////////////////
 
//SPI2��ʼ�� PB13-SCK PB14-MISO PB15-MOSI
void SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );									//PORTB CLK enable
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );									//SPI2 CLK	enable
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 												//PB13/14/15 ����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);																		

 	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); 							 //PB13/14/15����

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	 //SPI2����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;													 //SPI2����Ϊ����ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;											 //SPI2����Ϊ�շ�δ8֡ģʽ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;														 //����ͬ��ʱ�ӿ���״̬����Ϊ�͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;													 //����ͬ��ʱ�ӵ�һ�������أ����ݱ��ɼ�
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;															 //NSSʹ��������ƣ���I/O����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		 //������Ԥ��ƵֵΪ16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;										 //���ݴ����MSB��ʼ�����λ
	SPI_InitStructure.SPI_CRCPolynomial = 7;														   //CRC��������
	SPI_Init(SPI2, &SPI_InitStructure);  																   
 
	SPI_Cmd(SPI2, ENABLE); 																								 //SPI2 enable
	SPI2_ReadWriteByte(0xff);																							 //start TX/RX
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);																 //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��
}  

//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   
//SPI_BaudRatePrescaler_8   8��Ƶ   
//SPI_BaudRatePrescaler_16  16��Ƶ  
//SPI_BaudRatePrescaler_256 256��Ƶ 
  
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1&=0XFFC7;
	SPI2->CR1|=SPI_BaudRatePrescaler;																					//����SPI2�ٶ� 
	SPI_Cmd(SPI2,ENABLE); 
} 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) 						//���ָ����SPI��־λ�������:���ͻ���ձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, TxData); 																				 //ͨ������SPIx����һ������
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)					//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); 																			//����ͨ��SPIx������յ�����					    
}































