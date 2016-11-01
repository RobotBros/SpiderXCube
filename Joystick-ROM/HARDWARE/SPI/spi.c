#include "spi.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01 20150814  fishcan
//SPI2初始化 配置成主机模式
//////////////////////////////////////////////////////////////////////////////////
 
//SPI2初始化 PB13-SCK PB14-MISO PB15-MOSI
void SPI2_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );									//PORTB CLK enable
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE );									//SPI2 CLK	enable
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 												//PB13/14/15 复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);																		

 	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15); 							 //PB13/14/15上拉

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	 //SPI2设置为双线双线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;													 //SPI2设置为主机模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;											 //SPI2设置为收发未8帧模式
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;														 //串行同步时钟空闲状态设置为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;													 //串行同步时钟第一个跳变沿，数据被采集
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;															 //NSS使用软件控制，即I/O控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		 //波特率预分频值为16
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;										 //数据处理从MSB开始，最高位
	SPI_InitStructure.SPI_CRCPolynomial = 7;														   //CRC计算设置
	SPI_Init(SPI2, &SPI_InitStructure);  																   
 
	SPI_Cmd(SPI2, ENABLE); 																								 //SPI2 enable
	SPI2_ReadWriteByte(0xff);																							 //start TX/RX
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);																 //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）
}  

//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频  
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI2->CR1&=0XFFC7;
	SPI2->CR1|=SPI_BaudRatePrescaler;																					//设置SPI2速度 
	SPI_Cmd(SPI2,ENABLE); 
} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) 						//检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, TxData); 																				 //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)					//检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); 																			//返回通过SPIx最近接收的数据					    
}































