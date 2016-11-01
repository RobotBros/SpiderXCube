/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/nrf24l01.c
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   NRF24L01 驱动实现文件
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
#include "nrf24l01.h"

/* Global varialbles **********************************************************/

NRF24L01Handler nrfHandler;

/* Private varialbles *********************************************************/

static const u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; /*<! 发送地址 */
static const u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; /*<! 接收地址 */

/**
 *@brief  NRF24l01 module TX/RX buffer
  */
//#define NRF24L01_LEN						33
//static u8 NRF24L01_RX_Buff[NRF24L01_LEN];          /*<! IRQ intterupt RX data buffer */
//static u8 NRF24L01_TX_Buff[NRF24L01_LEN];          /*<! IRQ intterupt TX data buffer */

//static u8 *NRF24L01_RX_Buff;                /*<! IRQ intterupt RX data buffer */
//static u8 *NRF24L01_TX_Buff;                /*<! IRQ intterupt TX data buffer */

/* Private function prototypes ************************************************/

static void NRF24L01_EXTI_Init(void);
//static void NRF24L01_Receive_Data_Handle(u8 *data_buf, u8 len);

/* Public functions ***********************************************************/

/**
 *@brief  计算校验和
 *@param  *data_buf: 数据缓冲区
 *@param  len: 数据缓冲区长度
 *@retval 校验和
 */
u8 NRF24L01_Calc_Checksum(u8 *data_buf, u8 len)
{
    u8 sum, cnt;

	/* 计算校验和 */
	sum = 0;
	for( cnt = 0; cnt < (len - 13); cnt ++ )
		sum += data_buf[cnt];
	sum ^= 0xFF;
    
    return sum;
}

/**
 * @brief  初始化24L01的IO口
 * @param  rxBufferPtr:  接收数据缓冲区指针
 * @param  txBufferPtr:  发送数据缓冲区指针
 * @param  rxBufferLen:  接收缓冲区长度 
 * @param  txBufferLen:  发送缓冲区长度 
 * @param  frameLen:     帧长
 * @retval 是否初始化成功. 
 *        TRUE  - 模块已接入并初始化成功
 *        FALSE - 模块未接入, 初始化失败
 */
Boolean NRF24L01_Init(u8 *rxBufferPtr, u8 *txBufferPtr, u16 rxBufferLen, u16 txBufferLen, u16 frameLen)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
    
    /* 设置缓冲区 */
    nrfHandler.rxBuffer = rxBufferPtr;
    nrfHandler.rxBufferLen = rxBufferLen;
    nrfHandler.txBuffer = txBufferPtr;
    nrfHandler.txBufferLen = txBufferLen;
    nrfHandler.frameLen = frameLen;

    /* 使能PB/A端口时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);	 
    	
    /* PA5-CE 推挽 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 	  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	

    /* PB12-CSN 推挽 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 	  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    /* PA4-IRQ 输入 */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 				 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* CE IRQ上拉,禁止NRF24L01工作 */
	GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5); 		
    
    /* CSN下拉 */
	GPIO_ResetBits(GPIOB,GPIO_Pin_12); 
	
    /* SPI2 initial */
    SPI2_Init();
	
    /* NRF24L01 enable */
	NRF24L01_CE = 0;
	NRF24L01_CSN = 1;
    
    if ( NRF24L01_Check() )
    {
        //
        //初始化失败
        //
        
        /* NRF Module is not ready */
        nrfHandler.state = NRF24L01StateNotAttached;
        
        return FALSE;
    }
    else
    {
        //
        //初始化成功
        //
        
        /* NRF EXTI IRQ init */
        NRF24L01_EXTI_Init();
        
        /* NRF Module is ready */
        nrfHandler.state = NRF24L01StateReady;
        
        /* 切换为RX 模式 */
        NRF24L01_RX_Mode();
        
        return TRUE;
    }
}

/**
 *@brief  检测24L01是否存在
 *@param  None
 *@retval 0: 存在; 1: 不存在
 */
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
    
    /* spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）*/
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4);
    
    /* 写入5个字节的地址 */
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);	
    
    /* 读出写入的地址 */
	NRF24L01_Read_Buf(TX_ADDR,buf,5);  
	for ( i = 0; i < 5; i ++ )
        if ( buf[i] != 0XA5 )
            break;
    
	if( i != 5 )
        /* 检测24L01错误 */
        return 1;	
    
    /* 检测到24L01 */
	return 0;
}	 	 

/**
 *@brief  SPI写寄存器
 *@param  reg: 指定寄存器地址
 *@param  value: 写入的值
 *@retval 状态值
 */
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;
    
    /* 使能SPI传输 */
   	NRF24L01_CSN = 0;
    
    /* 发送寄存器号 */
  	status = SPI2_ReadWriteByte(reg);
    
    /* 写入寄存器的值 */
  	SPI2_ReadWriteByte(value);
    
    /* 禁止SPI传输 */
  	NRF24L01_CSN=1;                 
    
  	return status;
}

/**
 *@brief  读取SPI寄存器值
 *@param  reg: 要读的寄存器
 *@retval 状态值
 */
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	 
    
 	NRF24L01_CSN = 0;      
    
  	SPI2_ReadWriteByte(reg);
    
  	reg_val = SPI2_ReadWriteByte(0XFF);
    
  	NRF24L01_CSN = 1;
    
  	return reg_val;
}	

/**
 *@brief  在指定位置读出指定长度的数据
 *@param  reg: 寄存器(位置)
 *@param  *pBuf: 数据指针
 *@retval 返回值,此次读到的状态寄存器值 
 */
u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;
    
    /* Enable NRF24L01 SPI */
  	NRF24L01_CSN = 0;
    
  	status = SPI2_ReadWriteByte(reg);
 	for(u8_ctr = 0; u8_ctr < len; u8_ctr++ )
        pBuf[u8_ctr] = SPI2_ReadWriteByte(0XFF);
    
    /* Disable NRF24L01 SPI */
  	NRF24L01_CSN = 1;
  	
    return status;
}

/**
 *@brief  在指定位置写指定长度的数据
 *@param  reg:   寄存器(位置)
 *@param  *pBuf: 数据指针
 *@param  len:   数据长度
 *@retval 返回值,此次读到的状态寄存器值 
 */
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	
    NRF24L01_CSN = 0;
    
  	status = SPI2_ReadWriteByte(reg);
    
  	for ( u8_ctr = 0; u8_ctr < len; u8_ctr ++ )
        SPI2_ReadWriteByte(*pBuf++); //写入数据	 
    
  	NRF24L01_CSN = 1;
    
  	return status;
}

/**
 *@brief  启动NRF24L01发送一次数据
 *@param  txbuf:待发送数据首地址
 *@retval 发送完成状况
 */
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
    
    /* spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）*/
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);
    
	NRF24L01_CE = 0;
    
    /* 写数据到TX BUF  32个字节 */
  	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, nrfHandler.frameLen);
    
    /* 启动发送 */
 	NRF24L01_CE = 1;
    
    /*等待发送完成 */
	while ( NRF24L01_IRQ != 0);
    
    /* 读取状态寄存器的值 */
	sta = NRF24L01_Read_Reg(STATUS); 	   
    
    /* 清除TX_DS或MAX_RT中断标志 */
	NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS, sta);
    
    /*达到最大重发次数 */
	if ( sta & MAX_TX )
	{
        /* 清除TX FIFO寄存器 */
		NRF24L01_Write_Reg(FLUSH_TX, 0xff); 
        
		return MAX_TX; 
	}
    
	if ( sta & TX_OK )
	{
        //发送完成
		return TX_OK;
	}
    
    /* 其他原因发送失败 */
	return 0xff;
}

/**
 *@brief  启动NRF24L01接收一次数据
 *@param  txbuf:待发送数据首地址
 *@retval 发送完成状况, 0:接收完成; 其他，错误代码
 */
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
    
    //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
    // SPI2_SetSpeed(SPI_BaudRatePrescaler_8); 
    
    /* 读取状态寄存器的值 */
	sta = NRF24L01_Read_Reg(STATUS);
    
    /* 清除TX_DS或MAX_RT中断标志 */
	NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS, sta);
    
    /* 接收到数据 */
	if ( sta & RX_OK )
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, nrfHandler.frameLen);
        
        /* 清除RX FIFO寄存器 */
		NRF24L01_Write_Reg(FLUSH_RX, 0xff);
        
		return 0; 
	}
    
    /* 没收到任何数据 */
	return 1;
}			    

/**
 *@brief  初始化NRF24L01到RX模式
 *   Note: 设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
 *         当CE变高后,即进入RX模式,并可以接收数据了
 *@param  None
 *@retval None
 */
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE = 0; 

    /* 写RX节点地址 */
  	NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); 
    
    /* 使能通道0的自动应答 */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA,0x01);
    
    /* 使能通道0的接收地址 */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_RXADDR, 0x01);  	 
    
    /* 设置RF通信频率 */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 40);
    
    /* 选择通道0的有效数据宽度 */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + RX_PW_P0,nrfHandler.frameLen);
    
    /* 设置TX发射参数,0db增益,2Mbps,低噪声增益开启 */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f);
    
    /* 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 IRQ enable */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0f);

    /* CE为高,进入接收模式 */
  	NRF24L01_CE = 1; 
    
    nrfHandler.state = NRF24L01StateReady;
}

/**
 *@brief  初始化NRF24L01到TX模式
 *   Note: 设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,
 *         选择RF频道,波特率和LNA HCURR PWR_UP,CRC使能
 *         当CE变高后,即进入RX模式,并可以接收数据了		   
 *         CE为高大于10us,则启动发送.	 
 *@param  None
 *@retval None
 */
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE = 0;
    
    /* 写TX节点地址 */
  	NRF24L01_Write_Buf(WRITE_REG_NRF + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);
    
    /* 设置TX节点地址,主要为了使能ACK */
  	NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); 	  

    /* 使能通道0的自动应答 */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA, 0x01);
    
    /* 使能通道0的接收地址 */
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR, 0x01); 
  	
    /* 设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 */
    NRF24L01_Write_Reg(WRITE_REG_NRF + SETUP_RETR, 0x1a);
  	
    /* 设置RF通道为40 */
    NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 40);
  	
    /* 设置TX发射参数,0db增益,2Mbps,低噪声增益开启 */
    NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f);
  	
    /* 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断 */
    NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0e);    
	
    /* CE为高,10us后启动发送 */
    NRF24L01_CE = 1;
    
    nrfHandler.state = NRF24L01StateReady;
}

/**
 *@brief  清空TX缓冲区
 *@param  None
 *@return None
 */
void NRF24L01_ClearTxBuffer(void)
{
	  u8 i;
    for( i = 0; i < nrfHandler.txBufferLen; i ++)
        nrfHandler.txBuffer[i] = 0;
}

/**
 *@brief  清空RX缓冲区
 *@param  None
 *@return None
 */
void NRF24L01_ClearRxBuffer(void)
{
	  u8 i;
    for( i = 0; i < nrfHandler.rxBufferLen; i ++)
        nrfHandler.rxBuffer[i] = 0;
    
    nrfHandler.state = NRF24L01StateReady;
    
    /* 默认进入接收模式 */
    NRF24L01_RX_Mode();
}

/**
 *@biref  系统发送2.4G 数据处理
 *@param  None
 *@return None
 */
void NRF24L01_Send_Frame(void)
{
    /* 检测2.4模块是否空闲 */
	if ( nrfHandler.state == NRF24L01StateReady || 
         nrfHandler.state == NRF24L01StateSendTimeout || 
         nrfHandler.state == NRF24L01StateReceived )
	{
        /* 锁定NRF 2.4G模块为正在发送模式 */
        nrfHandler.state = NRF24L01StateSending;
        
        NRF24L01_TX_Mode();
		HAL_Delay(1);
		NRF24L01_Write_Reg(FLUSH_TX, 0xff);						//Clear TX FIFO buffer 
        
		NRF24L01_CE=0;
  		NRF24L01_Write_Buf(WR_TX_PLOAD, nrfHandler.txBuffer, nrfHandler.frameLen);//写数据到TX BUF  32个字节
 		NRF24L01_CE=1;//启动发送
	}
}

/**
 *@brief  NRF24L01 接收数据中断服务函数
 *@param  None
 *@retval None
 */
void NRF24L01_IRQHandler(void)
{	
	u8 status;
    
	if(EXTI_GetFlagStatus(EXTI_Line4) != RESET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 0)
		{
			status = NRF24L01_Read_Reg(READ_REG_NRF + STATUS);
            
            /* RX 中断 */
			if(status & RX_OK)
			{
                /* Lock NRF Module as receiving status */
                nrfHandler.state = NRF24L01StateReceiving;
                
                /* RX FIFO data buffer to NRF buffer */
				NRF24L01_Read_Buf(RD_RX_PLOAD, nrfHandler.rxBuffer, nrfHandler.frameLen);
                
                for (u8 i = 0; i < 100; i ++);
                
                /* Clear RX FIFO data buffer */
				NRF24L01_Write_Reg(FLUSH_RX, 0xff);							
                
                /* Notify upper layer data has been received */
                nrfHandler.state = NRF24L01StateReceived;
			}
            /* TX data times was the max occured */
			else if (( status & MAX_TX ) > 0)
			{
                /* Clear TX FIFO buffer */
				NRF24L01_Write_Reg(FLUSH_TX, 0xff);											
				//printf("NRF24l01 TX is out of times! \r\n");//Debug TX is out of times
                
                /* Change ro RX mode */
				NRF24L01_RX_Mode();
                
                /* TX timeout */
                nrfHandler.state = NRF24L01StateSendTimeout;
			}
            /* TX data is done(ACK feedback) occured */
			else if((status & TX_OK) > 0)
			{
                /* Clear TX FIFO buffer */
				NRF24L01_Write_Reg(FLUSH_TX, 0xff);
                
                /* Change ro RX mode */
				NRF24L01_RX_Mode();
                
                /* Unlock NRF module */
                nrfHandler.state = NRF24L01StateReady;
			}
			
            /* Clear NRF IRQ flag */
			NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS, status);
		}
        
        /* 清除LINE4上的中断标志位 */
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

/**
 *@brief  NRF 2.4G 外部中断4初始化(NRF IRQ - GPIOA.4)
 *@param  None
 *@retval None
 */
static void NRF24L01_EXTI_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* GPIOA.4 中断线以及中断初始化配置, 下降沿触发 */
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);

  	EXTI_InitStructure.EXTI_Line = NRF24L01_EXTI_Line;  //NRF24L01 IRQ
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

    /* 使能NRF24L01 IRQ所在的外部中断通道 */
	NVIC_InitStructure.NVIC_IRQChannel = NRF24L01_EXTI_IRQn; 
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_24G_Priority;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}
