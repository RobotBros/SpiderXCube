/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/nrf24l01.c
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   NRF24L01 ����ʵ���ļ�
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

static const u8 TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; /*<! ���͵�ַ */
static const u8 RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; /*<! ���յ�ַ */

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
 *@brief  ����У���
 *@param  *data_buf: ���ݻ�����
 *@param  len: ���ݻ���������
 *@retval У���
 */
u8 NRF24L01_Calc_Checksum(u8 *data_buf, u8 len)
{
    u8 sum, cnt;

	/* ����У��� */
	sum = 0;
	for( cnt = 0; cnt < (len - 13); cnt ++ )
		sum += data_buf[cnt];
	sum ^= 0xFF;
    
    return sum;
}

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
Boolean NRF24L01_Init(u8 *rxBufferPtr, u8 *txBufferPtr, u16 rxBufferLen, u16 txBufferLen, u16 frameLen)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
    
    /* ���û����� */
    nrfHandler.rxBuffer = rxBufferPtr;
    nrfHandler.rxBufferLen = rxBufferLen;
    nrfHandler.txBuffer = txBufferPtr;
    nrfHandler.txBufferLen = txBufferLen;
    nrfHandler.frameLen = frameLen;

    /* ʹ��PB/A�˿�ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);	 
    	
    /* PA5-CE ���� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 	  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 	GPIO_Init(GPIOA, &GPIO_InitStructure);	

    /* PB12-CSN ���� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 	  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
  
    /* PA4-IRQ ���� */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 				 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* CE IRQ����,��ֹNRF24L01���� */
	GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5); 		
    
    /* CSN���� */
	GPIO_ResetBits(GPIOB,GPIO_Pin_12); 
	
    /* SPI2 initial */
    SPI2_Init();
	
    /* NRF24L01 enable */
	NRF24L01_CE = 0;
	NRF24L01_CSN = 1;
    
    if ( NRF24L01_Check() )
    {
        //
        //��ʼ��ʧ��
        //
        
        /* NRF Module is not ready */
        nrfHandler.state = NRF24L01StateNotAttached;
        
        return FALSE;
    }
    else
    {
        //
        //��ʼ���ɹ�
        //
        
        /* NRF EXTI IRQ init */
        NRF24L01_EXTI_Init();
        
        /* NRF Module is ready */
        nrfHandler.state = NRF24L01StateReady;
        
        /* �л�ΪRX ģʽ */
        NRF24L01_RX_Mode();
        
        return TRUE;
    }
}

/**
 *@brief  ���24L01�Ƿ����
 *@param  None
 *@retval 0: ����; 1: ������
 */
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
    
    /* spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��*/
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4);
    
    /* д��5���ֽڵĵ�ַ */
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);	
    
    /* ����д��ĵ�ַ */
	NRF24L01_Read_Buf(TX_ADDR,buf,5);  
	for ( i = 0; i < 5; i ++ )
        if ( buf[i] != 0XA5 )
            break;
    
	if( i != 5 )
        /* ���24L01���� */
        return 1;	
    
    /* ��⵽24L01 */
	return 0;
}	 	 

/**
 *@brief  SPIд�Ĵ���
 *@param  reg: ָ���Ĵ�����ַ
 *@param  value: д���ֵ
 *@retval ״ֵ̬
 */
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;
    
    /* ʹ��SPI���� */
   	NRF24L01_CSN = 0;
    
    /* ���ͼĴ����� */
  	status = SPI2_ReadWriteByte(reg);
    
    /* д��Ĵ�����ֵ */
  	SPI2_ReadWriteByte(value);
    
    /* ��ֹSPI���� */
  	NRF24L01_CSN=1;                 
    
  	return status;
}

/**
 *@brief  ��ȡSPI�Ĵ���ֵ
 *@param  reg: Ҫ���ļĴ���
 *@retval ״ֵ̬
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
 *@brief  ��ָ��λ�ö���ָ�����ȵ�����
 *@param  reg: �Ĵ���(λ��)
 *@param  *pBuf: ����ָ��
 *@retval ����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
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
 *@brief  ��ָ��λ��дָ�����ȵ�����
 *@param  reg:   �Ĵ���(λ��)
 *@param  *pBuf: ����ָ��
 *@param  len:   ���ݳ���
 *@retval ����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
 */
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	
    NRF24L01_CSN = 0;
    
  	status = SPI2_ReadWriteByte(reg);
    
  	for ( u8_ctr = 0; u8_ctr < len; u8_ctr ++ )
        SPI2_ReadWriteByte(*pBuf++); //д������	 
    
  	NRF24L01_CSN = 1;
    
  	return status;
}

/**
 *@brief  ����NRF24L01����һ������
 *@param  txbuf:�����������׵�ַ
 *@retval �������״��
 */
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
    
    /* spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��*/
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);
    
	NRF24L01_CE = 0;
    
    /* д���ݵ�TX BUF  32���ֽ� */
  	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, nrfHandler.frameLen);
    
    /* �������� */
 	NRF24L01_CE = 1;
    
    /*�ȴ�������� */
	while ( NRF24L01_IRQ != 0);
    
    /* ��ȡ״̬�Ĵ�����ֵ */
	sta = NRF24L01_Read_Reg(STATUS); 	   
    
    /* ���TX_DS��MAX_RT�жϱ�־ */
	NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS, sta);
    
    /*�ﵽ����ط����� */
	if ( sta & MAX_TX )
	{
        /* ���TX FIFO�Ĵ��� */
		NRF24L01_Write_Reg(FLUSH_TX, 0xff); 
        
		return MAX_TX; 
	}
    
	if ( sta & TX_OK )
	{
        //�������
		return TX_OK;
	}
    
    /* ����ԭ����ʧ�� */
	return 0xff;
}

/**
 *@brief  ����NRF24L01����һ������
 *@param  txbuf:�����������׵�ַ
 *@retval �������״��, 0:�������; �������������
 */
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
    
    //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
    // SPI2_SetSpeed(SPI_BaudRatePrescaler_8); 
    
    /* ��ȡ״̬�Ĵ�����ֵ */
	sta = NRF24L01_Read_Reg(STATUS);
    
    /* ���TX_DS��MAX_RT�жϱ�־ */
	NRF24L01_Write_Reg(WRITE_REG_NRF + STATUS, sta);
    
    /* ���յ����� */
	if ( sta & RX_OK )
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, nrfHandler.frameLen);
        
        /* ���RX FIFO�Ĵ��� */
		NRF24L01_Write_Reg(FLUSH_RX, 0xff);
        
		return 0; 
	}
    
    /* û�յ��κ����� */
	return 1;
}			    

/**
 *@brief  ��ʼ��NRF24L01��RXģʽ
 *   Note: ����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
 *         ��CE��ߺ�,������RXģʽ,�����Խ���������
 *@param  None
 *@retval None
 */
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE = 0; 

    /* дRX�ڵ��ַ */
  	NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); 
    
    /* ʹ��ͨ��0���Զ�Ӧ�� */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA,0x01);
    
    /* ʹ��ͨ��0�Ľ��յ�ַ */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_RXADDR, 0x01);  	 
    
    /* ����RFͨ��Ƶ�� */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 40);
    
    /* ѡ��ͨ��0����Ч���ݿ�� */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + RX_PW_P0,nrfHandler.frameLen);
    
    /* ����TX�������,0db����,2Mbps,���������濪�� */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f);
    
    /* ���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ IRQ enable */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0f);

    /* CEΪ��,�������ģʽ */
  	NRF24L01_CE = 1; 
    
    nrfHandler.state = NRF24L01StateReady;
}

/**
 *@brief  ��ʼ��NRF24L01��TXģʽ
 *   Note: ����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,
 *         ѡ��RFƵ��,�����ʺ�LNA HCURR PWR_UP,CRCʹ��
 *         ��CE��ߺ�,������RXģʽ,�����Խ���������		   
 *         CEΪ�ߴ���10us,����������.	 
 *@param  None
 *@retval None
 */
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE = 0;
    
    /* дTX�ڵ��ַ */
  	NRF24L01_Write_Buf(WRITE_REG_NRF + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);
    
    /* ����TX�ڵ��ַ,��ҪΪ��ʹ��ACK */
  	NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); 	  

    /* ʹ��ͨ��0���Զ�Ӧ�� */
  	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA, 0x01);
    
    /* ʹ��ͨ��0�Ľ��յ�ַ */
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR, 0x01); 
  	
    /* �����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10�� */
    NRF24L01_Write_Reg(WRITE_REG_NRF + SETUP_RETR, 0x1a);
  	
    /* ����RFͨ��Ϊ40 */
    NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 40);
  	
    /* ����TX�������,0db����,2Mbps,���������濪�� */
    NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f);
  	
    /* ���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж� */
    NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0e);    
	
    /* CEΪ��,10us���������� */
    NRF24L01_CE = 1;
    
    nrfHandler.state = NRF24L01StateReady;
}

/**
 *@brief  ���TX������
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
 *@brief  ���RX������
 *@param  None
 *@return None
 */
void NRF24L01_ClearRxBuffer(void)
{
	  u8 i;
    for( i = 0; i < nrfHandler.rxBufferLen; i ++)
        nrfHandler.rxBuffer[i] = 0;
    
    nrfHandler.state = NRF24L01StateReady;
    
    /* Ĭ�Ͻ������ģʽ */
    NRF24L01_RX_Mode();
}

/**
 *@biref  ϵͳ����2.4G ���ݴ���
 *@param  None
 *@return None
 */
void NRF24L01_Send_Frame(void)
{
    /* ���2.4ģ���Ƿ���� */
	if ( nrfHandler.state == NRF24L01StateReady || 
         nrfHandler.state == NRF24L01StateSendTimeout || 
         nrfHandler.state == NRF24L01StateReceived )
	{
        /* ����NRF 2.4Gģ��Ϊ���ڷ���ģʽ */
        nrfHandler.state = NRF24L01StateSending;
        
        NRF24L01_TX_Mode();
		HAL_Delay(1);
		NRF24L01_Write_Reg(FLUSH_TX, 0xff);						//Clear TX FIFO buffer 
        
		NRF24L01_CE=0;
  		NRF24L01_Write_Buf(WR_TX_PLOAD, nrfHandler.txBuffer, nrfHandler.frameLen);//д���ݵ�TX BUF  32���ֽ�
 		NRF24L01_CE=1;//��������
	}
}

/**
 *@brief  NRF24L01 ���������жϷ�����
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
            
            /* RX �ж� */
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
        
        /* ���LINE4�ϵ��жϱ�־λ */
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

/**
 *@brief  NRF 2.4G �ⲿ�ж�4��ʼ��(NRF IRQ - GPIOA.4)
 *@param  None
 *@retval None
 */
static void NRF24L01_EXTI_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	/* GPIOA.4 �ж����Լ��жϳ�ʼ������, �½��ش��� */
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);

  	EXTI_InitStructure.EXTI_Line = NRF24L01_EXTI_Line;  //NRF24L01 IRQ
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

    /* ʹ��NRF24L01 IRQ���ڵ��ⲿ�ж�ͨ�� */
	NVIC_InitStructure.NVIC_IRQChannel = NRF24L01_EXTI_IRQn; 
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_24G_Priority;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
}
