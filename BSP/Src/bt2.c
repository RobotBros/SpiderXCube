/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/bt2.c
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    11-Feb-2016
  * @brief   ����2.0 HC-06ģ����������
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



#include "bt2.h"
#include "string.h"

#if BLUETOOTH_4_0 == 0

/* Global Variable ************************************************************/

/* Private Variable ***********************************************************/
static u8 *USARTx_tx_buf;                               /* ���ڷ��ͻ�����     */
static u8 *USARTx_rx_buf;                               /* ���ڽ��ջ�����     */


/* Global Variable ************************************************************/
u32 USARTx_RX_CURRENT_LEN = 0;
u32 USARTx_RX_LEN         = 0;
Boolean BT_DataReady  = FALSE;                          /* �����������ݽ�����ϱ�־ */

/* Private Variable ***********************************************************/

const char AT_cmd_ask[]                  = {"AT"};                              /*<! ͨ�ò�ѯ                          */
const char AT_cmd_answer[]               = {"OK"};

const char AT_cmd_name_set[]             = {"AT+NAMESpiderX"};                  /*<! �������� (Ĭ��ΪSpiderX)          */
const char AT_cmd_name_set_answer[]      = {"OKsetname"};	

const char AT_cmd_pin_set[]              = {"AT+PIN2016"};                      /*<! ������� (Ĭ��2016)               */
const char AT_cmd_pin_answer[]           = {"OKsetPIN"};

const char AT_cmd_baud_set[]             = {"AT+BAUD8"};                        /*<! ���������� (1 - 8)
                                                                                     1 - 1200
                                                                                     2 - 2400
                                                                                     3 - 4800
                                                                                     4 - 9600
                                                                                     5 - 19200
                                                                                     6 - 38400
                                                                                     7 - 57600
                                                                                     8 - 115200
                                                                                     9 - 230400
                                                                                     A - 460800
                                                                                     B - 921600
                                                                                     C - 1382400
                                                                                     */
const char AT_cmd_baud_answer[]          = {"OK115200"};
/* Private functions **********************************************************/

static void BT_ClearBuffer(u8 *buffer, u8 data_len);
static u8 BT_SetCmdCheck(const char *p, u8 *data, u8 data_len, Boolean clear_buf);
static u8 BT_SetCmd(const char *atstr, const char *atstr_ask, u8 retry);

/* Private functions prototypes ***********************************************/

/* Public functions ***********************************************************/

/**
 *@brief  ���ݴ�����ɺ�BUFF���㣬��־������֡��������
 *@param  None
 *@return None
 */
void USARTx_resetBuffer(void)
{    
    USART_ITConfig(BT_USARTx, USART_IT_RXNE, DISABLE);
    
    USARTx_RX_LEN = 0;
    USARTx_RX_CURRENT_LEN = 0;
    BT_DataReady = FALSE;
    
    USART_ITConfig(BT_USARTx, USART_IT_RXNE, ENABLE);	 				
}

/**
 *@brief  ��ʼ��IO ����3 
 *@param  bound  ������
 *@return None
 */
void USARTx_Init(u32 bound)
{    
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	__BT_PORT_RCC_ENABLE__();	//ʹ��USART3��GPIOCʱ�� PC10--TX PC11--RX
	__BT_USART_RCC_ENABLE__();
    
 	USART_DeInit(BT_USARTx);                                                    //��λ����3
	__BT_PIN_REMAP__();                                  						//������ӳ��
	
    //USART3 I/O TX
	GPIO_InitStructure.GPIO_Pin = BT_TX_PIN; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;								//�����������
  	GPIO_Init(BT_USARTx_PORT, &GPIO_InitStructure); 

    //USART3_I/O RX	  
 	GPIO_InitStructure.GPIO_Pin = BT_RX_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                       //��������
  	GPIO_Init(BT_USARTx_PORT, &GPIO_InitStructure);  

    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;									
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;					//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                      //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                         //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				//�շ�ģʽ
  	USART_Init(BT_USARTx, &USART_InitStructure);                                   //��ʼ������
																				
    //USARTx NVIC ����
  	NVIC_InitStructure.NVIC_IRQChannel = BT_USARTx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_BT_USARTx_Priority; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);                                             //����ָ���Ĳ�����ʼ��VIC�Ĵ���   
  	USART_ITConfig(BT_USARTx, USART_IT_RXNE, ENABLE);                           //�����ж�  ����
    
    USART_Cmd(BT_USARTx, ENABLE);                                                  //ʹ�ܴ��� 	
}

/*
 *@brief  ����3 RX DMA ģʽ�����ر� 
 *@param  len  DMA rx �������
 *@return Nonoe
 */
void USARTx_Init_DMA(u16 bufferLen)
{
    
    DMA_InitTypeDef DMA_InitStructure; 
    NVIC_InitTypeDef NVIC_InitStructure;
    
    BT_DataReady = FALSE;
    
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);							//�ر�USART3�ж�
    USART_Cmd(USART3, DISABLE);
	
    //************************USART RX DMA1 CH3**************************//
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);					    //ʹ��DMA CLK	
    DMA_DeInit(BT_USARTx_DMA_RX_CH);  									    //RX DMA1 CH3	
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(BT_USARTx->DR)); 		//DMA����USART3 DATA����ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USARTx_rx_buf;              //DMA�ڴ����ַ RX_buf
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  					//���ݴ��䷽�򣬴����跢�͵��ڴ��ȡ
    DMA_InitStructure.DMA_BufferSize = bufferLen;                           //DMAͨ����DMA����Ĵ�С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  		//�����ַ�Ĵ�������
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  				//�ڴ��ַ�Ĵ�������
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //���ݿ���Ϊ8λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //���ݿ���Ϊ8λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //��������������ģʽ  ����
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     //DMAͨ�� xӵ�и����ȼ� 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
    DMA_Init(BT_USARTx_DMA_RX_CH, &DMA_InitStructure);  
    DMA_Cmd(BT_USARTx_DMA_RX_CH, ENABLE);
    
    //***********************DMA1 RX CH3�ж�����**************************//
    NVIC_InitStructure.NVIC_IRQChannel = BT_DMA_Channel_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_BT_USARTx_Priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                      //�����ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         //IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);                                         //����ָ���Ĳ�����ʼ��VIC�Ĵ���   
    DMA_ITConfig(BT_USARTx_DMA_RX_CH, DMA_IT_TC, ENABLE);                     //��������ж�
    DMA_ClearFlag(BT_USARTx_DMA_RX_TC);
    DMA_Cmd(BT_USARTx_DMA_RX_CH, ENABLE); 
    USART_DMACmd(BT_USARTx, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(BT_USARTx, ENABLE);                                              //ʹ�ܴ��� 
}

/**
 *@biref ����printf ����
 *       ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
 *@param  fmt  ���ݴ�ӡ��ʽ
 *@param  ...  ����
 *@return None
 */
void USARTx_printf(char* fmt,...)  
{  
    u16 i;
    
    va_list ap;
    va_start(ap,fmt);
    vsprintf((char*)USARTx_tx_buf, fmt, ap);
    va_end(ap);
    
    /* �������ݵĳ��� */
    i = strlen((const char*)USARTx_tx_buf);
    
    USARTx_print(USARTx_tx_buf, i);
}

/**
 *  @brief  ��ʼ��BT_4.0ģ��
 *  @param  retry: �ظ����ô���
 *  @param  rxBufferPtr: ���ջ�����ָ��
 *  @param  txBufferPtr: ���ͻ�����ָ��
 *  @retval ����ֵ: TRUE:�ɹ�; FALSE:ʧ��.
 */
u8 BT_Init(u8 retry, u8 *rxBufferPtr, u8 *txBufferPtr)
{
	u8 bandsel_cnt, flag;
	u32 bandsel[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

	flag = FALSE;
    USARTx_rx_buf = rxBufferPtr;
    USARTx_tx_buf = txBufferPtr;
    
    /* �ȴ��������� */
    HAL_Delay(100);
	
    /* Ĭ�ϼ�Ⲩ���� */
	USARTx_Init(BT_BAUND);
    
    /* ��ʼ���жϲ�����Ϊ115200��Ĭ��Ϊģ������������� */
	if(BT_SetCmd(AT_cmd_ask, AT_cmd_answer, retry) == TRUE)			
		flag = TRUE;
	
    /* ѭ��ȷ��ģ�鲨���� */	
	if(!flag)
	{
        /* ���ģ�鲨�����Ƿ�������ȷ */
		for ( bandsel_cnt = 0; bandsel_cnt < 8; bandsel_cnt ++ )
		{	
			USARTx_Init(bandsel[bandsel_cnt]);
			HAL_Delay(100);
            
            /* ����AT����ָ��, �ж��Ƿ����óɹ� */
			if ( BT_SetCmd(AT_cmd_ask, AT_cmd_answer, retry) == TRUE )
				break;
		}
		
        /* �޷���⵽��ֱ������ΪĬ�ϲ����� */
		if ( BT_SetCmd(AT_cmd_baud_set, AT_cmd_baud_answer, retry) == FALSE )
			return flag;
        
		HAL_Delay(100);
        
        /* �������ô��ڲ����� */
		USARTx_Init(BT_BAUND);
		HAL_Delay(100);
	}
    
	/* ����Bluetoothģ�� */
    
    /* 1. ������������ */
	if(BT_SetCmd(AT_cmd_name_set, AT_cmd_name_set_answer, retry) == FALSE)	
        return flag = FALSE;
    
	/* 2. �������� */
	if(BT_SetCmd(AT_cmd_pin_set, AT_cmd_pin_answer, retry) == FALSE)	
        return flag = FALSE;

	return flag = TRUE;
}


/**
 *@brief  ����֡������
 *@param  frame: ��Ҫ���͵�֡
 *@retval None
 */
void BT_SendFrame(u8 *buffer, u16 bufferLen)
{
   USARTx_print(buffer, bufferLen);
}

/* Private functions **********************************************************/


/**
 *@brief             BTģ����������
 *                   �˺���������������,�����ڽ�����OKӦ���ATָ��  
 *@param  atstr      ATָ�.����:"AT+RESET"/"AT+UART=9600,0,0"/"AT+ROLE=0"���ַ���
 *@param  atstr_ask  ATָ���Ӧ��������
 *@param  retry      �ظ����õĴ���
 *@return            TRUE,���óɹ�; FALSE,����ʧ��.	
 */
static u8 BT_SetCmd(const char *atstr, const char *atstr_ask, u8 retry)
{			
  u8 i;	
	Boolean flag = FALSE;

	while(retry--)
	{		
		HAL_Delay(10);
        
        USARTx_RX_LEN = strlen(atstr_ask);
        
        /* ����AT�ַ��� */
		USARTx_printf("%s",atstr);
        
        /* ��ȴ�100ms,������BTģ��Ļ�Ӧ */
        for ( i = 0; i < 10 ; i ++ )
        {
            HAL_Delay(100);
        }
        
        /* �жϹ���Ӧ��������RX����һ�� */
        if( BT_SetCmdCheck(atstr_ask, USARTx_rx_buf, USARTx_RX_CURRENT_LEN, TRUE) == TRUE )
        {
            flag = TRUE;
            
            break;			 
        } 
        else 
        {
            USARTx_resetBuffer();
        }
	}
    
    USARTx_resetBuffer();
	return flag;
}

/**
 *@brief  �ж������ַ���p�Ƿ���data��������һ��
 *@param  *p         �ַ�����
 *@param  *data      ԭ�Ա�����
 *@param   data_len  �Ա�ԭ���ݳ���
 *@return  һ�·���TRUE����ͬ����FALSE
 */
static u8 BT_SetCmdCheck(const char *p,u8 *data,u8 data_len, Boolean clear_buf)
{
	u8 cnt,p_len,flag;

	flag = FALSE;

	p_len = strlen(p);
    
    //�Ա������ݳ���
	if(p_len != data_len)					
	{
		// ��ջ�����
		if(clear_buf)
			BT_ClearBuffer(data, data_len);
		return flag;
	}

	for(cnt=0;cnt<p_len;cnt++)
	{
		if(data[cnt] != *(p+cnt))
			break;
	}

	if(cnt != p_len)
		flag = FALSE;
	else
		flag = TRUE;

	// ��ջ�����
	if(clear_buf)
		BT_ClearBuffer(data, data_len);
	return flag;
}

/**
 *@biref  ��ջ�����
 *@param  buffer   ������ָ��
 *@param  data_len ����������
 *@return None
 */
static void BT_ClearBuffer(u8 *buffer, u8 data_len)
{
	u8 i;
	for(i = 0; i < data_len; i ++)
	{
		buffer[i] = 0x00;
	}
}

/**
 *@brief  �������ݵ�����
 *@param  buffer    ���ݻ�����ָ��
 *@param  bufferLen ���ݳ���
 *@return None
 */
void USARTx_print(u8 *buffer, u16 bufferLen)
{
    u16 i;
    for( i = 0; i < bufferLen; i ++ )
    {
        /*�ȴ��ϴδ������ */
        while ( USART_GetFlagStatus(BT_USARTx, USART_FLAG_TC) == RESET );  
        
        /*�������ݵ�����3 */
        USART_SendData(BT_USARTx, buffer[i]);
    }
}

/*
 *@brief  DMA RX�жϴ�������
 *@param  None
 *@return None
 */
void BT_DMAx_Channelx_IRQHandler(void)
{
	if(DMA_GetFlagStatus(BT_USARTx_DMA_RX_TC) != RESET)
	{
        /* ������������ */
        BT_DataReady = TRUE;
	}
	DMA_ClearITPendingBit(BT_USARTx_DMA_RX_TC);
}

/**
 *@brief  �����жϷ������
 *@param  None
 *@return None
 */
void BT_USARTx_IRQHandler(void)
{
	u8 rx_buf_temp;	
    
    /* ���յ����� */
	if(USART_GetITStatus(BT_USARTx, USART_IT_RXNE) != RESET)		
	{	
        USARTx_RX_CURRENT_LEN ++;
        
        /* ���յ�����������֡��֪ͨ�ϲ�Ӧ�� */
        if ( USARTx_RX_CURRENT_LEN == USARTx_RX_LEN )
            BT_DataReady = TRUE;
        
        rx_buf_temp = USART_ReceiveData(BT_USARTx);
        if (USARTx_RX_CURRENT_LEN <= USARTx_RX_LEN) {
            USARTx_rx_buf[USARTx_RX_CURRENT_LEN - 1] = rx_buf_temp;
        } else {
            /* �쳣Խ�磬���������� */
            USARTx_RX_CURRENT_LEN  = 0;
        }
 	}
}

#endif