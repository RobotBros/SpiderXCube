/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/bt2.c
  * @author  Enix Yu, RobotBros
  * @version V0.1.0
  * @date    11-Feb-2016
  * @brief   蓝牙2.0 HC-06模块驱动函数
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
static u8 *USARTx_tx_buf;                               /* 串口发送缓存区     */
static u8 *USARTx_rx_buf;                               /* 串口接收缓存区     */


/* Global Variable ************************************************************/
u32 USARTx_RX_CURRENT_LEN = 0;
u32 USARTx_RX_LEN         = 0;
Boolean BT_DataReady  = FALSE;                          /* 蓝牙串口数据接收完毕标志 */

/* Private Variable ***********************************************************/

const char AT_cmd_ask[]                  = {"AT"};                              /*<! 通用查询                          */
const char AT_cmd_answer[]               = {"OK"};

const char AT_cmd_name_set[]             = {"AT+NAMESpiderX"};                  /*<! 蓝牙名称 (默认为SpiderX)          */
const char AT_cmd_name_set_answer[]      = {"OKsetname"};	

const char AT_cmd_pin_set[]              = {"AT+PIN2016"};                      /*<! 配对密码 (默认2016)               */
const char AT_cmd_pin_answer[]           = {"OKsetPIN"};

const char AT_cmd_baud_set[]             = {"AT+BAUD8"};                        /*<! 波特率设置 (1 - 8)
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
 *@brief  数据处理完成后BUFF清零，标志清理，帧计数清零
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
 *@brief  初始化IO 串口3 
 *@param  bound  波特率
 *@return None
 */
void USARTx_Init(u32 bound)
{    
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	__BT_PORT_RCC_ENABLE__();	//使能USART3，GPIOC时钟 PC10--TX PC11--RX
	__BT_USART_RCC_ENABLE__();
    
 	USART_DeInit(BT_USARTx);                                                    //复位串口3
	__BT_PIN_REMAP__();                                  						//部分重映射
	
    //USART3 I/O TX
	GPIO_InitStructure.GPIO_Pin = BT_TX_PIN; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;								//复用推挽输出
  	GPIO_Init(BT_USARTx_PORT, &GPIO_InitStructure); 

    //USART3_I/O RX	  
 	GPIO_InitStructure.GPIO_Pin = BT_RX_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                       //浮空输入
  	GPIO_Init(BT_USARTx_PORT, &GPIO_InitStructure);  

    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;									
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;					//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                      //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                         //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				//收发模式
  	USART_Init(BT_USARTx, &USART_InitStructure);                                   //初始化串口
																				
    //USARTx NVIC 配置
  	NVIC_InitStructure.NVIC_IRQChannel = BT_USARTx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_BT_USARTx_Priority; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //次优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);                                             //根据指定的参数初始化VIC寄存器   
  	USART_ITConfig(BT_USARTx, USART_IT_RXNE, ENABLE);                           //开启中断  空闲
    
    USART_Cmd(BT_USARTx, ENABLE);                                                  //使能串口 	
}

/*
 *@brief  串口3 RX DMA 模式开启关闭 
 *@param  len  DMA rx 缓存深度
 *@return Nonoe
 */
void USARTx_Init_DMA(u16 bufferLen)
{
    
    DMA_InitTypeDef DMA_InitStructure; 
    NVIC_InitTypeDef NVIC_InitStructure;
    
    BT_DataReady = FALSE;
    
    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);							//关闭USART3中断
    USART_Cmd(USART3, DISABLE);
	
    //************************USART RX DMA1 CH3**************************//
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);					    //使能DMA CLK	
    DMA_DeInit(BT_USARTx_DMA_RX_CH);  									    //RX DMA1 CH3	
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&(BT_USARTx->DR)); 		//DMA外设USART3 DATA基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USARTx_rx_buf;              //DMA内存基地址 RX_buf
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  					//数据传输方向，从外设发送到内存读取
    DMA_InitStructure.DMA_BufferSize = bufferLen;                           //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  		//外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  				//内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         //数据宽度为8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //工作在正常缓存模式  单次
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     //DMA通道 x拥有高优先级 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            //DMA通道x没有设置为内存到内存传输
    DMA_Init(BT_USARTx_DMA_RX_CH, &DMA_InitStructure);  
    DMA_Cmd(BT_USARTx_DMA_RX_CH, ENABLE);
    
    //***********************DMA1 RX CH3中断配置**************************//
    NVIC_InitStructure.NVIC_IRQChannel = BT_DMA_Channel_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = IRQ_BT_USARTx_Priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                      //次优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                         //IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                                         //根据指定的参数初始化VIC寄存器   
    DMA_ITConfig(BT_USARTx_DMA_RX_CH, DMA_IT_TC, ENABLE);                     //传输完成中断
    DMA_ClearFlag(BT_USARTx_DMA_RX_TC);
    DMA_Cmd(BT_USARTx_DMA_RX_CH, ENABLE); 
    USART_DMACmd(BT_USARTx, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(BT_USARTx, ENABLE);                                              //使能串口 
}

/**
 *@biref 串口printf 函数
 *       确保一次发送数据不超过USART3_MAX_SEND_LEN字节
 *@param  fmt  数据打印格式
 *@param  ...  数据
 *@return None
 */
void USARTx_printf(char* fmt,...)  
{  
    u16 i;
    
    va_list ap;
    va_start(ap,fmt);
    vsprintf((char*)USARTx_tx_buf, fmt, ap);
    va_end(ap);
    
    /* 发送数据的长度 */
    i = strlen((const char*)USARTx_tx_buf);
    
    USARTx_print(USARTx_tx_buf, i);
}

/**
 *  @brief  初始化BT_4.0模块
 *  @param  retry: 重复设置次数
 *  @param  rxBufferPtr: 接收缓冲区指针
 *  @param  txBufferPtr: 发送缓冲区指针
 *  @retval 返回值: TRUE:成功; FALSE:失败.
 */
u8 BT_Init(u8 retry, u8 *rxBufferPtr, u8 *txBufferPtr)
{
	u8 bandsel_cnt, flag;
	u32 bandsel[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

	flag = FALSE;
    USARTx_rx_buf = rxBufferPtr;
    USARTx_tx_buf = txBufferPtr;
    
    /* 等待蓝牙启动 */
    HAL_Delay(100);
	
    /* 默认检测波特率 */
	USARTx_Init(BT_BAUND);
    
    /* 初始化判断波特率为115200，默认为模块曾经配置完毕 */
	if(BT_SetCmd(AT_cmd_ask, AT_cmd_answer, retry) == TRUE)			
		flag = TRUE;
	
    /* 循环确定模块波特率 */	
	if(!flag)
	{
        /* 检测模块波特率是否设置正确 */
		for ( bandsel_cnt = 0; bandsel_cnt < 8; bandsel_cnt ++ )
		{	
			USARTx_Init(bandsel[bandsel_cnt]);
			HAL_Delay(100);
            
            /* 发送AT测试指令, 判断是否配置成功 */
			if ( BT_SetCmd(AT_cmd_ask, AT_cmd_answer, retry) == TRUE )
				break;
		}
		
        /* 无法检测到，直接设置为默认波特率 */
		if ( BT_SetCmd(AT_cmd_baud_set, AT_cmd_baud_answer, retry) == FALSE )
			return flag;
        
		HAL_Delay(100);
        
        /* 重新设置串口波特率 */
		USARTx_Init(BT_BAUND);
		HAL_Delay(100);
	}
    
	/* 配置Bluetooth模块 */
    
    /* 1. 设置蓝牙名字 */
	if(BT_SetCmd(AT_cmd_name_set, AT_cmd_name_set_answer, retry) == FALSE)	
        return flag = FALSE;
    
	/* 2. 设置密码 */
	if(BT_SetCmd(AT_cmd_pin_set, AT_cmd_pin_answer, retry) == FALSE)	
        return flag = FALSE;

	return flag = TRUE;
}


/**
 *@brief  发送帧至蓝牙
 *@param  frame: 需要发送的帧
 *@retval None
 */
void BT_SendFrame(u8 *buffer, u16 bufferLen)
{
   USARTx_print(buffer, bufferLen);
}

/* Private functions **********************************************************/


/**
 *@brief             BT模块设置命令
 *                   此函数用于设置蓝牙,适用于仅返回OK应答的AT指令  
 *@param  atstr      AT指令串.比如:"AT+RESET"/"AT+UART=9600,0,0"/"AT+ROLE=0"等字符串
 *@param  atstr_ask  AT指令对应返回数据
 *@param  retry      重复设置的次数
 *@return            TRUE,设置成功; FALSE,设置失败.	
 */
static u8 BT_SetCmd(const char *atstr, const char *atstr_ask, u8 retry)
{			
  u8 i;	
	Boolean flag = FALSE;

	while(retry--)
	{		
		HAL_Delay(10);
        
        USARTx_RX_LEN = strlen(atstr_ask);
        
        /* 发送AT字符串 */
		USARTx_printf("%s",atstr);
        
        /* 最长等待100ms,来接收BT模块的回应 */
        for ( i = 0; i < 10 ; i ++ )
        {
            HAL_Delay(100);
        }
        
        /* 判断功能应答数据与RX数据一致 */
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
 *@brief  判断输入字符串p是否与data接收数据一致
 *@param  *p         字符串；
 *@param  *data      原对比数据
 *@param   data_len  对比原数据长度
 *@return  一致返回TRUE，不同返回FALSE
 */
static u8 BT_SetCmdCheck(const char *p,u8 *data,u8 data_len, Boolean clear_buf)
{
	u8 cnt,p_len,flag;

	flag = FALSE;

	p_len = strlen(p);
    
    //对比两数据长度
	if(p_len != data_len)					
	{
		// 清空缓冲区
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

	// 清空缓冲区
	if(clear_buf)
		BT_ClearBuffer(data, data_len);
	return flag;
}

/**
 *@biref  清空缓冲区
 *@param  buffer   缓冲区指针
 *@param  data_len 缓冲区长度
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
 *@brief  发送数据到串口
 *@param  buffer    数据缓冲区指针
 *@param  bufferLen 数据长度
 *@return None
 */
void USARTx_print(u8 *buffer, u16 bufferLen)
{
    u16 i;
    for( i = 0; i < bufferLen; i ++ )
    {
        /*等待上次传输完成 */
        while ( USART_GetFlagStatus(BT_USARTx, USART_FLAG_TC) == RESET );  
        
        /*发送数据到串口3 */
        USART_SendData(BT_USARTx, buffer[i]);
    }
}

/*
 *@brief  DMA RX中断处理程序
 *@param  None
 *@return None
 */
void BT_DMAx_Channelx_IRQHandler(void)
{
	if(DMA_GetFlagStatus(BT_USARTx_DMA_RX_TC) != RESET)
	{
        /* 处理接收数据 */
        BT_DataReady = TRUE;
	}
	DMA_ClearITPendingBit(BT_USARTx_DMA_RX_TC);
}

/**
 *@brief  串口中断服务程序
 *@param  None
 *@return None
 */
void BT_USARTx_IRQHandler(void)
{
	u8 rx_buf_temp;	
    
    /* 接收到数据 */
	if(USART_GetITStatus(BT_USARTx, USART_IT_RXNE) != RESET)		
	{	
        USARTx_RX_CURRENT_LEN ++;
        
        /* 接收到完整的数据帧，通知上层应用 */
        if ( USARTx_RX_CURRENT_LEN == USARTx_RX_LEN )
            BT_DataReady = TRUE;
        
        rx_buf_temp = USART_ReceiveData(BT_USARTx);
        if (USARTx_RX_CURRENT_LEN <= USARTx_RX_LEN) {
            USARTx_rx_buf[USARTx_RX_CURRENT_LEN - 1] = rx_buf_temp;
        } else {
            /* 异常越界，计数器回零 */
            USARTx_RX_CURRENT_LEN  = 0;
        }
 	}
}

#endif