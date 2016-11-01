#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01 
//接收上位机数据  Data 20150820 By fishcan
//数据结构			 头码+命令字（读/写）+DATA（8bit*19）+checksum  AA接收指令 AB发送指令
//接收数据结构	FA+FB+AA+19路舵机角度数据+数据SUM，最低四位取反
////////////////////////////////////////////////////////////////////////////////// 

u8 Usart_rx_cammand_flag;	
u8 Usart_rx_flame_count;
u8 Usart_rx_done_flag;
u8 Usart_rx_ctrl_flag;

//校验和计算
//成功返回：1
//失败返回：0
u8 Check_sum(u8 *CheckAdrr, u8 CheckCnt)
{
	u8 i=0;
	u8 *loadAddr;
	u8 sum = 0;

	loadAddr = CheckAdrr;
	for(i=0; i<(CheckCnt - 1); i++)
	{
		sum += *loadAddr;
		loadAddr ++;
	}
	sum = sum ^ 0xFF;
	
	if (sum == *loadAddr) 
	{
		return 1;
	}
	else	
	{
		return 0;
	}	
}


//PC命令处理
void comm_pc_cammand(void)
{	
		u8 cnt;																								//debug

		if(Usart_rx_ctrl_flag)
			{Usart_rx_cammand_flag = USART_CAMMAND_CTRL;}
		else
			{Usart_rx_cammand_flag = USART_CAMMAND_CHECK;}					//未接受到数据，默认为查询模式
	
		if(Usart_rx_done_flag)
		{
				if (Check_sum(Usart_rx_buf,Usart_rx_flame_count) == 0)
				{
					comm_pc_cammand_reset();
					return;
				}				
					
//////////////////////////////////debug start///////////////////////////////////////////////////////	
//		for(cnt=0;cnt<USART_REC_LEN-1;cnt++)
//		{
//			printf("%x ",Usart_rx_buf[cnt]);										//debug print usart rx data 22 bytes
//		}
//		printf("%x \r\n",Usart_rx_buf[cnt]);
//////////////////////////////////debug end ///////////////////////////////////////////////////////		
				
				if(Usart_rx_buf[2] == 0xAA)														//控制命令字
				{
					Usart_rx_cammand_flag = USART_CAMMAND_CTRL;
//					LED0 =!LED0;						//debug
					Usart_rx_ctrl_flag = 1;											
				}
				else if(Usart_rx_buf[2] == 0xAB)												  //查询命令字
				{
					Usart_rx_cammand_flag = USART_CAMMAND_CHECK;
//					Usart_rx_success_flag = 1;
				}
		}

}	

//数据处理完成后BUFF清零，标志清理，帧计数清零
void comm_pc_cammand_reset(void)
{
		u8 counter;

		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
		Usart_rx_flame_count = 0;
//		Usart_rx_success_flag = 0;
	  	Usart_rx_done_flag = 0;
		for(counter=0;counter<USART_REC_LEN-1;counter++)
		{
				Usart_rx_buf[counter] = 0;
		}
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	 				//指令执行一周后开中断，重新接收数据处理
}

//Send slave data to PC
void comm_to_pc(void)
{
	u8 cnt;
	
	for(cnt=0;cnt<USART_REC_LEN;cnt++)
	{
		USART_SendData(USART3,Usart_tx_buf[cnt]); 						//16进制形式发送
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);			
	}	
}

