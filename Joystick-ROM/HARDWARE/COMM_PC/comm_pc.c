#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01 
//������λ������  Data 20150820 By fishcan
//���ݽṹ			 ͷ��+�����֣���/д��+DATA��8bit*19��+checksum  AA����ָ�� AB����ָ��
//�������ݽṹ	FA+FB+AA+19·����Ƕ�����+����SUM�������λȡ��
////////////////////////////////////////////////////////////////////////////////// 

u8 Usart_rx_cammand_flag;	
u8 Usart_rx_flame_count;
u8 Usart_rx_done_flag;
u8 Usart_rx_ctrl_flag;

//У��ͼ���
//�ɹ����أ�1
//ʧ�ܷ��أ�0
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


//PC�����
void comm_pc_cammand(void)
{	
		u8 cnt;																								//debug

		if(Usart_rx_ctrl_flag)
			{Usart_rx_cammand_flag = USART_CAMMAND_CTRL;}
		else
			{Usart_rx_cammand_flag = USART_CAMMAND_CHECK;}					//δ���ܵ����ݣ�Ĭ��Ϊ��ѯģʽ
	
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
				
				if(Usart_rx_buf[2] == 0xAA)														//����������
				{
					Usart_rx_cammand_flag = USART_CAMMAND_CTRL;
//					LED0 =!LED0;						//debug
					Usart_rx_ctrl_flag = 1;											
				}
				else if(Usart_rx_buf[2] == 0xAB)												  //��ѯ������
				{
					Usart_rx_cammand_flag = USART_CAMMAND_CHECK;
//					Usart_rx_success_flag = 1;
				}
		}

}	

//���ݴ�����ɺ�BUFF���㣬��־����֡��������
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
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	 				//ָ��ִ��һ�ܺ��жϣ����½������ݴ���
}

//Send slave data to PC
void comm_to_pc(void)
{
	u8 cnt;
	
	for(cnt=0;cnt<USART_REC_LEN;cnt++)
	{
		USART_SendData(USART3,Usart_tx_buf[cnt]); 						//16������ʽ����
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);			
	}	
}

