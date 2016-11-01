#include <stdio.h>  
#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif

#if defined ( __ICCARM__ )  //IAR��ʹ��  
//�ض���fputc����   
int fputc(int ch, FILE *f)  
{        
    while((USART1->SR&0X40)==0);//ѭ������,ֱ���������     
    USART1->DR = (u8) ch;        
    return ch;  
}  
#else  //��IAR��ʹ��  
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);
    USART_SendData(USART3,(uint8_t)ch);   
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART3, (uint8_t) ch);

	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART3->SR & USART_FLAG_RXNE));

    return ((int)(USART3->DR & 0x1FF));
}
*/
 
#if EN_USART3_RX   //���ʹ���˽���
//����3�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  

//��ʼ��IO ����3 
//bound:������
void uart_init(u32 bound){
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);	//ʹ��USART3��GPIOCʱ�� PC10--TX PC11--RX
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 	USART_DeInit(USART3); 																											//��λ����3
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); 											//������ӳ��
	
	//USART3 I/O TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;															//�����������
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
   
  //USART3_I/O RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;												//��������
  GPIO_Init(GPIOC, &GPIO_InitStructure);  

  //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;																			//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;											//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;													//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;															//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;									//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); 																			//��ʼ������
	
	#if EN_USART3_RX																																//���ʹ���˽���  
  //Usart3 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;											  	//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;															//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;																	//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);																									//����ָ���Ĳ�����ʼ��VIC�Ĵ���   
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);																	//�����ж�
	#endif
	
  USART_Cmd(USART3, ENABLE);                    																	//ʹ�ܴ��� 
}

void USART3_IRQHandler(void)                																		  //����3�жϷ������
{
	u8 Res;
	#ifdef OS_TICKS_PER_SEC	 																												//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
		OSIntEnter();    
	#endif
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 												 //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART3);																							 //(USART3->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)																								 //����δ���
		{
			if(USART_RX_STA&0x4000)																										 //���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;																						 //���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;																							 //��������� 
			}
			else 																																			 //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;											 //�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
   } 
	#ifdef OS_TICKS_PER_SEC	 																												//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
		OSIntExit();  											 
	#endif
}

#endif	

