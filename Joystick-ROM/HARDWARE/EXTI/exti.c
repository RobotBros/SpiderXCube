#include "exti.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01  20150813 by:fishcan
//NRF24L01 IRQ intterupt
//�ⲿ�ж�0�������
/////////////////////////////////////////////////////////////////////////////////
u8 Joystick_left_x_axis_val,Joystick_left_y_axis_val;		//joystick X/Y value
u8 Joystick_right_x_axis_val,Joystick_right_y_axis_val;
u8 Nrf24l01_rx_handle_flag;
u8 Nrf24l01_rx_buff[NRF42L01_LEN];

void EXTIX_Init(void)
{
 
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);											 //ʹ�ܸ��ù���ʱ��
	

		//GPIOA.8 �ж����Լ��жϳ�ʼ������   �½��ش���
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;																		//NRF24L01 IRQ
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);																					  	//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;														//ʹ��NRF24L01 IRQ���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;								//��ռ���ȼ�1 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;												//�����ȼ�0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;															//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  																				//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
}

//�ⲿ�ж�8�жϴ��� ����NRF24L01 DATA RX
void EXTI9_5_IRQHandler(void)
{	
	u8 status;

	if(EXTI_GetFlagStatus(EXTI_Line8) != RESET)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_PinSource8) == 0)
		{
			status = NRF24L01_Read_Reg(READ_REG_NRF+STATUS);
			if(status & RX_OK)																	//RX data occured
			{
				NRF24L01_Read_Buf(RD_RX_PLOAD,Nrf24l01_rx_buff,RX_PLOAD_WIDTH);			//RX FIFO data buffer to NRF buffer
				NRF24L01_Write_Reg(FLUSH_RX,0xff);											//Clear RX FIFO data buffer
				//Nrf24l01_Receive_Data_Handle(Nrf24l01_rx_buff,NRF42L01_LEN);				//NRF RX data to USART TX buffer,whaiting for send action
				Nrf24l01_rx_handle_flag = DONE;												//NRF RX data handle is done flag
				//printf("NRF24l01 RX is done! \r\n");												//Debug RX is done
				//GUI_ShowString(10,70,12,"RX DONE",0);
				TIM_Cmd(TIM2, DISABLE);  												// RX OK clear timeout counter
				NRF24L01_TX_Mode();
			}
			
			else if((status & MAX_TX) > 0)							//TX data times was the max  occured
			{
				NRF24L01_Write_Reg(FLUSH_TX,0xff);				//Clear TX FIFO buffer 
				//printf("NRF24l01 TX is out of times! \r\n");			//Debug TX is out of times
				//GUI_ShowString(10,70,12,"TX out",0);
				NRF24L01_TX_Mode();							//Change ro RX mode							
			}
			
			else if((status & TX_OK) > 0)							//TX data is done(ACK feedback) occured
			{
				NRF24L01_Write_Reg(FLUSH_TX,0xff);				//Clear TX FIFO buffer 
				//printf("NRF24l01 TX is done! \r\n");					//Debug TX is done
				//GUI_ShowString(10,70,12,"TX DONE",0);
				TIM_Cmd(TIM2, ENABLE);  							// start RX timeout counter 1s
				NRF24L01_RX_Mode();														//Change ro RX mode							
			}
			
			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,status);			  					//Clear NRF IRQ flag			
		}
		EXTI_ClearITPendingBit(EXTI_Line8);  							 						//���LINE4�ϵ��жϱ�־λ
	}
}


 
