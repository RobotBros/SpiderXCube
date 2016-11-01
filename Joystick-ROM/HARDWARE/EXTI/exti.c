#include "exti.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01  20150813 by:fishcan
//NRF24L01 IRQ intterupt
//外部中断0服务程序
/////////////////////////////////////////////////////////////////////////////////
u8 Joystick_left_x_axis_val,Joystick_left_y_axis_val;		//joystick X/Y value
u8 Joystick_right_x_axis_val,Joystick_right_y_axis_val;
u8 Nrf24l01_rx_handle_flag;
u8 Nrf24l01_rx_buff[NRF42L01_LEN];

void EXTIX_Init(void)
{
 
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);											 //使能复用功能时钟
	

		//GPIOA.8 中断线以及中断初始化配置   下降沿触发
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line8;																		//NRF24L01 IRQ
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);																					  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;														//使能NRF24L01 IRQ所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;								//抢占优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;												//子优先级0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;															//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  																				//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
}

//外部中断8中断处理 接收NRF24L01 DATA RX
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
		EXTI_ClearITPendingBit(EXTI_Line8);  							 						//清除LINE4上的中断标志位
	}
}


 
