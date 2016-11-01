#include "exti.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01  20150813 by:fishcan
//NRF24L01 IRQ intterupt
//
/////////////////////////////////////////////////////////////////////////////////
u8 Ctrl_state_flag;
u8 Nrf24l01_rx_buff[NRF24L01_LEN]; 			 //
u8 Steering_engine_L_B_E_angle;
u8 Steering_engine_L_B_F_angle;
u8 Steering_engine_L_B_M_angle;
u8 Steering_engine_L_M_E_angle;
u8 Steering_engine_L_M_M_angle;
u8 Steering_engine_L_M_F_angle;
u8 Steering_engine_L_F_E_angle;
u8 Steering_engine_L_F_M_angle;
u8 Steering_engine_L_F_F_angle;
u8 Steering_engine_R_B_E_angle;
u8 Steering_engine_R_B_M_angle;
u8 Steering_engine_R_B_F_angle;
u8 Steering_engine_R_M_E_angle;
u8 Steering_engine_R_M_M_angle;
u8 Steering_engine_R_M_F_angle;
u8 Steering_engine_R_F_E_angle;
u8 Steering_engine_R_F_M_angle;
u8 Steering_engine_R_F_F_angle;

// 4·��չ��
u8 Steering_engine_DRV1_angle;
u8 Steering_engine_DRV2_angle;
u8 Steering_engine_DRV3_angle;
u8 Steering_engine_DRV4_angle;

u8 Joystick_left_x_axis_val,Joystick_left_y_axis_val;		//joystick X/Y value
u8 Joystick_right_x_axis_val,Joystick_right_y_axis_val;

u8 System_HW_cur_protect_flag;

//�ⲿ�ж�4��ʼ��,NRF IRQ 
//GPIOA.4
void EXTIX_Init_Nrfirq(void)
{
    
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);								//ʹ�ܸ��ù���ʱ��
	
	//GPIOA.4 �ж����Լ��жϳ�ʼ������   �½��ش���
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource4);
    
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;													//NRF24L01 IRQ
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);															//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
    
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;										//ʹ��NRF24L01 IRQ���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;								//��ռ���ȼ�2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;										//�����ȼ�0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;										//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  													//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
    
}


//�ⲿ�ж�4��ʼ��,Peak current protection  
//GPIOB.5
void EXTIX_Init_curkill(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
    //*********************GPIO init******************************//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 							//ʹ��PB�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 											//PB5-IRQ ����  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	GPIO_SetBits(GPIOB,GPIO_Pin_5); 														//current kill IRQ����
    //*********************Extix init******************************//
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);								//ʹ�ܸ��ù���ʱ��
	
	//GPIOB.5 �ж����Լ��жϳ�ʼ������   �½��ش���
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource5);
    
  	EXTI_InitStructure.EXTI_Line=EXTI_Line5;													
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);															//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
    
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;										//ʹ�� IRQ���ڵ��ⲿ�ж�ͨ��
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;								//��ռ���ȼ�0 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;										//�����ȼ�0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;										//ʹ���ⲿ�ж�ͨ��
  	NVIC_Init(&NVIC_InitStructure);  	  													//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
    
	System_HW_cur_protect_flag =NULL;
}




//�ⲿ�ж�4�жϴ��� ����NRF24L01 DATA RX
void EXTI4_IRQHandler(void)
{	
	u8 status;
    
	if(EXTI_GetFlagStatus(EXTI_Line4) != RESET)
	{   
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) == 0)
		{
			status = NRF24L01_Read_Reg(READ_REG_NRF+STATUS);
			if(status & RX_OK)																	//RX data occured
			{
				NRF24L01_Read_Buf(RD_RX_PLOAD,Nrf24l01_rx_buff,RX_PLOAD_WIDTH);			//RX FIFO data buffer to NRF buffer
				
				Nrf24l01_Receive_Data_Handle(Nrf24l01_rx_buff,NRF24L01_LEN);				//NRF RX data to USART TX buffer,whaiting for send action
				Nrf24l01_rx_handle_flag = DONE;												//NRF RX data handle is done flag
#ifdef DEBUG
                printf("NRF24l01 RX is done! STATUS=%x \r\n", status);												//Debug RX is done
#endif
                NRF24L01_Write_Reg(FLUSH_RX,0xff);											//Clear RX FIFO buffer 				
			}
			
			else if((status & MAX_TX) > 0)														//TX data times was the max  occured
			{
				NRF24L01_Write_Reg(FLUSH_TX,0xff);											//Clear TX FIFO buffer 
#ifdef DEBUG
				printf("NRF24l01 TX is out of times! \r\n");											//Debug TX is out of times
#endif
				NRF24L01_RX_Mode();														//Change ro RX mode							
			}
			
			else if((status & TX_OK) > 0)														//TX data is done(ACK feedback) occured
			{
				NRF24L01_Write_Reg(FLUSH_TX,0xff);											//Clear TX FIFO buffer 
#ifdef DEBUG
				printf("NRF24l01 TX is done! \r\n");												//Debug TX is done
#endif
				NRF24L01_RX_Mode();														//Change ro RX mode							
			}
            
            NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,status);			  			    //Clear NRF IRQ flag
		}
        
		EXTI_ClearITPendingBit(EXTI_Line4);  							 						//���LINE4�ϵ��жϱ�־λ
	}
}


//�ⲿ�ж�5�жϴ��� ������Ᵽ��
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line5) != RESET)								
	{
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5) == 0)
		{
			System_HW_cur_protect_flag = DONE;
			SYS_SW = OFF;																	//�رն����Դ
		}																	
	}
	EXTI_ClearITPendingBit(EXTI_Line5);  							 							//���LINE5�ϵ��жϱ�־λ
}


//2.4G �������ݴ���
void Nrf24l01_Receive_Data_Handle(u8 *data_buf,u8 len)
{
	u8 sum,cnt;
    
	//************** ckeck sum ***************//
	sum = 0;
	for( cnt = 0; cnt < len - 1; cnt ++ )
    {
        sum += data_buf[cnt];
    }
	sum = sum ^ 0xFF;
    
    //**************************************//
	//***************debug*****************//
#ifdef DEBUG
    printf("NRF24L01 Recv: ");
	for( cnt = 0; cnt < len; cnt ++ )
    {
        printf("%d ",data_buf[cnt]);
    }
    printf("\r\n");
#endif
	//*************************************//
    
	if(!(sum == data_buf[len - 1]))
	{
        return;
    }
	
	if(*(data_buf) == 0xAA)
	{
		Ctrl_state_flag = SYSTEM_CTRL_STATE;
		Steering_engine_L_B_E_angle 	= data_buf[1];
		Steering_engine_L_B_F_angle 	= data_buf[2];
		Steering_engine_L_B_M_angle 	= data_buf[3];
		Steering_engine_L_M_E_angle 	= data_buf[4];
		Steering_engine_L_M_M_angle 	= data_buf[5];
		Steering_engine_L_M_F_angle 	= data_buf[6];
		Steering_engine_L_F_E_angle 	= data_buf[7];
		Steering_engine_L_F_M_angle 	= data_buf[8];
		Steering_engine_L_F_F_angle 	= data_buf[9];
		Steering_engine_R_B_E_angle 	= data_buf[10];
		Steering_engine_R_B_M_angle 	= data_buf[11];
		Steering_engine_R_B_F_angle 	= data_buf[12];
		Steering_engine_R_M_E_angle 	= data_buf[13];
		Steering_engine_R_M_M_angle 	= data_buf[14];
		Steering_engine_R_M_F_angle 	= data_buf[15];
		Steering_engine_R_F_E_angle 	= data_buf[16];
		Steering_engine_R_F_M_angle 	= data_buf[17];
		Steering_engine_R_F_F_angle 	= data_buf[18];
        
		Steering_engine_DRV1_angle = data_buf[19];
        Steering_engine_DRV2_angle = data_buf[20];
        Steering_engine_DRV3_angle = data_buf[21];
        Steering_engine_DRV4_angle = data_buf[22];
        
	}
	else if(*(data_buf) == 0xAB)
	{
		Ctrl_state_flag = SYSTEM_CHECK_STATE;
		Joystick_left_x_axis_val   = data_buf[1];
		Joystick_left_y_axis_val   = data_buf[2];
		Joystick_right_x_axis_val = data_buf[3];
		Joystick_right_y_axis_val = data_buf[4];
	}
}

