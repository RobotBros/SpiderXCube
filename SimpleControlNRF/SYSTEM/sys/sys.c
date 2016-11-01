#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01
//1��ϵͳ��ʼ������  Date 20150808   By  fishcan
//2��ϵͳ״̬ת����ϵͳ״̬չʾ
//********************************************************************************  


u16 Bat_val;									//��ص�ѹADֵ,ȫ�ֱ���
u16 Bat_cur;									//��ص���ADֵ,ȫ�ֱ���
u8 Nrf24l01_tx_buff[NRF24L01_LEN];			//NRF���ͻ��棬ȫ�ֱ���
//u8 bat_avg_cnt;								//��������ƽ��ֵ�����������ֲ�����
u8 average_filter_len_cnt;					//��ѹ/�����˲���λ������
u16 bat_vol_buf[SAMPLE_LEN];  				//��ص�ѹ�������������ֲ�����
u16 bat_cur_buf[SAMPLE_LEN];					//��ص����������������ֲ�����
u16 Bat_adc_converted_value[2];				//ADCֵ DMA������ 0:��ѹ 1:����
u8 System_fault_state = SYSTEM_FAULT_NULL;	//system  fault state


void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
}

//system initial 
void SysInit(void)
{
	u8 Nrf24l01_check_flag = NRF24L01_CHECK_NULL;
	
	System_state_flag = SYSTEM_NULL;
	Old_system_state_flag = SYSTEM_NULL;
	System_initial_state_flag = SYSTEM_INITIALZE_NULL;
	Communicate_state_flag = SYSTEM_LOCK;
	Ctrl_state_flag = SYSTEM_CHECK_STATE;
	System_fault_state = SYSTEM_FAULT_NULL;
	
	delay_init();	    							  				//��ʱ������ʼ��	  
	NVIC_Configuration(); 	 								//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	SYS_SW_Init();											//����ܵ�Դ���ƶ˿ڳ�ʼ��	
	LED_Init();			    			 						//LED�˿ڳ�ʼ��
	BEEP_Init();												//�������˿ڳ�ʼ��	
	Steering_engine_PWM_initial();							//���PWM��ʼ�� 
	Steering_engine_action_initial();   						//��������ϵ縴λ��ʼ��
	Adc_DMA_Init();											// ADC��ѹ��������DMA������ʼ��
	Adc_Init();												//��ز���ADC��ʼ��
	System_pre_sampling();									// Ԥ����������ѹ
    
#ifdef DEBUG
	uart_init(115200);	 				 						//���ڳ�ʼ��Ϊ115200
#endif
    
	NRF24L01_Init();    										//NRF24L01 initial
	if(NRF24L01_Check())
    {
	 	Nrf24l01_check_flag = NRF24L01_CHECK_NULL;
#ifdef DEBUG
		printf("NRF24L01 Error........\r\n"); 
#endif
		LED0 = 0;
    }
	else
	{
		Nrf24l01_check_flag = NRF24L01_CHECK_OK;
#ifdef DEBUG
		printf("NRF24L01 OK........\r\n"); 
#endif
        /* ����2.4G��ʱ���� */
        Timer7_nrf_enable = 1;
	}														//NRF14L01 RX mode IRQ mode enable
    
	BEEP_flash(1,120);										// beep 1 time ,gap 240ms
	
	TIM6_Int_Init(7199,199);									//20 ms  DIV=1 AHB1 = 36M*2  7200*200/72(us)=20ms   	battery voltage and current sampling
	TIM7_Int_Init(7199,999);									// 100ms system display and NRF/BT send data time base
	
	if(Nrf24l01_check_flag == NRF24L01_CHECK_OK)				//check NRF24l01 initial is done?
    {
        System_initial_state_flag = SYSTEM_INITIALZE_DONE;
        NRF24L01_RX_Mode();   								//NRF24L01 TX mode enable
        EXTIX_Init_Nrfirq();							    //NRF24L01 RX IRQ interrupt initial,handle RX data 
    }
	else
    {
        System_initial_state_flag = SYSTEM_INITIALZE_NULL;
    }	
    
	SYS_SW = ON;
	EXTIX_Init_curkill();										//������������ʼ��
    
#ifdef DEBUG
    printf("System Initial is done...... \r\n"); 
#endif
}

//ϵͳ״̬չʾ
void System_state_display(void)
{
    
    if(System_state_flag != Old_system_state_flag)				//system state is change
    {
        switch(System_state_flag)
        {
            case SYSTEM_NULL:
            {
                LED0 = 0;																						//LED ON
                break;
            }						
            
            case SYSTEM_STAND_BY:
            {
                System_tim7_display_flag = SYSTEM_DISPLAY_1S;
                Timer7_base_cnt = 0;
                break;
            }		
            
            case SYSTEM_NORMAL:
            {
                System_tim7_display_flag = SYSTEM_DISPLAY_500MS;
                Timer7_base_cnt = 0;
                BEEP_flash(1,150);														// beep 1 time ,gap 300ms
                break;
            }	
            
            case SYSTEM_NORMAL_WORRING:
            {
                System_tim7_display_flag = SYSTEM_DISPLAY_500MS;
                Timer7_base_cnt = 0;
                BEEP_flash(3,150);														// beep 3 time ,gap 300ms	
                break;
            }	
            
            case SYSTEM_CTRL:
            {
                System_tim7_display_flag = SYSTEM_DISPLAY_200MS;
                Timer7_base_cnt = 0;
                BEEP_flash(1,150);														// beep 1 time ,gap 300ms
                break;
            }
            
            case SYSTEM_CTRL_WORRING:
            {
                System_tim7_display_flag = SYSTEM_DISPLAY_200MS;
                Timer7_base_cnt = 0;
                BEEP_flash(3,150);														// beep 3 time ,gap 300ms
                break;
            }
            
            case SYSTEM_STOP:
            {															
                System_fault_display();	
                break;
            }
            
            default:
            {
                System_tim7_display_flag = SYSTEM_DISPLAY_1S;
                Timer7_base_cnt = 0;
                LED0 = 1;
                BEEP = OFF;
                break;
            }
            
        }
        Old_system_state_flag = System_state_flag;
    }	
    
}


//Ԥ���� 
void System_pre_sampling(void)
{
	u8 cnt;
	u32 sum1 =0; 
	u32 sum2 = 0;
	
	for(cnt=0;cnt<SAMPLE_LEN;cnt++)
	{
		//bat_vol_buf[cnt] = Get_Adc(BATTERY_AD_VOL_CHANNEL);    			//��ѹ
		
		delay_ms(10);
		bat_vol_buf[cnt]  = Bat_adc_converted_value[0];
		bat_cur_buf[cnt] = Bat_adc_converted_value[1];	
	}
	
	for(cnt=0;cnt<SAMPLE_LEN;cnt++)
	{
		sum2 +=  bat_cur_buf[cnt];
		sum1 +=  bat_vol_buf[cnt];
	}
	Bat_val = sum1 / SAMPLE_LEN;	
	Bat_cur = sum2 / SAMPLE_LEN;
}

//����ƽ���˲� ��ѹ , ����
void System_moving_average_filter(void)
{
	u8 cnt;
	u32 sum_vol = 0;
	u32 sum_cur = 0;
    
	bat_vol_buf[average_filter_len_cnt]  = Bat_adc_converted_value[0];
	bat_cur_buf[average_filter_len_cnt]  = Bat_adc_converted_value[1];
    
	if(average_filter_len_cnt == SAMPLE_LEN-1)
    {average_filter_len_cnt = 0;}
	else
    {average_filter_len_cnt ++;}
	
	for(cnt=0;cnt<SAMPLE_LEN;cnt++)
	{
		sum_vol  +=  bat_vol_buf[cnt];
		sum_cur  += bat_cur_buf[cnt];
	}		
    
	Bat_val = (u16)(sum_vol /SAMPLE_LEN);
	Bat_cur = (u16)(sum_cur /SAMPLE_LEN);
	
}

//ϵͳ״̬�ж� 20ms ����һ�Σ����˲�
void System_state_check(void)  
{
	if(System_initial_state_flag == SYSTEM_INITIALZE_DONE)             						//check system initial is done 
	{	
		System_moving_average_filter();												// ֻ�Ե�ѹ������ƽ��ֵ�˲�
		//Bat_cur = Bat_adc_converted_value[1];										// ����ֱ��ȡDMA��������������˲�
		
		if((Bat_val < BATTERY_OVER_VALTAGE) && (Bat_val > BATTERY_UNDER_VALTAGE) && (Bat_cur < BATTERY_OVER_CURRENT))       // Bat <= 7.4V and Bat => 8.5V STOP
		{
			if(Ctrl_state_flag == SYSTEM_CTRL_STATE)									//PC��λ������״̬����
			{
				if(Bat_val > BATTERY_WRRING_VALTAGE)
                {System_state_flag =  SYSTEM_CTRL;}
				else
                {System_state_flag =  SYSTEM_CTRL_WORRING;}
			}
			
			else if(Ctrl_state_flag == SYSTEM_CHECK_STATE)							//ң����λ������״̬����
			{
				if(Communicate_state_flag == SYSTEM_LOCK)							//ң�ؽ���״̬�ж�
				{
					System_state_flag = SYSTEM_STAND_BY;
				}
				else if(Communicate_state_flag == SYSTEM_UNLOCK)
				{
					if(Bat_val > BATTERY_WRRING_VALTAGE)	
                    {System_state_flag =  SYSTEM_NORMAL;}
					else
                    {System_state_flag =  SYSTEM_NORMAL_WORRING;}
				}			
			}
		}
		else
		{
			System_state_flag = SYSTEM_STOP;
			SYS_SW = OFF;
			//Steering_engine_all_off();
		}
        
		System_fault_check();			//���ϵͳ���Ϻ�
	}					
}

//ϵͳ�����ж�
void System_lock_check(void)
{
	if(Communicate_state_flag == SYSTEM_LOCK)
	{
		if((Joystick_left_y_axis_val > 90) && (Joystick_right_y_axis_val > 90))
		{
			delay_ms(500);
			if((Joystick_left_y_axis_val > 90) && (Joystick_right_y_axis_val > 90))
            {Communicate_state_flag = SYSTEM_UNLOCK;}
		}
	}
}

//ϵͳ2.4G ����֡���ݴ���
void System_nrf_tx_flame(void)
{
	u8 sum,cnt;
    
	Nrf24l01_tx_buff[0] = 0xAC;								//ͷ��
    //********************* DATA Handle *******************//
	Nrf24l01_tx_buff[1] 	= (Bat_val & 0xFF00) >> 8;			//Bat_slave_high_val  ,slave battery AD VOL high byte value
	Nrf24l01_tx_buff[2] 	= Bat_val & 0x00FF;					//Bat_slave_low_val  ,slave battery AD VOL low byte value
	Nrf24l01_tx_buff[3] 	= (Bat_cur & 0xFF00) >> 8;			//Bat_slave_high_val  ,slave battery AD CUR high byte value
	Nrf24l01_tx_buff[4] 	= Bat_cur & 0x00FF;					//Bat_slave_low_val  ,slave battery AD CUR low byte value
	
	for(cnt=5; cnt<NRF24L01_LEN; cnt++)						//not use bytes is clearing
	{
		Nrf24l01_tx_buff[cnt] = 0;
	}
    //*********************CHECK SUM*********************//
	sum = 0;
	for( cnt = 0; cnt < NRF24L01_LEN - 1; cnt ++ )
	{
        sum += Nrf24l01_tx_buff[cnt];
    }
	sum = sum ^ 0xFF;
	Nrf24l01_tx_buff[NRF24L01_LEN - 1] = sum;
    
    //**********************debug*************************//
#ifdef DEBUG
    printf("NRF24L01 Send: ");
	for( cnt = 0; cnt < NRF24L01_LEN; cnt++ )
	{
		printf("%x ",Nrf24l01_tx_buff[cnt]);
	}
	printf("\r\n");	
#endif 
    
}

//ϵͳ����2.4G ���ݴ���
void System_nrf_tx_handle(void)
{
	if(Nrf24l01_rx_handle_flag == DONE)
	{
		//nrf_send_data();
		System_nrf_tx_flame();
		
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,0xFF); 		//���״̬��־
		NRF24L01_Write_Reg(FLUSH_RX,0xff);						//Clear RX FIFO data buffer
		NRF24L01_Write_Reg(FLUSH_TX,0xff);						//Clear TX FIFO buffer 
		NRF24L01_TX_Mode();
		delay_ms(1);
		NRF24L01_CE=0;
  		NRF24L01_Write_Buf(WR_TX_PLOAD,Nrf24l01_tx_buff,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 		NRF24L01_CE=1;//��������	   
 		
		Nrf24l01_rx_handle_flag = NULL;		
	}
}

//ϵͳ���Ϻż��
void System_fault_check(void)
{
	if((Bat_val > BATTERY_OVER_VALTAGE) || (Bat_val < BATTERY_UNDER_VALTAGE)) 
	{
		System_fault_state = SYSTEM_FAULT_VOL_SOFT;
		System_state_flag = SYSTEM_STOP;
		TIM_Cmd(TIM6, DISABLE); 
	}
	
	else if(Bat_cur >= BATTERY_OVER_CURRENT)
	{
		System_fault_state = SYSTEM_FAULT_OVER_CUR_SOFT;
		System_state_flag = SYSTEM_STOP;
		TIM_Cmd(TIM6, DISABLE); 
	}
	
	//else if(PER)										//���ʱ���Ԥ��
	//	{System_fault_state = SYSTEM_FAULT_OVER_PWR;}
	
	else if(System_HW_cur_protect_flag == DONE)
	{
		System_fault_state = SYSTEM_FAULT_OVER_CUR_HW;
		System_state_flag = SYSTEM_STOP;
		TIM_Cmd(TIM6, DISABLE); 
	}
	
	else
    {System_fault_state = SYSTEM_FAULT_NULL;}
}

//ϵͳ���Ϻ���ʾ
void System_fault_display(void)
{
	u8 fault_display_gap_cnt;
    
	switch(System_fault_state)
	{
        case SYSTEM_FAULT_NULL:
        break;
        
        case SYSTEM_FAULT_VOL_SOFT:		// �����������ѹ����Ƿѹ����
		{
			Timer7_base_cnt = 0;		// ����������ֵ
			
			while(Timer7_base_cnt < SYSTEM_FAULT_DISPLAY_RETURN_MS)   //�����ӹ�����������������λ
			{
				LED_flash_short(System_fault_state);
				for(fault_display_gap_cnt=0;fault_display_gap_cnt<SYSTEM_FAULT_DISPLAY_GAP_SEC;fault_display_gap_cnt++)  //������ʾ��� 5s
                {delay_ms(1000);}		
			}
			System_fault_state = SYSTEM_FAULT_NULL;
			System_state_flag = SYSTEM_STAND_BY;
			TIM_Cmd(TIM6, ENABLE); 
			Timer7_base_cnt = 0;	
			Steering_engine_action_initial();
			SYS_SW = ON;
			break;
		}
        
        case SYSTEM_FAULT_OVER_CUR_SOFT:		// �����������������
		{
			Timer7_base_cnt = 0;				// ����������ֵ
			
			while(Timer7_base_cnt < SYSTEM_FAULT_DISPLAY_RETURN_MS)
			{
				LED_flash_short(System_fault_state);
				for(fault_display_gap_cnt=0;fault_display_gap_cnt<SYSTEM_FAULT_DISPLAY_GAP_SEC;fault_display_gap_cnt++)
                {delay_ms(1000);}		
			}
			System_fault_state = SYSTEM_FAULT_NULL;
			System_state_flag = SYSTEM_STAND_BY;
			TIM_Cmd(TIM6, ENABLE); 
			Timer7_base_cnt = 0;	
			Steering_engine_action_initial();
			SYS_SW = ON;
			break;
		}
        
        case SYSTEM_FAULT_OVER_PWR:			// �����ʹ���
		{
			Timer7_base_cnt = 0;				// ����������ֵ
			
			while(Timer7_base_cnt < SYSTEM_FAULT_DISPLAY_RETURN_MS)
			{
				LED_flash_short(System_fault_state);
				for(fault_display_gap_cnt=0;fault_display_gap_cnt<SYSTEM_FAULT_DISPLAY_GAP_SEC;fault_display_gap_cnt++)
                {delay_ms(1000);}		
			}
			System_fault_state = SYSTEM_FAULT_NULL;
			System_state_flag = SYSTEM_STAND_BY;
			TIM_Cmd(TIM6, ENABLE); 
			Timer7_base_cnt = 0;	
			Steering_engine_action_initial();
			SYS_SW = ON;
			break;
		}
        
        case SYSTEM_FAULT_OVER_CUR_HW:	// Ӳ������������ֵ����
		{
			Timer7_base_cnt = 0;			// ����������ֵ
			
			while(Timer7_base_cnt < SYSTEM_FAULT_DISPLAY_RETURN_MS)
			{
				LED_flash_short(System_fault_state);
				for(fault_display_gap_cnt=0;fault_display_gap_cnt<SYSTEM_FAULT_DISPLAY_GAP_SEC;fault_display_gap_cnt++)
                {delay_ms(1000);}		
			}
			System_HW_cur_protect_flag = NULL;
			System_fault_state = SYSTEM_FAULT_NULL;
			System_state_flag = SYSTEM_STAND_BY;
			TIM_Cmd(TIM6, ENABLE); 
			Timer7_base_cnt = 0;	
			Steering_engine_action_initial();
			SYS_SW = ON;
			break;
		}
        
        default:
		{
			System_HW_cur_protect_flag = NULL;
			System_fault_state = SYSTEM_FAULT_NULL;
			System_state_flag = SYSTEM_STAND_BY;
			TIM_Cmd(TIM6, ENABLE); 
			Timer7_base_cnt = 0;						// ����������ֵ
			break;
		}
		
	}
    
}



