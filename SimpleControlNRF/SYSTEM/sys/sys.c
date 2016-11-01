#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V01
//1、系统初始化函数  Date 20150808   By  fishcan
//2、系统状态转换，系统状态展示
//********************************************************************************  


u16 Bat_val;									//电池电压AD值,全局变量
u16 Bat_cur;									//电池电流AD值,全局变量
u8 Nrf24l01_tx_buff[NRF24L01_LEN];			//NRF发送缓存，全局变量
//u8 bat_avg_cnt;								//采样计算平均值计数触发，局部变量
u8 average_filter_len_cnt;					//电压/电流滤波移位计数器
u16 bat_vol_buf[SAMPLE_LEN];  				//电池电压采样缓存器，局部变量
u16 bat_cur_buf[SAMPLE_LEN];					//电池电流采样缓存器，局部变量
u16 Bat_adc_converted_value[2];				//ADC值 DMA缓存器 0:电压 1:电流
u8 System_fault_state = SYSTEM_FAULT_NULL;	//system  fault state


void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
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
	
	delay_init();	    							  				//延时函数初始化	  
	NVIC_Configuration(); 	 								//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	SYS_SW_Init();											//舵机总电源控制端口初始化	
	LED_Init();			    			 						//LED端口初始化
	BEEP_Init();												//蜂鸣器端口初始化	
	Steering_engine_PWM_initial();							//舵机PWM初始化 
	Steering_engine_action_initial();   						//舵机动作上电复位初始化
	Adc_DMA_Init();											// ADC电压、电流，DMA采样初始化
	Adc_Init();												//电池采样ADC初始化
	System_pre_sampling();									// 预采样电流电压
    
#ifdef DEBUG
	uart_init(115200);	 				 						//串口初始化为115200
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
        /* 启用2.4G定时发送 */
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
	EXTIX_Init_curkill();										//过电流保护初始化
    
#ifdef DEBUG
    printf("System Initial is done...... \r\n"); 
#endif
}

//系统状态展示
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


//预采样 
void System_pre_sampling(void)
{
	u8 cnt;
	u32 sum1 =0; 
	u32 sum2 = 0;
	
	for(cnt=0;cnt<SAMPLE_LEN;cnt++)
	{
		//bat_vol_buf[cnt] = Get_Adc(BATTERY_AD_VOL_CHANNEL);    			//电压
		
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

//滑动平均滤波 电压 , 电流
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

//系统状态判断 20ms 采样一次，并滤波
void System_state_check(void)  
{
	if(System_initial_state_flag == SYSTEM_INITIALZE_DONE)             						//check system initial is done 
	{	
		System_moving_average_filter();												// 只对电压做滑动平均值滤波
		//Bat_cur = Bat_adc_converted_value[1];										// 电流直接取DMA采样，不做软件滤波
		
		if((Bat_val < BATTERY_OVER_VALTAGE) && (Bat_val > BATTERY_UNDER_VALTAGE) && (Bat_cur < BATTERY_OVER_CURRENT))       // Bat <= 7.4V and Bat => 8.5V STOP
		{
			if(Ctrl_state_flag == SYSTEM_CTRL_STATE)									//PC上位机控制状态操作
			{
				if(Bat_val > BATTERY_WRRING_VALTAGE)
                {System_state_flag =  SYSTEM_CTRL;}
				else
                {System_state_flag =  SYSTEM_CTRL_WORRING;}
			}
			
			else if(Ctrl_state_flag == SYSTEM_CHECK_STATE)							//遥控上位机控制状态操作
			{
				if(Communicate_state_flag == SYSTEM_LOCK)							//遥控解锁状态判断
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
        
		System_fault_check();			//检测系统故障号
	}					
}

//系统解锁判断
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

//系统2.4G 发送帧数据处理
void System_nrf_tx_flame(void)
{
	u8 sum,cnt;
    
	Nrf24l01_tx_buff[0] = 0xAC;								//头码
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

//系统发送2.4G 数据处理
void System_nrf_tx_handle(void)
{
	if(Nrf24l01_rx_handle_flag == DONE)
	{
		//nrf_send_data();
		System_nrf_tx_flame();
		
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,0xFF); 		//清除状态标志
		NRF24L01_Write_Reg(FLUSH_RX,0xff);						//Clear RX FIFO data buffer
		NRF24L01_Write_Reg(FLUSH_TX,0xff);						//Clear TX FIFO buffer 
		NRF24L01_TX_Mode();
		delay_ms(1);
		NRF24L01_CE=0;
  		NRF24L01_Write_Buf(WR_TX_PLOAD,Nrf24l01_tx_buff,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 		NRF24L01_CE=1;//启动发送	   
 		
		Nrf24l01_rx_handle_flag = NULL;		
	}
}

//系统故障号检测
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
	
	//else if(PER)										//功率保护预留
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

//系统故障号显示
void System_fault_display(void)
{
	u8 fault_display_gap_cnt;
    
	switch(System_fault_state)
	{
        case SYSTEM_FAULT_NULL:
        break;
        
        case SYSTEM_FAULT_VOL_SOFT:		// 软件采样，过压或者欠压故障
		{
			Timer7_base_cnt = 0;		// 重启计数器值
			
			while(Timer7_base_cnt < SYSTEM_FAULT_DISPLAY_RETURN_MS)   //三分钟故障清除，舵机动作复位
			{
				LED_flash_short(System_fault_state);
				for(fault_display_gap_cnt=0;fault_display_gap_cnt<SYSTEM_FAULT_DISPLAY_GAP_SEC;fault_display_gap_cnt++)  //故障显示间隔 5s
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
        
        case SYSTEM_FAULT_OVER_CUR_SOFT:		// 软件采样过电流故障
		{
			Timer7_base_cnt = 0;				// 重启计数器值
			
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
        
        case SYSTEM_FAULT_OVER_PWR:			// 过功率故障
		{
			Timer7_base_cnt = 0;				// 重启计数器值
			
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
        
        case SYSTEM_FAULT_OVER_CUR_HW:	// 硬件过电流，峰值保护
		{
			Timer7_base_cnt = 0;			// 重启计数器值
			
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
			Timer7_base_cnt = 0;						// 重启计数器值
			break;
		}
		
	}
    
}



