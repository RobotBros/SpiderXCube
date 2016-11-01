 #include "sys.h"

u8 Nrf24l01_tx_buff[NRF42L01_LEN];
extern u8 Nrf24l01_rx_buff[NRF42L01_LEN];
u16 bat_val;
_system_ini_type System_ini_state;

u8 System_fault_flag = SYSTEM_NO_FAULT;
static u8 system_fault_check = 0;
static u8 system_fault_counter_start =0;		// 故障计时启动信号
static u8 system_fault_counter_end =0;		// 故障计时结束信号
static u8 system_nrf_tx_check =0;
static u8 system_nrf_rx_check=0;

static signed char gyro_orientation[9] = {1,   0,  0,		//X						|
                                          			 0,   1,  0,		//Y						| 
                                          			 0,   0,  1};	   	//Z 定义航向正方向   Y<----Z    Pitch x轴旋转；Roll Y轴；Yaw Z轴 

//////////////////////////////////////////////////////////////////////////////////	 
//V01
//1、系统初始化函数  Date 20150808   By  fishcan
//2、系统状态转换，系统状态展示
//********************************************************************************  
void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
}

//system initial 
void SysInit(void)
{
	int mpu_init_flag;
	
	System_variable_init();
	delay_init();											// system tick timer 初始化
	NVIC_Configuration(); 								// 中断优先级组别设置
	LED_Init(LED_SCALE,LED_DIV);							// LED_RGB_PWM初始化	
	//LCD_Nokia5110_Init(LCD_BL_SCALE,LCD_BL_DIV);		// LCD nokia I/O BL_PWM初始化	
	Lcd_144inch_Init();									// LCD 1.44 初始化
	Adc_DMA_Init();										// ADC DMA初始化								
	Adc_Init();											// ADC初始化，连续装换
	NRF24L01_Init();   									// NRF 初始化 																		
	EXTIX_Init();										// NRF INT初始化
//------------------------NRF模块检测-----------------------------
	if(NRF24L01_Check())
	{
		System_fault_flag = SYSTEM_NRF_NULL;
		System_ini_state.nrf_ini_state =NOT;
	}
	else
	{
		System_fault_flag = SYSTEM_NO_FAULT;
		System_ini_state.nrf_ini_state =DONE;
		NRF24L01_TX_Mode();   	
	}
//----------------------------------------------------------------	
	BEEP_Init(BEEP_SCALE, BEEP_DIV);						// 蜂鸣器PWM初始化
	buzzer_power_on();									// 蜂鸣器设置开机音乐类型
	Key_init();											// 按键初始化
	i2c_Init();											// I2C初始化
//----------------------------------------------------------------
	TIM4_Int_Init(7199,19);								// 系统函数运行判定时钟 2ms 0.5k
	TIM2_Int_Init(7199,9999);								// Start 2.4G RX超时计数器  1s
	LCD_power_on_action();								// 系统开机画面	
//------------------------MPU6050 DMP初始化-----------------------
	mpu_init_flag = mpu_init();
	if(!mpu_init_flag)	
	{  
		GUI_ShowString(10,24,12,"MPU6050 ini OK..",0);						
	
	 	if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))			 //mpu_set_sensor
		 	GUI_ShowString(10,36,12,"MPU set OK..",0);
		 else
	  		 {Point_color = RED;GUI_ShowString(0,36,12,"MPU set error..",0);Point_color = GREEN;}
	  	delay_ms(500);
	  	if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))	   	  //mpu_configure_fifo
	  		GUI_ShowString(10,48,12,"MPU FIFO OK..",0);
	  	else
			{Point_color = RED;GUI_ShowString(10,48,12,"MPU FIFO error..",0);Point_color = GREEN;}
		delay_ms(500);
	  	if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))	   	 			 //mpu_set_sample_rate
	  		GUI_ShowString(10,60,12,"MPU sam_rate OK..",0);
	  	else
			{Point_color = RED;GUI_ShowString(10,60,12,"MPU sam_rate error..",0);Point_color = GREEN;}
		delay_ms(500);
	  	if(!dmp_load_motion_driver_firmware())   	 				 //dmp_load_motion_driver_firmvare
	  		GUI_ShowString(10,72,12,"MPU firmware OK..",0);
	 	else
			{Point_color = RED;GUI_ShowString(10,72,12,"MPU firmware error..",0);Point_color = GREEN;}
		delay_ms(500);
	 	 if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) 	  //dmp_set_orientation
	 	 	GUI_ShowString(10,84,12,"MPU orient OK..",0);
	  	else
			{Point_color = RED;GUI_ShowString(10,84,12,"MPU orient error..",0);Point_color = GREEN;}
		delay_ms(500);
	  	if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        					     DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        					     DMP_FEATURE_GYRO_CAL))		   	  //dmp_enable_feature
	        	GUI_ShowString(10,96,12,"MPU feature OK..",0);				     
	  	else
			{Point_color = RED;GUI_ShowString(10,96,12,"MPU feature error..",0);Point_color = GREEN;}
		delay_ms(500);
	  	if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))   	  				//dmp_set_fifo_rate
	  		GUI_ShowString(10,108,12,"MPU FIFO_rate OK..",0);
	 	else
			{Point_color = RED;GUI_ShowString(10,108,12,"MPU FIFO_rate error..",0);Point_color = GREEN;}

		Lcd_Init_Clear(Back_color);
		
	  	run_self_test();
		
	 	 if(!mpu_set_dmp_state(1))
	 	 {
		 	GUI_ShowString(10,24,12,"MPU state OK..",0);
			delay_ms(500);
			GUI_ShowString(2,36,12,"System ini is all OK",0);
			delay_ms(1000);
	 	 }
	  	 else
			{Point_color = RED;GUI_ShowString(10,24,12,"MPU state error..",0);}
	}

	Point_color = RED;
	Lcd_Clear(Back_color,0,0);	
	GUI_ShowString(20,0,12,"Spider_System",0);	
	Point_color = GREEN;
}

//----------------------------------------------------------------
//系统变量初始化
//----------------------------------------------------------------
void System_variable_init(void)
{
	System_ini_state.led_ini_state = NOT;
	System_ini_state.adc_ini_state =NOT;
	System_ini_state.beep_ini_state =NOT;
	System_ini_state.dma_ini_state =NOT;
	System_ini_state.key_ini_state =NOT;
	System_ini_state.lcd_ini_state =NOT;
	System_ini_state.nrf_ini_state =NOT;
	System_ini_state.system_timer_ini_state =NOT;
	System_fault_flag = SYSTEM_NO_FAULT;
}

//----------------------------------------------------------------
//主端数据2.4G发送处理
//----------------------------------------------------------------
void Syterm_master_nrf_tx_handle(void)
{
	u8 cnt,sum;

	if(system_nrf_tx_check)
	{
		system_nrf_tx_check = 0;
		Nrf24l01_tx_buff[0] = 0xAB;						//头码
//********************master data handle***************************//		
		Nrf24l01_tx_buff[1] = Pre_ladx;
		Nrf24l01_tx_buff[2] = Pre_lady;
		Nrf24l01_tx_buff[3] = Pre_radx;
		Nrf24l01_tx_buff[4] = Pre_rady;

		for(cnt=5;cnt<NRF42L01_LEN;cnt++)					//NRF TX data 未使用字节清零处理
			{Nrf24l01_tx_buff[cnt] = 0;}
//***********************check sum******************************//				
		sum = 0;
		sum = Nrf24l01_tx_buff[0] + Nrf24l01_tx_buff[1] + Nrf24l01_tx_buff[2] + Nrf24l01_tx_buff[3] + Nrf24l01_tx_buff[4];
		sum = sum ^ 0xFF;
		Nrf24l01_tx_buff[20] = sum;		
		//NRF24L01_TX_Mode();
		//delay_ms(1);
		NRF24L01_Write_Reg(FLUSH_TX,0xff);						//Clear TX FIFO buffer 
		NRF24L01_CE=0;
  		NRF24L01_Write_Buf(WR_TX_PLOAD,Nrf24l01_tx_buff,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 		NRF24L01_CE=1;//启动发送	
	}
}

//-------------------------------------------------------
 // 2.4G RX接收数据处理
 //------------------------------------------------------
void Syterm_master_nrf_rx_handle(void)
{
	u8 sum,cnt;
	u16 vol_value,cur_value;
	u32 vol,cur,pr;

	if(system_nrf_rx_check)
	{
		system_nrf_rx_check =0;
		if(Nrf24l01_rx_handle_flag == DONE)
		{
			
//************** ckeck sum ***************//
			sum = 0;
			for(cnt=0;cnt<(NRF42L01_LEN-13);cnt++)
				{sum += Nrf24l01_rx_buff[cnt];}
			sum = sum ^ 0xFF;
			if(!(sum == Nrf24l01_rx_buff[20]))		
				{return;}		
			if(*(Nrf24l01_rx_buff) == 0xAC)
			{
				//LCD显示下位机电压，电流，功率等
				vol_value = (Nrf24l01_rx_buff[1]<<8) | Nrf24l01_rx_buff[2];
				cur_value = (Nrf24l01_rx_buff[3]<<8) | Nrf24l01_rx_buff[4];
				vol = vol_value*165 / 512;							//放大100倍 3.3*100*4/4096 下位机电压
				cur = cur_value*275 / 1536;							//放大10倍  3.3*1000*10/4096/15 下位机电流
				pr = vol*cur/10;									//放大100倍 
//------------------1.44 TFT--------------------------------------//
				if(System_key_type.new_key_type == KEY_PRESS_UP)
				{
#ifdef	LCD_144INCH
					GUI_ShowString(10,82,12,"VL:",0);					//12*6 size 上位机电压
					GUI_ShowNum(28,82,vol/100,2,12);					// 整数位 2
					GUI_ShowString(40,82,12,".",0);
					GUI_ShowNum(46,82,(vol - (vol/100)*1000),2,12);	// 小数位 2
					GUI_ShowString(64,82,12,"V",0);
					
					GUI_ShowString(10,94,12,"CR:",0);					// 上位机电流
					GUI_ShowNum(28,94,cur/10,2,12);					// 整数位 2
					GUI_ShowString(40,94,12,".",0);
					GUI_ShowNum(46,94,(cur - (cur/10)*100),1,12);		// 小数位 1
					GUI_ShowString(64,94,12,"A",0);

					GUI_ShowString(10,106,12,"PR:",0);					// 上位机功率
					GUI_ShowNum(28,106,pr/100,2,12);					// 整数位 3
					GUI_ShowString(40,106,12,".",0);
					GUI_ShowNum(46,106,(pr - (pr/100)*1000),2,12);		// 小数位 2
					GUI_ShowString(64,106,12,"W",0);
#else
//------------------Nokia5110-------------------------------------//				
				//LCD_write_String(1, 	4, "Vol:");		 			//y:4
				//LCD_write_number(24,	3, vol/100, 1);    			//X:8*6,取电压整数位
				//LCD_write_String(30,	3, ".");			 			//x:48+6
				//LCD_write_number(36,	3, vol - (vol/100)*100 , 2);   //X:54+6,取电压小数部分
				//LCD_write_String(72,	3, "V");			 			//x:60+12		
#endif
				}
			}
			Nrf24l01_rx_handle_flag=NOT;
		}
	}
}

//--------------------------------------------
//系统2.4G TX定时		240ms
//--------------------------------------------
void System_nrf_tx_timer(void)
{
	static u8 tick_4ms = 0;

	tick_4ms++;
	
	if (tick_4ms >= 60)			// 4ms*60 = 240ms
	{	
		tick_4ms=0;
		system_nrf_tx_check= 1;
	}
}

//--------------------------------------------
//系统2.4G RX数据处理定时		300ms
//--------------------------------------------
void System_nrf_rx_timer(void)
{
	static u8 tick_4ms = 0;

	tick_4ms++;
	
	if (tick_4ms >= 75)			// 4ms*115 = 30s
	{	
		tick_4ms=0;
		system_nrf_rx_check= 1;
	}
}

//--------------------------------------------
//系统状态清除定时器   0.5min
//--------------------------------------------
void System_fault_clear_timer(void)
{
	static u8 tick_4ms = 0;

	if(system_fault_counter_start == 1)
	{
		tick_4ms++;
	
		if (tick_4ms >= 7500)			// 4ms*7550 = 30s
		{	
			tick_4ms=0;
			system_fault_counter_end = 1;
		}
	}
}


//--------------------------------------------
//系统状态清除
//--------------------------------------------
void System_fault_clear_handle(void)
{
	if(system_fault_counter_end ==1)
	{
		System_fault_flag = SYSTEM_NO_FAULT;
		//System_state_flag = SYSTEM_NORMAL;
		system_fault_counter_start = 0;
	}
}

//--------------------------------------
//系统故障定时时间 200ms
//--------------------------------------
void System_fault_check_timer(void)
{
	static u8 tick_4ms = 0;

	tick_4ms++;
	
	if (tick_4ms >= 50)			// 4ms*50 = 200ms
	{	
		tick_4ms=0;
		system_fault_check = 1;
	}
}

//---------------------------------------------
//系统故障处理函数
//---------------------------------------------
void System_fault_check_handle(void)
{
	static u8 fault_temp_falg = SYSTEM_NO_FAULT;

	if(system_fault_check == 1)
	{
		system_fault_check = 0;
		if(fault_temp_falg != System_fault_flag)
		{
			fault_temp_falg = System_fault_flag;
		
			if(fault_temp_falg == SYSTEM_NO_FAULT)  {return;}
			else if(fault_temp_falg == SYSTEM_NRF_NULL)
			{
				LED_RGB_turnoff();
				buzzer_sound(SOUND_TURN_OFF);
				//LCD_BL_PWM_duty_set(0);
				LCD_fault_display(fault_temp_falg);
				LED_G_shadow(50);						//绿灯亮
				system_fault_counter_start = 1;			//计时0.5分钟清故障
			}
			else if(fault_temp_falg == SYSTEM_VOLTAGE_FAULT)
			{
				LED_RGB_turnoff();
				buzzer_sound(SOUND_TURN_OFF);
				//LCD_BL_PWM_duty_set(0);
				LCD_fault_display(fault_temp_falg);
				LED_R_shadow(50);						//红灯亮
				system_fault_counter_start = 1;			//计时0.5分钟清故障
			}
		}
	}
}


