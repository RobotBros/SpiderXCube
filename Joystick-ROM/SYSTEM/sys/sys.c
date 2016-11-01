 #include "sys.h"

u8 Nrf24l01_tx_buff[NRF42L01_LEN];
extern u8 Nrf24l01_rx_buff[NRF42L01_LEN];
u16 bat_val;
_system_ini_type System_ini_state;

u8 System_fault_flag = SYSTEM_NO_FAULT;
static u8 system_fault_check = 0;
static u8 system_fault_counter_start =0;		// ���ϼ�ʱ�����ź�
static u8 system_fault_counter_end =0;		// ���ϼ�ʱ�����ź�
static u8 system_nrf_tx_check =0;
static u8 system_nrf_rx_check=0;

static signed char gyro_orientation[9] = {1,   0,  0,		//X						|
                                          			 0,   1,  0,		//Y						| 
                                          			 0,   0,  1};	   	//Z ���庽��������   Y<----Z    Pitch x����ת��Roll Y�᣻Yaw Z�� 

//////////////////////////////////////////////////////////////////////////////////	 
//V01
//1��ϵͳ��ʼ������  Date 20150808   By  fishcan
//2��ϵͳ״̬ת����ϵͳ״̬չʾ
//********************************************************************************  
void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
}

//system initial 
void SysInit(void)
{
	int mpu_init_flag;
	
	System_variable_init();
	delay_init();											// system tick timer ��ʼ��
	NVIC_Configuration(); 								// �ж����ȼ��������
	LED_Init(LED_SCALE,LED_DIV);							// LED_RGB_PWM��ʼ��	
	//LCD_Nokia5110_Init(LCD_BL_SCALE,LCD_BL_DIV);		// LCD nokia I/O BL_PWM��ʼ��	
	Lcd_144inch_Init();									// LCD 1.44 ��ʼ��
	Adc_DMA_Init();										// ADC DMA��ʼ��								
	Adc_Init();											// ADC��ʼ��������װ��
	NRF24L01_Init();   									// NRF ��ʼ�� 																		
	EXTIX_Init();										// NRF INT��ʼ��
//------------------------NRFģ����-----------------------------
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
	BEEP_Init(BEEP_SCALE, BEEP_DIV);						// ������PWM��ʼ��
	buzzer_power_on();									// ���������ÿ�����������
	Key_init();											// ������ʼ��
	i2c_Init();											// I2C��ʼ��
//----------------------------------------------------------------
	TIM4_Int_Init(7199,19);								// ϵͳ���������ж�ʱ�� 2ms 0.5k
	TIM2_Int_Init(7199,9999);								// Start 2.4G RX��ʱ������  1s
	LCD_power_on_action();								// ϵͳ��������	
//------------------------MPU6050 DMP��ʼ��-----------------------
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
//ϵͳ������ʼ��
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
//��������2.4G���ʹ���
//----------------------------------------------------------------
void Syterm_master_nrf_tx_handle(void)
{
	u8 cnt,sum;

	if(system_nrf_tx_check)
	{
		system_nrf_tx_check = 0;
		Nrf24l01_tx_buff[0] = 0xAB;						//ͷ��
//********************master data handle***************************//		
		Nrf24l01_tx_buff[1] = Pre_ladx;
		Nrf24l01_tx_buff[2] = Pre_lady;
		Nrf24l01_tx_buff[3] = Pre_radx;
		Nrf24l01_tx_buff[4] = Pre_rady;

		for(cnt=5;cnt<NRF42L01_LEN;cnt++)					//NRF TX data δʹ���ֽ����㴦��
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
  		NRF24L01_Write_Buf(WR_TX_PLOAD,Nrf24l01_tx_buff,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 		NRF24L01_CE=1;//��������	
	}
}

//-------------------------------------------------------
 // 2.4G RX�������ݴ���
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
				//LCD��ʾ��λ����ѹ�����������ʵ�
				vol_value = (Nrf24l01_rx_buff[1]<<8) | Nrf24l01_rx_buff[2];
				cur_value = (Nrf24l01_rx_buff[3]<<8) | Nrf24l01_rx_buff[4];
				vol = vol_value*165 / 512;							//�Ŵ�100�� 3.3*100*4/4096 ��λ����ѹ
				cur = cur_value*275 / 1536;							//�Ŵ�10��  3.3*1000*10/4096/15 ��λ������
				pr = vol*cur/10;									//�Ŵ�100�� 
//------------------1.44 TFT--------------------------------------//
				if(System_key_type.new_key_type == KEY_PRESS_UP)
				{
#ifdef	LCD_144INCH
					GUI_ShowString(10,82,12,"VL:",0);					//12*6 size ��λ����ѹ
					GUI_ShowNum(28,82,vol/100,2,12);					// ����λ 2
					GUI_ShowString(40,82,12,".",0);
					GUI_ShowNum(46,82,(vol - (vol/100)*1000),2,12);	// С��λ 2
					GUI_ShowString(64,82,12,"V",0);
					
					GUI_ShowString(10,94,12,"CR:",0);					// ��λ������
					GUI_ShowNum(28,94,cur/10,2,12);					// ����λ 2
					GUI_ShowString(40,94,12,".",0);
					GUI_ShowNum(46,94,(cur - (cur/10)*100),1,12);		// С��λ 1
					GUI_ShowString(64,94,12,"A",0);

					GUI_ShowString(10,106,12,"PR:",0);					// ��λ������
					GUI_ShowNum(28,106,pr/100,2,12);					// ����λ 3
					GUI_ShowString(40,106,12,".",0);
					GUI_ShowNum(46,106,(pr - (pr/100)*1000),2,12);		// С��λ 2
					GUI_ShowString(64,106,12,"W",0);
#else
//------------------Nokia5110-------------------------------------//				
				//LCD_write_String(1, 	4, "Vol:");		 			//y:4
				//LCD_write_number(24,	3, vol/100, 1);    			//X:8*6,ȡ��ѹ����λ
				//LCD_write_String(30,	3, ".");			 			//x:48+6
				//LCD_write_number(36,	3, vol - (vol/100)*100 , 2);   //X:54+6,ȡ��ѹС������
				//LCD_write_String(72,	3, "V");			 			//x:60+12		
#endif
				}
			}
			Nrf24l01_rx_handle_flag=NOT;
		}
	}
}

//--------------------------------------------
//ϵͳ2.4G TX��ʱ		240ms
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
//ϵͳ2.4G RX���ݴ���ʱ		300ms
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
//ϵͳ״̬�����ʱ��   0.5min
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
//ϵͳ״̬���
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
//ϵͳ���϶�ʱʱ�� 200ms
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
//ϵͳ���ϴ�����
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
				LED_G_shadow(50);						//�̵���
				system_fault_counter_start = 1;			//��ʱ0.5���������
			}
			else if(fault_temp_falg == SYSTEM_VOLTAGE_FAULT)
			{
				LED_RGB_turnoff();
				buzzer_sound(SOUND_TURN_OFF);
				//LCD_BL_PWM_duty_set(0);
				LCD_fault_display(fault_temp_falg);
				LED_R_shadow(50);						//�����
				system_fault_counter_start = 1;			//��ʱ0.5���������
			}
		}
	}
}


