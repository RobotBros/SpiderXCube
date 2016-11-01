#include "key.h"
#include "delay.h"

//static u8 key_type = KEY_PRESS_NULL;
_system_key_type System_key_type;

static u8 key_handle_flag_60ms=0;
								    
//------------------------------------------
//按键初始化
//------------------------------------------
void Key_init(void) 
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);			//关闭JTAG，留SWD

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 							//上拉
 	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	System_ini_state.key_ini_state =DONE;
	System_key_type.new_key_type= KEY_PRESS_NULL;
	System_key_type.old_key_type= KEY_PRESS_NULL;
	System_key_type.key_handle_flag = 0;
}

//------------------------------------------------
//按键检测 
//------------------------------------------------
void Key_detect(void)
{
	static u8 tick_4ms = 0;
	
	if(System_key_type.key_handle_flag == 0)
	{
		if(KEY_LEFT == 0)
		{
			tick_4ms++;
			if (tick_4ms >= 12)			// 48ms
			{
				if(KEY_LEFT == 0)
				{while(KEY_LEFT == 0);buzzer_sound(SOUND_1SHORT);System_key_type.new_key_type= KEY_PRESS_LEFT;tick_4ms = 0;System_key_type.key_handle_flag =1;return;}
				else {tick_4ms = 0;}
			}
		}

		if(KEY_UP == 0)
		{
			tick_4ms++;
			if (tick_4ms >= 12)			
			{
				if(KEY_UP == 0)
				{while(KEY_UP == 0);buzzer_sound(SOUND_1SHORT);System_key_type.new_key_type= KEY_PRESS_UP;tick_4ms = 0;System_key_type.key_handle_flag =1;return;}
				else {tick_4ms = 0;}
			}
		}

		if(KEY_RIGHT == 0)
		{
			tick_4ms++;
			if (tick_4ms >= 12)			
			{
				if(KEY_RIGHT == 0)
				{while(KEY_RIGHT == 0);buzzer_sound(SOUND_1SHORT);System_key_type.new_key_type= KEY_PRESS_RIGHT;tick_4ms = 0;System_key_type.key_handle_flag =1;return;}
				else {tick_4ms = 0;}
			}
		}

		if(KEY_DOWN == 0)
		{
			tick_4ms++;
			if (tick_4ms >= 12)			
			{
				if(KEY_DOWN == 0) 
				{while(KEY_DOWN == 0);buzzer_sound(SOUND_1SHORT);System_key_type.new_key_type= KEY_PRESS_DOWN;tick_4ms = 0;System_key_type.key_handle_flag =1;return;}
				else {tick_4ms = 0;}
			}
		}

		if(KEY_ENTER == 0)
		{
			tick_4ms++;
			if (tick_4ms >= 12)			
			{
				if(KEY_ENTER == 0) 
				{while(KEY_ENTER == 0);buzzer_sound(SOUND_1SHORT);System_key_type.new_key_type= KEY_PRESS_ENTER;tick_4ms = 0;System_key_type.key_handle_flag =1;return;}
				else {tick_4ms = 0;}
			}
		}
	}
}

//------------------------------------------------
//按键操作
//------------------------------------------------
void Key_action(void)
{	 
	if(key_handle_flag_60ms)
	{
		key_handle_flag_60ms =0;
		
		if(System_key_type.key_handle_flag == 1)
		{
			//System_key_type.key_handle_flag = 0;
		
			switch(System_key_type.new_key_type)
			{
				case KEY_PRESS_NULL:
					//key action
					if(System_key_type.old_key_type != System_key_type.new_key_type){Back_color =BLACK;Lcd_Clear(Back_color, 0, 70);System_key_type.old_key_type = System_key_type.new_key_type;}
					System_key_type.key_handle_flag = 0;
					break;
				case KEY_PRESS_LEFT:
					//key action
					if(System_key_type.old_key_type != System_key_type.new_key_type){Back_color =BLACK;Lcd_Clear(Back_color, 0, 70);System_key_type.old_key_type = System_key_type.new_key_type;}
					System_key_type.key_handle_flag = 0;
					break;
				case KEY_PRESS_UP:
					//key action
					if(System_key_type.old_key_type != System_key_type.new_key_type){Back_color =BLACK;Lcd_Clear(Back_color, 0, 70);System_key_type.old_key_type = System_key_type.new_key_type;}
					System_key_type.key_handle_flag = 0;
					break;
				case KEY_PRESS_RIGHT:
					//key action
					if(System_key_type.old_key_type != System_key_type.new_key_type){Back_color =BLACK;Lcd_Clear(Back_color, 0, 70);System_key_type.old_key_type = System_key_type.new_key_type;}
					System_key_type.key_handle_flag = 0;
					break;
				case KEY_PRESS_DOWN:
					//key action
					if(System_key_type.old_key_type != System_key_type.new_key_type){Back_color =BLACK;Lcd_Clear(Back_color, 0, 70);System_key_type.old_key_type = System_key_type.new_key_type;}
					System_key_type.key_handle_flag = 0;
					break;
				case KEY_PRESS_ENTER:
					//key action
					if(System_key_type.old_key_type != System_key_type.new_key_type){Back_color =BLACK;Lcd_Clear(Back_color, 0, 70);System_key_type.old_key_type = System_key_type.new_key_type;}
					System_key_type.key_handle_flag = 0;
					break;
				default: break;
			}
		}
	}
}

//--------------------------------------
//系统KEY处理定时器 60ms
//--------------------------------------
void Key_handle_timer(void)
{
	static u8 tick1_4ms = 0;

	tick1_4ms++;
	
	if (tick1_4ms >= 15)			// 4*15 = 60ms
	{	
		tick1_4ms=0;
		key_handle_flag_60ms = 1;
	}
}

