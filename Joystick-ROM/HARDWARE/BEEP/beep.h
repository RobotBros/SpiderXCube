#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h"

//beep端口定义

#define BEEP1 	PAout(9)
#define BEEP2	PAout(10)
#define BEEP		PBout(10)

#define 	BEEP_SCALE		3599		//Feq = 4KHz
#define	BEEP_DIV		4

#define 	BEEP_ON		1
#define 	BEEP_OFF		0

//------------蜂鸣器单个音符持续响时间-----
#define	BEEP_PWM_TIME		36			//蜂鸣器单个音符响持续时间
#define   BEEP_DISCHARGE	72			//单个音符拖尾响时间

//-----------蜂鸣器响声类型---------------
#define	SOUND_QUIET		0
#define	SOUND_TURN_ON	1
#define	SOUND_TURN_OFF	2
#define	SOUND_1SHORT		3
#define	SOUND_2SHORT		4
#define	SOUND_3SHORT		5
#define	SOUND_POWER_ON1	6
#define	SOUND_POWER_ON2	7


void BEEP_Init(u16 arr,u16 psc);	
void BEEP_PWM_duty_set(u8 beep1_duty);
void buzzer_power_on(void);
void buzzer_high_Hz(u16 period, u16 prescaler);
void buzzer_type_timer(void);
void buzzer_sound(u8 sound_type);

#endif

















