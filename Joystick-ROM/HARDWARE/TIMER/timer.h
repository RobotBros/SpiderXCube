#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

void TIM2_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);


#endif
