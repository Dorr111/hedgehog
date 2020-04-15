#ifndef __RANGING_TASK_H
#define __RANGING_TASK_H

#include "stm32f4xx_hal.h"

void Ranging_TIM2_ICtask(void);
void Ranging_TIM5_ICtask(void);
void Ranging_TIM2_PEtask(void);
void Ranging_TIM5_PEtask(void);
void Get_Ranging_Date1(void);
void Get_Ranging_Date2(void);

extern uint16_t Temp1;
extern uint16_t Temp2;








#endif
