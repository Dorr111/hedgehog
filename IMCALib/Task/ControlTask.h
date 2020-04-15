#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "mytype.h"

extern uint8_t PID_CALC;
extern uint8_t Data_Send;

bool_t ParamInit(void);
void AllTask(void);
void All_PidCalc(void);
void All_DateUpdate(void);

/*速度环PID参数重置*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd);
/*位置环PID参数重置*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd);

void LEDTest(int flag);

#endif
