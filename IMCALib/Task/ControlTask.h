#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include "mytype.h"

extern uint8_t PID_CALC;
extern uint8_t Data_Send;

bool_t ParamInit(void);
void AllTask(void);
void All_PidCalc(void);
void All_DateUpdate(void);

/*�ٶȻ�PID��������*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd);
/*λ�û�PID��������*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd);

void LEDTest(int flag);

#endif
