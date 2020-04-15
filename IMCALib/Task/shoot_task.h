#ifndef __SHOOT_
#define __SHOOT_

#include "pid.h"
#include "CanBus_Task.h"

#define SNAIL_MAX_PULSE 2000
#define SNAIL_MIN_PULSE 900

enum{
    
    PLUCK_201 = 0, 
    PLUCK_202 = 1, 
    PLUCK_203 = 2,      
};

extern pid_t Pluck_Pid_Pos[3];
extern pid_t Pluck_Pid_Spd[3];
extern pid_t pid_spd[2];

void PluckTask(void);
void Shoot_init(void);
void Pluck_Control(void);
void PluckPidCalc(void);
void Friction_3508Speed_Control(void);
void Shoot_Control(void);


#endif
