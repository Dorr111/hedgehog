#include "ControlTask.h"
#include "ChassisControl.h"
#include "shoot_task.h" 
#include "gimbal_task.h" 
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "Ano_Dt.h"
#include "arm_math.h"
#include "stdio.h"

uint8_t PID_CALC; //解算控制标志位
uint8_t Data_Send;//控制信号

bool_t ParamInit(void)
{
    memset(&Chassis, 0, sizeof(chassis_t));
//    memset(&remote_control, 0, sizeof(RC_Type));
    
	Motor_PwmControl(SNAIL_MAX_PULSE);
	
	/*三个拨弹轮电机位置环速度环初始化PID参数*/
	for(int j=0; j<3; j++)
    {
		PID_struct_init(&Pluck_Pid_Pos[j], POSITION_PID, 3628, 10, 10, 0.01f, 0.0f);//初始化位置环PID参数
		PID_struct_init(&Pluck_Pid_Spd[j], POSITION_PID, 6000, 1000, 4.0f, 0, 0); //初始化速度环PID参数
    }
	
    /*底盘两个电机速度环初始化PID参数*/
    for(int i=0; i<2; i++)
    {
        PID_struct_init(&Moto_Chassis_Pid_Spd[i], POSITION_PID, 15000, 500, 12.0f, 0.15f, 2.0f);
    }

	Gimbal_init();

	
	Motor_PwmControl(SNAIL_MIN_PULSE);
	
	return TRUE;
	
}



void AllTask(void)
{
    if(PID_CALC) //频率1kHz
    {
        PID_CALC = 0;
		All_PidCalc();
//		GimbalPidCalc();
//		SetChassisAndGimbalMotorValue(&hcan1, Moto_Chassis_Pid_Spd[FRON_RIGH_205].pos_out, Moto_Chassis_Pid_Spd[FRON_LEFT_206].pos_out, \
//                     Gimbal_Pid_Spd[0].pos_out, Gimbal_Pid_Spd[1].pos_out);

    }
    if(Data_Send) //频率500Hz
    {
        Data_Send = 0;
		All_DateUpdate();
//		Gimbal_Control();		
//        ANO_DT_DataUpdate(); /*发送数据到匿名地面站*/
        
//        printf("\n\r The sin and cos value:%f  , %f  \n\r",arm_sin_f32(PI/6), arm_cos_f32(PI/6));
        
    }
    
}

void All_PidCalc(void)
{
	PluckPidCalc(); 
	SetPluckMotorCurrent(&hcan1, Pluck_Pid_Spd[PLUCK_201].pos_out, Pluck_Pid_Spd[PLUCK_202].pos_out,Pluck_Pid_Spd[PLUCK_203].pos_out,0);
	ChassisPidCalc();
	GimbalPidCalc();
//	SetChassisAndGimbalMotorValue(&hcan1, Moto_Chassis_Pid_Spd[FRON_RIGH_205].pos_out, Moto_Chassis_Pid_Spd[FRON_LEFT_206].pos_out, \
//                     Gimbal_Pid_Spd[0].pos_out, Gimbal_Pid_Spd[1].pos_out);
}

void All_DateUpdate(void)
{
	Pluck_Control();
	Shoot_snailSpeed_Control();
	ChassisDataUpdate(); 	
	Gimbal_Control();
}


/*************************************************PID参数调试函数***********************************************************/
//在线调参时上位机发过来的参数是已经乘以1000的，这里要除以1000
/*速度环PID参数重置*/
void PidResetSpeed(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Gimbal_Pid_Spd[0].MaxOutput = maxout/1000.0f;
    Gimbal_Pid_Spd[0].IntegralLimit = intergral_limit/1000.0f;
    Gimbal_Pid_Spd[0].p = kp/1000.0f;
    Gimbal_Pid_Spd[0].i = ki/1000.0f;
    Gimbal_Pid_Spd[0].d = kd/1000.0f;
    
}

/*位置环PID参数重置*/
void PidResetPosition(uint32_t maxout, uint32_t intergral_limit, uint32_t kp, uint32_t ki, uint32_t kd)
{
    
    Gimbal_Pid_Pos[0].MaxOutput = maxout/1000.0f;
    Gimbal_Pid_Pos[0].IntegralLimit = intergral_limit/1000.0f;
    Gimbal_Pid_Pos[0].p = kp/1000.0f;
    Gimbal_Pid_Pos[0].i = ki/1000.0f;
    Gimbal_Pid_Pos[0].d = kd/1000.0f;
    
}

/********************************************************END**********************************************************/


