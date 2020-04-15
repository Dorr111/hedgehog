/**********************************************************************************************************************
 * @file   Top_Gimbal.c
 * @brief  哨兵上云台的控制  两个6020 
 *
**********************************************************************************************************************/

#include "Top_Gimbal.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "main.h"
#include "pid.h"

pid_t Gimbal_Pid_Pos[2];
pid_t Gimbal_Pid_Spd[2];

int YAW_Gaimbal=0;
int PITCH_Gaimbal=1350;

int16_t YAW_ERR=0;
int16_t PITCH_ERR=0;
uint8_t PITCH_flag=0;
uint8_t YAW_flag=0;
uint8_t FLAG=0;
uint8_t wave_task;
uint8_t auto_task;


void Gimbal_init(void)  //上云台初始化PID参数
{
		/*YAW电机位置环速度环初始化PID参数*/
		PID_struct_init(&Gimbal_Pid_Pos[GIMBAL_YAW_207], POSITION_PID, 6000, 100, 8, 0.01f, 0.0f);//初始化位置环PID参数
		PID_struct_init(&Gimbal_Pid_Spd[GIMBAL_YAW_207], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f); //初始化速度环PID参数
	  /*PITCH电机位置环速度环初始化PID参数*/
		PID_struct_init(&Gimbal_Pid_Pos[GIMBAL_PITCH_208], POSITION_PID, 15, 60, 0.05f, 0.0f, 0.0f);//初始化位置环PID参数
		PID_struct_init(&Gimbal_Pid_Spd[GIMBAL_PITCH_208], POSITION_PID, 30000, 23000, 200.0f, 4.00f, 0.0f); //初始化速度环PID参数 
		
}

void Gimbal_RC(void)   //更新接收遥控器的数据
{
		  YAW_Gaimbal +=RcDeadlineLimit(remote_control.ch3,GIMBAL_DEAD)*10/660;
			PITCH_Gaimbal +=RcDeadlineLimit(remote_control.ch4,GIMBAL_DEAD)*3/660;
	
	    //PITCH轴限位	
		if(PITCH_Gaimbal<MIN_ANGLE)
			PITCH_Gaimbal=MIN_ANGLE;
		else if(PITCH_Gaimbal>MAX_ANGLE)
			PITCH_Gaimbal=MAX_ANGLE;
		
}


void GimbalPidCalc(void)
{
	pid_calc(&Gimbal_Pid_Pos[GIMBAL_YAW_207], Gimbal_Motor[GIMBAL_YAW_207].total_angle/36, YAW_Gaimbal);
	pid_calc(&Gimbal_Pid_Pos[GIMBAL_YAW_207], Gimbal_Motor[GIMBAL_YAW_207].speed_rpm, Gimbal_Pid_Pos[GIMBAL_YAW_207].pos_out);
	
	pid_calc(&Gimbal_Pid_Pos[GIMBAL_PITCH_208], Gimbal_Motor[GIMBAL_PITCH_208].total_angle/36, PITCH_Gaimbal);
	pid_calc(&Gimbal_Pid_Pos[GIMBAL_PITCH_208], Gimbal_Motor[GIMBAL_PITCH_208].speed_rpm, Gimbal_Pid_Pos[GIMBAL_PITCH_208].pos_out);

}


void Gimbal_Control(void)
{
	  SetPluckMotorCurrent(&hcan1, Gimbal_Pid_Pos[GIMBAL_YAW_207].pos_out, Gimbal_Pid_Pos[GIMBAL_PITCH_208].pos_out, 0, 0);   

}

