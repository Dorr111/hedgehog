#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include "pid.h"

#define MAX_ANGLE 1450
#define MIN_ANGLE 850
#define GIMBAL_DEAD 3

enum{
    
    GIMBAL_YAW_207 = 0, 
    GIMBAL_PITCH_208 = 1, 
    
};
//云台扫描模式枚举
typedef enum
{
  GIMBAL_AUTO=0,
  GIMBAL_WAVE,
} gimbal_scan_e;
//云台模式枚举
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //云台无力
  GIMBAL_Auto_aiming,    //云台自瞄测试
  GIMBAL_Manaul_control, //云台推杆遥控测试
  GIMBAL_Wave_move,   //云台调参模式 
} gimbal_behaviour_e;
//云台电机数据结构体
typedef struct 
{
	
	float angle_set;           //给定角度
	float actual_angle;        //实际角度
	
  uint16_t target_val;       //视觉目标值
	
	float base_angle;         //基准角度
	
	float relative_angle;     //编码器角度
	float relative_angle_set; //编码器角度设置
	float relative_offset_angle; //编码器初始角度
	float absolute_angle;     //陀螺仪角度
	float absolute_angle_set; //陀螺仪角度设置
	float absolute_offset_angle;//陀螺仪初始角度
	
	float gimbal_can_send;     //云台CAN发送值
	
	float cail_angle;       //修正角度
	
	int16_t Vision_Px_Error;    //视觉像素偏差
	float   Vision_Angle_Error;
	float   Vision_Data;

}gimbal_motor_t;

typedef struct
{
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
} gimbal_control_t;

void Gimbal_init(void);

void Gimbal_RC(void);
void Gimbal_Auto(void);
void Gimbal_AUTO_WAVE(void);
void Wave_Auto(void);

void Gimbal_limit(int angle);
void Gimbal_Control(void);
void GimbalPidCalc(void);

extern gimbal_control_t gimbal_control;
extern gimbal_scan_e    gimbal_scan;
extern gimbal_behaviour_e gimbal_behaviour;
extern uint8_t wave_task;

extern uint8_t auto_task;

extern pid_t Gimbal_Pid_Pos[2];
extern pid_t Gimbal_Pid_Spd[2];

extern pid_t Gimbal_Pid_Vision_Pos[2];
extern pid_t Gimbal_Pid_Vision_Spd[2];




#endif

