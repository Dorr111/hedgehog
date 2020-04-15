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
//��̨ɨ��ģʽö��
typedef enum
{
  GIMBAL_AUTO=0,
  GIMBAL_WAVE,
} gimbal_scan_e;
//��̨ģʽö��
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_Auto_aiming,    //��̨�������
  GIMBAL_Manaul_control, //��̨�Ƹ�ң�ز���
  GIMBAL_Wave_move,   //��̨����ģʽ 
} gimbal_behaviour_e;
//��̨������ݽṹ��
typedef struct 
{
	
	float angle_set;           //�����Ƕ�
	float actual_angle;        //ʵ�ʽǶ�
	
  uint16_t target_val;       //�Ӿ�Ŀ��ֵ
	
	float base_angle;         //��׼�Ƕ�
	
	float relative_angle;     //�������Ƕ�
	float relative_angle_set; //�������Ƕ�����
	float relative_offset_angle; //��������ʼ�Ƕ�
	float absolute_angle;     //�����ǽǶ�
	float absolute_angle_set; //�����ǽǶ�����
	float absolute_offset_angle;//�����ǳ�ʼ�Ƕ�
	
	float gimbal_can_send;     //��̨CAN����ֵ
	
	float cail_angle;       //�����Ƕ�
	
	int16_t Vision_Px_Error;    //�Ӿ�����ƫ��
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

