/*************************************************************************************************************************
 * @File   ChassisControl.c        
 * @Brief  �����˵��̿��ƣ��ӵ������Ͻǵ����ʱ�뿪ʼ�������̶�Ӧ�����IDΪ1 ~ 4�������ĸ�����Ŀ����ǹ���CAN1�ϣ�
 *         PID��Ӧ�Ĳ������ڴ������Ӹ��ص�����µ��Եõ���
 *
*************************************************************************************************************************/

#include "ChassisControl.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ranging_task.h"
#include "gimbal_task.h" 


pid_t  Moto_Chassis_Pid_Pos[4];  //���̵��λ�û�PID�ṹ��
pid_t  Moto_Chassis_Pid_Spd[4];  //���̵���ٶȻ�PID�ṹ��
chassis_t  Chassis;  /*���ڵ���*/
int speed=MAXSPEED_RPM;
uint8_t time=0;
uint8_t flag=0;

void ChassisDataUpdate(void)
{
    
    #if defined (SKYGUARD_CHASSIS)
    
    if(RC_UPPER_LEFT_SW_MID)
	{
        Chassis.vx = RcDeadlineLimit(remote_control.ch1,10)*GUARD_CHASSIS_MAXSPEED_RPM/660; //����ƽ��
    
        Chassis.fr_motor_rpm_205 = -Chassis.vx;
        Chassis.fl_motor_rpm_206 = -Chassis.vx;
	}
	if(RC_UPPER_LEFT_SW_UP)
	{
		
		if(Temp1>RANGING_MAX&&Temp2>RANGING_MAX&&time==1)
		{
			time=0;
		}
		if((Temp1<RANGING_MIN||Temp2<RANGING_MIN)&&time==0)
		{
//			speed=-speed;
//			time=1;
			if(speed<=MAXSPEED_RPM&&flag==0)
			{
				speed -= 3;
				if(speed==-MAXSPEED_RPM)
				{
					flag=1;
				}
			}
			else if(speed>=-MAXSPEED_RPM&&flag==1)
			{
				speed += 3;
				if(speed==MAXSPEED_RPM)
				{
					flag=0;
				}
			}
			
			if(speed==MAXSPEED_RPM||speed==-MAXSPEED_RPM)
				time=1;
			
		}
        Chassis.fr_motor_rpm_205 = speed;
        Chassis.fl_motor_rpm_206 = speed;		
	}
    
    #else
        
        Chassis.vx = RcDeadlineLimit(remote_control.ch1,10)*CHASSIS_MAXSPEED_RPM/660; //����ƽ��
        Chassis.vy = RcDeadlineLimit(remote_control.ch2,10)*CHASSIS_MAXSPEED_RPM/660; //ǰ���˶�
        Chassis.vw = RcDeadlineLimit(remote_control.ch3,10)*CHASSIS_MAXSPEED_RPM/660; //��ת
        
        Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
        Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
        
    #endif
    
}


void ChassisPidCalc(void)
{
    
    pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_205], Chassis_Motor[FRON_RIGH_205].speed_rpm, Chassis.fr_motor_rpm_205*REDUCTION_RATIO_3508);
    pid_calc(&Moto_Chassis_Pid_Spd[FRON_LEFT_206], Chassis_Motor[FRON_LEFT_206].speed_rpm, Chassis.fl_motor_rpm_206*REDUCTION_RATIO_3508);
//    pid_calc(&Moto_Chassis_Pid_Spd[REAR_LEFT_203], Chassis_Motor[REAR_LEFT_203].speed_rpm, Chassis.rl_motor_rpm_203*REDUCTION_RATIO_3508);
//    pid_calc(&Moto_Chassis_Pid_Spd[REAR_RIGH_204], Chassis_Motor[REAR_RIGH_204].speed_rpm, Chassis.rr_motor_rpm_204*REDUCTION_RATIO_3508);
    
}




