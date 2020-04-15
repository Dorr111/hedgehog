#include "gimbal_task.h"  
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "Vision_interact.h"
#include "main.h"
/*pid计算储存*/
pid_t Gimbal_Pid_Pos[2];
pid_t Gimbal_Pid_Spd[2];
/*自瞄PID参数*/
pid_t Gimbal_Pid_Vision_Pos[2];
pid_t Gimbal_Pid_Vision_Spd[2];

gimbal_control_t gimbal_control;
gimbal_scan_e    gimbal_scan;
gimbal_behaviour_e gimbal_behaviour=GIMBAL_ZERO_FORCE;

void Vision_printf(void);

int YAW_Gaimbal=0;
int PITCH_Gaimbal=1350;
int16_t YAW_ERR=0;
int16_t PITCH_ERR=0;
uint8_t PITCH_flag=0;
uint8_t YAW_flag=0;
uint8_t FLAG=0;
uint8_t wave_task;
uint8_t auto_task;

int set_speed=0;
int get_speed=0;
int get_angle=0;

float Vision_Pitch_Angle;
float Vision_Yaw_Angle;

float Vision_Pitch_SPD;

void Gimbal_limit(int angle)
{
	if(angle<MIN_ANGLE)
		angle=MIN_ANGLE;
	else if(angle>MAX_ANGLE)
		angle=MAX_ANGLE;
}
void Gimbal_init(void)
{
		/*YAW电机位置环速度环初始化PID参数*/
		PID_struct_init(&Gimbal_Pid_Pos[GIMBAL_YAW_207], POSITION_PID, 6000, 100, 8, 0.01f, 0.0f);//初始化位置环PID参数
		PID_struct_init(&Gimbal_Pid_Spd[GIMBAL_YAW_207], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f); //初始化速度环PID参数
	  /*PITCH电机位置环速度环初始化PID参数*/
		PID_struct_init(&Gimbal_Pid_Pos[GIMBAL_PITCH_208], POSITION_PID, 15, 60, 0.05f, 0.0f, 0.0f);//初始化位置环PID参数
		PID_struct_init(&Gimbal_Pid_Spd[GIMBAL_PITCH_208], POSITION_PID, 30000, 23000, 200.0f, 4.00f, 0.0f); //初始化速度环PID参数 
		
//		/*Yaw电机位置环速度环自瞄位置环初始化PID参数*/
		PID_struct_init(&Gimbal_Pid_Vision_Pos[GIMBAL_YAW_207], POSITION_PID, 6000, 100, 2.5f, 0.0f, 100.0f);//初始化位置环PID参数
		PID_struct_init(&Gimbal_Pid_Vision_Spd[GIMBAL_YAW_207], POSITION_PID, 15000, 500,  12.0f, 0.1f, 1.5f); //初始化速度环PID参数 
//	 /*PITCH电机位置环速度环自瞄位置环初始化PID参数*/
		PID_struct_init(&Gimbal_Pid_Vision_Pos[GIMBAL_PITCH_208], POSITION_PID, 15, 60, 0.1f, 0.0f, 0.0f);//初始化位置环PID参数
		PID_struct_init(&Gimbal_Pid_Vision_Spd[GIMBAL_PITCH_208], POSITION_PID, 30000, 23000, 260.0f, 9.00f, 0.0f); //初始化速度环PID参数 
	  
}
void Gimbal_Control(void)
{
	switch(remote_control.switch_left)
	{
		case Switch_Middle:
		{
			Gimbal_RC();
      gimbal_behaviour=GIMBAL_Manaul_control;
		}break;
		
		case Switch_Up:
		{
			Gimbal_Auto();
		}break;			
	}
}
void Gimbal_task(void)
{
	if(auto_task)
	{
		Gimbal_Auto();
	}
	else if(wave_task)
	{
		Gimbal_AUTO_WAVE();
	}
}

void Gimbal_AUTO_WAVE(void)
{
 if(YAW_flag==0)
 { 
  YAW_Gaimbal += 50;
  if((Gimbal_Motor[0].total_angle/19.2f)>=7000)
  {
   YAW_flag=1;
   YAW_Gaimbal=7000;
  }
 }
 else if(YAW_flag==1)
 { 
  YAW_Gaimbal -= 50;
  if((Gimbal_Motor[0].total_angle/19.2f)<=-7000)
  {
   YAW_flag=0;
   YAW_Gaimbal=-7000;
  }
 }
 if(PITCH_flag==0)
 {
  PITCH_Gaimbal +=100;
  if(Gimbal_Motor[1].angle>=1350)
  {
   PITCH_flag=1; 
   PITCH_Gaimbal=1450;
  }
    
 }
 else if(PITCH_flag==1)
 {
  PITCH_Gaimbal -=100;
  if(Gimbal_Motor[1].angle<=950)
  {
   PITCH_flag=0; 
   PITCH_Gaimbal=850;
  }     
 } 
  //PITCH轴限位	
	if(PITCH_Gaimbal<MIN_ANGLE)
		PITCH_Gaimbal=MIN_ANGLE;
	else if(PITCH_Gaimbal>MAX_ANGLE)
		PITCH_Gaimbal=MAX_ANGLE;	
}

void Gimbal_RC(void)
{
		  YAW_Gaimbal +=RcDeadlineLimit(remote_control.ch3,GIMBAL_DEAD)*10/660;
			PITCH_Gaimbal +=RcDeadlineLimit(remote_control.ch4,GIMBAL_DEAD)*3/660;
	
	    //PITCH轴限位	
		if(PITCH_Gaimbal<MIN_ANGLE)
			PITCH_Gaimbal=MIN_ANGLE;
		else if(PITCH_Gaimbal>MAX_ANGLE)
			PITCH_Gaimbal=MAX_ANGLE;
}

void Gimbal_Auto(void)
{
	static float yaw_angle_err,pitch_angle_err;//YAW,pitch轴视觉角度偏差
	static uint32_t lose_times=0;
		//设置视觉目标值
	gimbal_control.gimbal_pitch_motor.target_val =0;
  gimbal_control.gimbal_yaw_motor.target_val =0;
	//自瞄目标相对角度值更新
	Vision_Yaw_Error(&(gimbal_control.gimbal_yaw_motor.Vision_Data));
	Vision_Pitch_Error(&(gimbal_control.gimbal_pitch_motor.Vision_Data));
	if(Vision_UpDate()==TRUE)
	{
		Vision_UpDate_Clean();//清零视觉数据更新标志位
		if(VisonRecvData.model==1)//如果识别到装甲板，就更新数据
		{
			     ;	
			 if(gimbal_scan==GIMBAL_AUTO)
			 {
				 gimbal_behaviour=GIMBAL_Auto_aiming;
				 //wave_task=0;
			     //lose_times=0;
				 pitch_angle_err=gimbal_control.gimbal_pitch_motor.Vision_Data;
				 yaw_angle_err=gimbal_control.gimbal_yaw_motor.Vision_Data;
					
				 gimbal_control.gimbal_pitch_motor.Vision_Angle_Error=pitch_angle_err;
				 gimbal_control.gimbal_yaw_motor.Vision_Angle_Error=-yaw_angle_err;
			 }
				
				 
		}
	}	
		else if(VisonRecvData.model==0)
		{
			
            if(gimbal_scan==GIMBAL_WAVE)
				
				YAW_Gaimbal=Gimbal_Motor[0].total_angle/19.2f;
				PITCH_Gaimbal=Gimbal_Motor[1].angle;
			
			    Gimbal_AUTO_WAVE();
    		   gimbal_behaviour=GIMBAL_Wave_move;
			}
} 
void GimbalPidCalc(void)
{	
	if(gimbal_behaviour==GIMBAL_Auto_aiming)
	{	
		FLAG=1;
		pid_calc(&Gimbal_Pid_Vision_Pos[GIMBAL_YAW_207], gimbal_control.gimbal_yaw_motor.Vision_Angle_Error, gimbal_control.gimbal_yaw_motor.target_val);
		pid_calc(&Gimbal_Pid_Vision_Pos[GIMBAL_PITCH_208], gimbal_control.gimbal_pitch_motor.Vision_Angle_Error, gimbal_control.gimbal_pitch_motor.target_val);
		pid_calc(&Gimbal_Pid_Vision_Spd[GIMBAL_YAW_207], Gimbal_Motor[GIMBAL_YAW_207].speed_rpm, Gimbal_Pid_Vision_Pos[GIMBAL_YAW_207].pos_out);
		pid_calc(&Gimbal_Pid_Vision_Spd[GIMBAL_PITCH_208], Gimbal_Motor[GIMBAL_PITCH_208].speed_rpm, Gimbal_Pid_Vision_Pos[GIMBAL_PITCH_208].pos_out);
//		if(RC_UPPER_RIGHT_SW_DOWN)
//		{
//		  SetChassisAndGimbalMotorValue(&hcan1, 0, 0,  Gimbal_Pid_Vision_Spd[GIMBAL_YAW_207].pos_out,  Gimbal_Pid_Vision_Spd[GIMBAL_PITCH_208].pos_out);
//    }
//		else
//		{
//			SetChassisAndGimbalMotorValue(&hcan1, 0, 0, 0,  0);
//		}
	}
	else if(gimbal_behaviour==GIMBAL_Wave_move||gimbal_behaviour==GIMBAL_Manaul_control)
	{
	  FLAG=0;
		pid_calc(&Gimbal_Pid_Pos[0], Gimbal_Motor[0].total_angle/19.2f,YAW_Gaimbal);
		pid_calc(&Gimbal_Pid_Pos[1], Gimbal_Motor[1].angle, PITCH_Gaimbal);
		pid_calc(&Gimbal_Pid_Spd[0], Gimbal_Motor[0].speed_rpm, Gimbal_Pid_Pos[0].pos_out);
		pid_calc(&Gimbal_Pid_Spd[1], Gimbal_Motor[1].speed_rpm, Gimbal_Pid_Pos[1].pos_out);
//		if(RC_UPPER_RIGHT_SW_DOWN)
//		{
//		 SetChassisAndGimbalMotorValue(&hcan1, 0, 0, Gimbal_Pid_Spd[0].pos_out,  Gimbal_Pid_Spd[1].pos_out);
//    }
//		else
//		{
//			SetChassisAndGimbalMotorValue(&hcan1, 0, 0, 0,  0);
//		}
	}
	
	
	set_speed=Gimbal_Pid_Pos[1].pos_out;
	
	get_speed=Gimbal_Motor[1].total_angle;
	
	get_angle=Gimbal_Motor[0].total_angle;

}
void Ecod_printf(void)
{
	
}
void Vision_printf(void)
{
	 Vision_Pitch_Angle=gimbal_control.gimbal_pitch_motor.Vision_Data;
     Vision_Yaw_Angle=gimbal_control.gimbal_yaw_motor.Vision_Data;
	 
	 Vision_Pitch_SPD=Gimbal_Pid_Vision_Spd[GIMBAL_PITCH_208].pos_out;
	
}
