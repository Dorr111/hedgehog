/**********************************************************************************************************************
 * 文件     ：shoot_task.c
 * 简介     ：3508ID分别是202和203，2006ID为201
				如果没有移植裁判系统读取需注释Pluck_Control的部分代码
**********************************************************************************************************************/



#include "shoot_task.h" 
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "tim.h"
#include "Judge_interact.h"
#include "pid.h"


/*常用参数*/
float set_pluck1_angle=0;			//拨弹轮通过减速箱后的机械角度
float set_pluck2_angle=0;			
float set_pluck3_angle=0;	

int expect_speed=0;				//预测的真实速度
extern TIM_HandleTypeDef htim4; //snail电机控制端，定时器4通道1，Prescaler/Period=83/19999


/*pid计算储存*/
pid_t Pluck_Pid_Pos[3];
pid_t Pluck_Pid_Spd[3];
pid_t pid_spd[2];        //摩擦轮


/*获取裁判系统实时数据*/
ext_power_heat_data_t Heat;
ext_game_robot_state_t Shoot;
int BufferTime = 0;//在定时器里有BufferTime++，间隔时间用于控制射频




void PluckTask(void)
{
//    Shoot_snailSpeed_Control();
           
}

/*-----------------------------------------------------------------------
\@brief 2006拨弹控制
\@param param：无
\@return 无
\@reference 遥控右拨键控制
			初始位在中间
			拨上持续发弹
*--------------------------------------------------------------------*/
int key=0,KEY=0;
void Pluck_Control(void)
{
//	if(remote_control.switch_right == Switch_Middle)
//	{
//		set_pluck_angle = moto_pluck[0].total_angle/36+0;//确保一定不会有多余的弹射出
//	}
//	else if(remote_control.switch_right == Switch_Up)
//	{
//		FireHeat = FireHeat17();//读取实际热量
//		CoolingRate = FireCoolingRate();//获取每秒冷却
//		if(FireHeat<(240-expect_speed*3)&&BufferTime>50)//expect_speed在3508控制中写入了预期射速，控制不超热量
//		{
//			set_pluck_angle += 910;//682-910为12-9位转盘拨一颗弹的机械角
//			BufferTime = 0;
//		}
//		else if(FireHeat<(240-expect_speed)&&BufferTime>expect_speed*1000/CoolingRate)//确保在没超热量的前提下再发弹
//		{
//			set_pluck_angle = moto_pluck[0].total_angle/36+910;//在当前位置上再拨弹，而不是直接拨弹，直接拨弹位置会累加从而超热量
//			BufferTime = 0;
//		}

//	}
	
//	switch(remote_control.switch_left)
//	{
//		case Switch_Up:
//		{
			if(RC_UPPER_RIGHT_SW_MID&&key==0)
			{
				set_pluck1_angle += 0;
				key=1;
			}
			else if(RC_UPPER_RIGHT_SW_UP&&key==1)
			{
				set_pluck1_angle += 1365.33f;
				key=0;
			}
			else if(RC_UPPER_RIGHT_SW_DOWN&&key==1)
			{
				set_pluck1_angle += 8192;
				key=0;
			}
//		}break;
//		case Switch_Middle:
//		{
//			if(remote_control.switch_right == Switch_Middle&&key==0)
//			{
//				set_pluck2_angle += 0;
//				key=1;
//			}
//			else if(remote_control.switch_right == Switch_Up&&key==1)
//			{
//				set_pluck2_angle += 1365.33f;
//				key=0;
//			}
//			else if(remote_control.switch_right == Switch_Down&&key==1)
//			{
//				set_pluck2_angle += 8192;
//				key=0;
//			}
//		}break;		
//		case Switch_Down:
//		{
//			if(remote_control.switch_right == Switch_Middle&&key==0)
//			{
//				set_pluck3_angle += 0;
//				key=1;
//			}
//			else if(remote_control.switch_right == Switch_Up&&key==1)
//			{
//				set_pluck3_angle += 1365.33f;
//				key=0;
//			}
//			else if(remote_control.switch_right == Switch_Down&&key==1)
//			{
//				set_pluck3_angle += 8192;
//				key=0;
//			}
//		}break;		
//	}		
   
}

void Shoot_init(void)  //PID参数初始化
{
	 PID_struct_init(&Pluck_Pid_Spd[PLUCK_201], POSITION_PID, 6000, 100, 8, 0.01f, 0.0f);//初始化位置环PID参数
	 PID_struct_init(&Pluck_Pid_Spd[PLUCK_201], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f); //初始化速度环PID参数
	
	 PID_struct_init(&pid_spd[0], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f);  //摩擦轮
	 PID_struct_init(&pid_spd[1], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f);
	
}


void PluckPidCalc(void)
{
		pid_calc(&pid_spd[PLUCK_201], Pluck_Motor[PLUCK_201].total_angle/36, set_pluck1_angle);
		pid_calc(&pid_spd[PLUCK_201], Pluck_Motor[PLUCK_201].speed_rpm, Pluck_Pid_Pos[PLUCK_201].pos_out);

}

/*-----------------------------------------------------------------------
\@brief 3508电机作摩擦轮 
\@param param：无
\@return 无
\@reference 遥控左键中间停止向上转动
*--------------------------------------------------------------------*/
uint16_t set_speed;

void Friction_3508Speed_Control(void)
{
    if(remote_control.switch_left == Switch_Middle)
    {
        set_speed = 0;
        
    }
    else if(remote_control.switch_left == Switch_Up)
    {
        set_speed = 1000;       
    }
    pid_calc(&pid_spd[0], Friction_Motor[0].speed_rpm, set_speed);
    pid_calc(&pid_spd[1], Friction_Motor[1].speed_rpm, set_speed);
    SetPluckMotorCurrent(&hcan1, pid_spd[0].pos_out, pid_spd[1].pos_out, 0, 0);   
		HAL_Delay(1);

}

void Shoot_Control(void)
{
	  Friction_3508Speed_Control();
	  Pluck_Control();
	
}


/*-----------------------------------------------------------------------
\@brief PWM占空比设置
\@param param：无
\@return 无
\@reference 需要根据所用的定时器和通道改变，不通用
*--------------------------------------------------------------------*/
//void Motor_PwmControl(uint16_t value)				//PA5,小摩擦轮
//{
//	TIM_OC_InitTypeDef sConfigOC;                 //输出比较模式结构体
//	sConfigOC.OCMode = TIM_OCMODE_PWM1;           //模式选择
//	sConfigOC.Pulse = value;                      //设置电平跳变值   
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;   //设置输出比较级性
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;    //输出比较快速使能和失能
//	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);   //设置PWM通道的值
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);                       //使能
//}

