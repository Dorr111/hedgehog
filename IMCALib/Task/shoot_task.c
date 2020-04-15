/**********************************************************************************************************************
 * �ļ�     ��shoot_task.c
 * ���     ��3508ID�ֱ���202��203��2006IDΪ201
				���û����ֲ����ϵͳ��ȡ��ע��Pluck_Control�Ĳ��ִ���
**********************************************************************************************************************/



#include "shoot_task.h" 
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "tim.h"
#include "Judge_interact.h"
#include "pid.h"


/*���ò���*/
float set_pluck1_angle=0;			//������ͨ���������Ļ�е�Ƕ�
float set_pluck2_angle=0;			
float set_pluck3_angle=0;	

int expect_speed=0;				//Ԥ�����ʵ�ٶ�
extern TIM_HandleTypeDef htim4; //snail������ƶˣ���ʱ��4ͨ��1��Prescaler/Period=83/19999


/*pid���㴢��*/
pid_t Pluck_Pid_Pos[3];
pid_t Pluck_Pid_Spd[3];
pid_t pid_spd[2];        //Ħ����


/*��ȡ����ϵͳʵʱ����*/
ext_power_heat_data_t Heat;
ext_game_robot_state_t Shoot;
int BufferTime = 0;//�ڶ�ʱ������BufferTime++�����ʱ�����ڿ�����Ƶ




void PluckTask(void)
{
//    Shoot_snailSpeed_Control();
           
}

/*-----------------------------------------------------------------------
\@brief 2006��������
\@param param����
\@return ��
\@reference ң���Ҳ�������
			��ʼλ���м�
			���ϳ�������
*--------------------------------------------------------------------*/
int key=0,KEY=0;
void Pluck_Control(void)
{
//	if(remote_control.switch_right == Switch_Middle)
//	{
//		set_pluck_angle = moto_pluck[0].total_angle/36+0;//ȷ��һ�������ж���ĵ����
//	}
//	else if(remote_control.switch_right == Switch_Up)
//	{
//		FireHeat = FireHeat17();//��ȡʵ������
//		CoolingRate = FireCoolingRate();//��ȡÿ����ȴ
//		if(FireHeat<(240-expect_speed*3)&&BufferTime>50)//expect_speed��3508������д����Ԥ�����٣����Ʋ�������
//		{
//			set_pluck_angle += 910;//682-910Ϊ12-9λת�̲�һ�ŵ��Ļ�е��
//			BufferTime = 0;
//		}
//		else if(FireHeat<(240-expect_speed)&&BufferTime>expect_speed*1000/CoolingRate)//ȷ����û��������ǰ�����ٷ���
//		{
//			set_pluck_angle = moto_pluck[0].total_angle/36+910;//�ڵ�ǰλ�����ٲ�����������ֱ�Ӳ�����ֱ�Ӳ���λ�û��ۼӴӶ�������
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

void Shoot_init(void)  //PID������ʼ��
{
	 PID_struct_init(&Pluck_Pid_Spd[PLUCK_201], POSITION_PID, 6000, 100, 8, 0.01f, 0.0f);//��ʼ��λ�û�PID����
	 PID_struct_init(&Pluck_Pid_Spd[PLUCK_201], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f); //��ʼ���ٶȻ�PID����
	
	 PID_struct_init(&pid_spd[0], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f);  //Ħ����
	 PID_struct_init(&pid_spd[1], POSITION_PID, 15000, 500, 12.0f, 0.1f, 1.5f);
	
}


void PluckPidCalc(void)
{
		pid_calc(&pid_spd[PLUCK_201], Pluck_Motor[PLUCK_201].total_angle/36, set_pluck1_angle);
		pid_calc(&pid_spd[PLUCK_201], Pluck_Motor[PLUCK_201].speed_rpm, Pluck_Pid_Pos[PLUCK_201].pos_out);

}

/*-----------------------------------------------------------------------
\@brief 3508�����Ħ���� 
\@param param����
\@return ��
\@reference ң������м�ֹͣ����ת��
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
\@brief PWMռ�ձ�����
\@param param����
\@return ��
\@reference ��Ҫ�������õĶ�ʱ����ͨ���ı䣬��ͨ��
*--------------------------------------------------------------------*/
//void Motor_PwmControl(uint16_t value)				//PA5,СĦ����
//{
//	TIM_OC_InitTypeDef sConfigOC;                 //����Ƚ�ģʽ�ṹ��
//	sConfigOC.OCMode = TIM_OCMODE_PWM1;           //ģʽѡ��
//	sConfigOC.Pulse = value;                      //���õ�ƽ����ֵ   
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;   //��������Ƚϼ���
//	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;    //����ȽϿ���ʹ�ܺ�ʧ��
//	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);   //����PWMͨ����ֵ
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);                       //ʹ��
//}

