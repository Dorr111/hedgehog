#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "pid.h"

#define SKYGUARD_CHASSIS       //�ڱ����̿���

#define GUARD_CHASSIS_MAXSPEED_RPM    400    //�ڱ����̵�����ת�٣���λ��rpm
#define MAXSPEED_RPM    150    //�ڱ����̵�����ת�٣���λ��rpm
#define CHASSIS_MAXSPEED_RPM          300    //�������̵�����ת�٣���λ��rpm
#define REDUCTION_RATIO_3508          19.2   //3508������ٱ�

#define RANGING_MIN 25
#define RANGING_MAX 50


/*���̵�����*/
enum{
    
    FRON_RIGH_205 = 0, //ǰ��
    FRON_LEFT_206 = 1, //ǰ��
//    REAR_LEFT_203 = 2, //����
//    REAR_RIGH_204 = 3, //����
    
};


/*���̽ṹ��*/
typedef struct{
    
    int32_t  fr_motor_rpm_205; //ǰ�ҵ��
    int32_t  fl_motor_rpm_206; //ǰ����
//    int32_t  rl_motor_rpm_203; //������
//    int32_t  rr_motor_rpm_204; //���ҵ��
    

    float vx; //����ƽ��
    float vy; //ǰ��
    float vw; //��ת
    
    //��������ϵ�е��ٶ�
    float car_vx; //��λ��m/s
    float car_vy;
    float car_vw; //rad/s
    
    //���Ӷ�Ӧ��������ϵ���ٶ�
    float wheel_rad_205; //���ӵ�ת�٣���λ��rad/s
    float wheel_rad_206;
//    float wheel_rad_203;
//    float wheel_rad_204;
    
}chassis_t;



extern pid_t  Moto_Chassis_Pid_Pos[4];  //λ�û�PID�ṹ��
extern pid_t  Moto_Chassis_Pid_Spd[4];  //�ٶȻ�PID�ṹ��
extern chassis_t  Chassis;


void ChassisDataUpdate(void);
void ChassisPidCalc(void);



#endif
