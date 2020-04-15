#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "pid.h"

#define SKYGUARD_CHASSIS       //哨兵底盘控制

#define GUARD_CHASSIS_MAXSPEED_RPM    400    //哨兵底盘电机最大转速，单位：rpm
#define MAXSPEED_RPM    150    //哨兵底盘电机最大转速，单位：rpm
#define CHASSIS_MAXSPEED_RPM          300    //步兵底盘电机最大转速，单位：rpm
#define REDUCTION_RATIO_3508          19.2   //3508电机减速比

#define RANGING_MIN 25
#define RANGING_MAX 50


/*底盘电机序号*/
enum{
    
    FRON_RIGH_205 = 0, //前右
    FRON_LEFT_206 = 1, //前左
//    REAR_LEFT_203 = 2, //后左
//    REAR_RIGH_204 = 3, //后右
    
};


/*底盘结构体*/
typedef struct{
    
    int32_t  fr_motor_rpm_205; //前右电机
    int32_t  fl_motor_rpm_206; //前左电机
//    int32_t  rl_motor_rpm_203; //后左电机
//    int32_t  rr_motor_rpm_204; //后右电机
    

    float vx; //左右平移
    float vy; //前后
    float vw; //自转
    
    //车体坐标系中的速度
    float car_vx; //单位：m/s
    float car_vy;
    float car_vw; //rad/s
    
    //轮子对应车体坐标系的速度
    float wheel_rad_205; //轮子的转速，单位：rad/s
    float wheel_rad_206;
//    float wheel_rad_203;
//    float wheel_rad_204;
    
}chassis_t;



extern pid_t  Moto_Chassis_Pid_Pos[4];  //位置环PID结构体
extern pid_t  Moto_Chassis_Pid_Spd[4];  //速度环PID结构体
extern chassis_t  Chassis;


void ChassisDataUpdate(void);
void ChassisPidCalc(void);



#endif
