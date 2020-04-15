#ifndef MPU_RX_H
#define MPU_RX_H

#include "mytype.h"

#define MPU_RC_BUFFER_SIZE  11

struct SGyro
{
	short w[3];
	short T;
};
struct SAngle
{
	short Angle[3];
	short T;
};

extern float yaw_angel;
extern float yaw_speed;

extern struct SGyro 		stcGyro;
extern struct SAngle 	stcAngle;

extern u16 MPU_USART_RX_STA;
extern uint8_t Mpu_Rx_Buffer[MPU_RC_BUFFER_SIZE] ;

//陀螺仪数据处理
void mpu_rxdata_deal(uint8_t *ucRxBuffer);

#endif
