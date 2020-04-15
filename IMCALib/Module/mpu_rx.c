/******************************************************************************
*功 能：串口DMA中断方式读取陀螺仪模块
*输 出：模块解算后数据（JY901/HWT101）后者漂移更低精度更高

*说 明：使用需移植mpu_rx.c和mpu_rx.h两文件，
				需在主函数使能中断，再在stm32f4xx_it.h的串口6中断内添加指定代码
				包含头文件后直接调用陀螺仪数据即可
*******************************************************************************/ 

#include "mpu_rx.h"

struct SGyro 		stcGyro;
struct SAngle 	stcAngle;

float yaw_angel;
float yaw_speed;

u16 MPU_USART_RX_STA=0;      								 //接收状态标记	
uint8_t Mpu_Rx_Buffer[MPU_RC_BUFFER_SIZE] ={0};	//接收缓冲数组


void mpu_rxdata_deal(uint8_t *ucRxBuffer)
{
	//printf("串口接收中断：%d \r\n",aRxBuffer2[0]);

		if (ucRxBuffer[0]!=0x55) 									//数据头不对，则重新开始寻找0x55数据头
		{
			return;
		}
		else
		{
			switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
			{
				//从数组的第三位开始拷贝,若直接在switch函数中解算解算，会占用时间
				case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8); /*yaw_speed = (float)stcGyro.w[2]/32768*2000;    */break;						//接收陀螺仪角速度数据
				case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);/*yaw_angel = (float)stcAngle.Angle[2]/32768*180;*/break;						//接收陀螺仪角度数据

			}
		}
}


