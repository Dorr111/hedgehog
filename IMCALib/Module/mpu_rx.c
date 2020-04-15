/******************************************************************************
*�� �ܣ�����DMA�жϷ�ʽ��ȡ������ģ��
*�� ����ģ���������ݣ�JY901/HWT101������Ư�Ƹ��;��ȸ���

*˵ ����ʹ������ֲmpu_rx.c��mpu_rx.h���ļ���
				����������ʹ���жϣ�����stm32f4xx_it.h�Ĵ���6�ж������ָ������
				����ͷ�ļ���ֱ�ӵ������������ݼ���
*******************************************************************************/ 

#include "mpu_rx.h"

struct SGyro 		stcGyro;
struct SAngle 	stcAngle;

float yaw_angel;
float yaw_speed;

u16 MPU_USART_RX_STA=0;      								 //����״̬���	
uint8_t Mpu_Rx_Buffer[MPU_RC_BUFFER_SIZE] ={0};	//���ջ�������


void mpu_rxdata_deal(uint8_t *ucRxBuffer)
{
	//printf("���ڽ����жϣ�%d \r\n",aRxBuffer2[0]);

		if (ucRxBuffer[0]!=0x55) 									//����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
		{
			return;
		}
		else
		{
			switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
			{
				//������ĵ���λ��ʼ����,��ֱ����switch�����н�����㣬��ռ��ʱ��
				case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8); /*yaw_speed = (float)stcGyro.w[2]/32768*2000;    */break;						//���������ǽ��ٶ�����
				case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);/*yaw_angel = (float)stcAngle.Angle[2]/32768*180;*/break;						//���������ǽǶ�����

			}
		}
}


