#ifndef _VISION_INTERACT_H
#define _VISION_INTERACT_H
#include "stm32f4xx_hal.h"
#include "MYtype.h"

/*--------��λ��Э��--------*/
//���ڹٷ�����ϵͳ���Ӿ�ͨѶ��ͨѶЭ���һЩע��
//�����Ӿ���ͨѶЭ��Ϊ�׵�ַΪ0XA5��SEQ֡���Ϊ2����Ϊ��ʱΪ�������ͣ����������ݳ��ȶ�
//Ҫע�����CRC8У��Ϊһ�ֽڣ�CRC16Ϊ���ֽڡ�CRC8��ͷУ�飬CRC16������������У��
//---header-------------------------------------//---data----------------//------tail--------//
//--�׵�ַ---֡���---ģʽѡ��-----CRC8У��λ--//---����������λ����--//---CRC16У��λ----//
//--0XA5-----2--------ռһ�ֽ�----ռһ�ֽ�----//---ռ���ֽ�------------//----ռ���ֽ�------//
/*--ע��--*/
//д�� CRC8֡ͷУ���� Append_CRC8_Check_Sum(param1,param2)
//param1 Ϊ��Ҫд������飬param2ΪCRC8д�������ݳ��ȡ���Ϊ��������0XA5��SEQ֡���λ֮������ݳ���Ϊ����crc8У����д��[2]��
//д�� CRC16����У���� Append_CRC16_Check_Sum(param1,param2)
//param1 Ϊ��Ҫд������飬param2Ϊд�����������ݳ��ȡ�ͬ��
//�Ӿ�����������16λ����ƫ�����ݸߵͰ�λ��ת��Ҫ����һ��ת����
#define    PROTOCOL_TEST  2

#define    NOW_t            1
#define    LAST_t           0

#define    SE_BUFFER_SIZE 20  //�Ӿ��������ݻ����ֽ�
#define    RC_BUFFER_SIZE 100 //�Ӿ��������ݻ����ֽ�
//��ʼ�ֽڣ� Ϊ0XA5
#define    VIOSN_SOF 0XA5
//֡���
#define    VISON_SEQ 2

//Э��֡����
#define    VISON_LEN_HAEDER 4 //֡ͷ��
#define    VISON_LEN_PACKED 14//���ݰ�����
#define    VISON_SEND_LENGTH 14//�Ӿ��������ݳ���
//�Ӿ�ģʽ����
#define    VISION_BLUE     (0X01)
#define    VISION_RED      (0X02)
//�Ӿ�ģʽ�л�
#define    VISION_MANUAL       (0X00)
#define    VISION_AUTO         (0x01)
#define    VISION_HBUFF        (0X02)
#define    VISION_LBUFF        (0X03)
//����ģʽѡ��
//#define    FeedBack_px     1  //����ƫ���
//#define    FeedBack_angle  2  //�Ƕ�ƫ���
/*--------��̨ģʽö��--------*/
typedef enum
{
  FeedBack_px, 
  FeedBack_angle,
} vision_feedback_e;

//��λ�����սṹ��
typedef __packed struct
{
	//֡ͷ
	uint8_t SOF;//�׵�ַ
	uint8_t seq; //֡���
	uint8_t model;//ģʽ
	uint8_t crc8;//CRC8У��λ
	
	//����
	float visionYawData;//�Ӿ�����YAW������
	float visionPitchData;//�Ӿ�����PITCH������
	//uint8_t VisionTrackFlag;//�Ӿ��Ƿ�ʶ��Ŀ��
	
	//����β
	uint16_t crc16;//CRC16У��λ
}stoVisonRecvData_t;

//��λ�����ͽṹ��,֡ͷ�����ݶ�֡βУ��ֿ�����Ϊ�ڴ������ݼ���ʱ����Ҫ�ֽڶ���
typedef  struct//ֻ��������
{
  //֡ͷ
	uint8_t SOF;//�׵�ַ
	uint8_t robotColor; //װ�װ���ɫʶ�� 0x01 ��װ�װ� 0x02 ��ɫװ�װ�
	uint8_t model;//ģʽ�л� 0x01 ���飬 0x02С�� 0x03���
	uint8_t crc8;//CRC8У��λ
}traVisonHeader_t;
typedef  struct
{
	//��������
	float YawAngle;//YAW������ƫ��
	float PitchAngle;//PITCH������ƫ��
  
		//����β
	uint16_t crc16;//CRC16У��λ
	
}traVisonSendData_t;



//��ʽת��������"�������Ա���������ڴ�"
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FloatTrans;

//���������������ȫ�ֱ���


extern stoVisonRecvData_t  VisonRecvData;//�Ӿ����սṹ��




extern uint8_t Rx_Buffer[RC_BUFFER_SIZE];//���ջ�������
extern uint8_t Visoin_Buffer[RC_BUFFER_SIZE];//�Ӿ���������

extern float  YawLast_T;
extern float  YawPreSpeed;
extern uint32_t count_time;//֡����
extern uint32_t time_ping;//TIM3����ֵ100us
extern uint8_t SEND_FALG;//���ݷ��ͱ�־

/*--------�Ӿ����ݶ�������--------*/
void sendVisionData(uint8_t ModleID,uint8_t Cmd);//�����Ӿ�����
void vision_connect(uint8_t *RecvUsartData);//�����Ӿ�����
/*--------�Ӿ�����ƫ����º���--------*/
float Vision_Yaw_Error(float *Yaw_Error);
void Vision_Pitch_Error(float *Pitch_Error);
/*--------�������Ͳ�������--------*/
void Float_to_Byte(float *target,unsigned char *buf,unsigned char beg);
void rev_shrort_data(void);
/*--------����֡����--------*/
uint16_t ping_count(void);//
/*--------����DMA����--------*/
void HAL_UART3_Receive_IT_IDLE(void);//����1DMA����
/*-------�Ӿ����ܺ���--------*/
bool_t Vision_UpDate(void);
void Vision_UpDate_Clean(void);
#if  PROTOCOL_TEST==1

extern PC_interact_Rc_data_set PC_interact_Rc_data;//����3�鸡����
extern u8 usart1_Recv_End_flag;//������ɱ�־λ
extern uint8_t seq;//֡���

typedef struct//�������ݽṹ��
{
	float data1;
	float data2;
	float data3;
	uint8_t masks;
}PC_interact_data_set;

typedef struct//�������ݽṹ��
{
	float data1;
	float data2;
	float data3;
	uint8_t masks;
}PC_interact_Rc_data_set;

void PC_interact_data_send_test(float Data_a,float Data_b, float Data_c);
void PC_interact_Rc_data_test(void);
void PC_interact_Rc_data_Replay(void);//����1��������

#endif


#endif
