//��������� �������жϵĴ���

#include "ranging_task.h"
#include "tim.h"
#include "stdio.h"

uint8_t   TIM2CH1_CAPTURE_STA=0;    //���벶��״̬   
uint8_t   TIM5CH2_CAPTURE_STA=0; 
uint32_t    TIM2CH1_CAPTURE_VAL;    //���벶��ֵ(TIM2/TIM5��32λ)
uint32_t    TIM5CH2_CAPTURE_VAL; 
uint16_t Temp1=30;
uint16_t Temp2=30;
long long temp1=0;
long long temp2=0;
void Ranging_TIM2_ICtask(void) ////��ʱ�����벶���жϴ���ص������������
{
	if((TIM2CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
    {
        if(TIM2CH1_CAPTURE_STA&0X40)        //����һ���½���      
            {               
                TIM2CH1_CAPTURE_STA|=0X80;      //��ǳɹ�����һ��
                TIM2CH1_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);//��ȡ��ǰ�Ĳ���ֵ.
                TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ��������
                TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//��ʱ��2ͨ��1����Ϊ�����ز���

            }
        else                                //��δ��ʼ,��һ�β���������
            {
                TIM2CH1_CAPTURE_STA=0;          //���
                TIM2CH1_CAPTURE_VAL=0;
                TIM2CH1_CAPTURE_STA|=0X40;      //��ǲ�����������
                __HAL_TIM_DISABLE(&htim2);        //�رն�ʱ��2
                __HAL_TIM_SET_COUNTER(&htim2,0);
                TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ��������
                TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);//��ʱ��2ͨ��1����Ϊ���½���? }           
				__HAL_TIM_ENABLE(&htim2);//ʹ�ܶ�ʱ��2
             }       
	}
}

void Ranging_TIM5_ICtask(void) ////��ʱ�����벶���жϴ���ص������������
{
	if((TIM5CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
    {
        if(TIM5CH2_CAPTURE_STA&0X40)        //����һ���½���      
            {               
                TIM5CH2_CAPTURE_STA|=0X80;      //��ǳɹ�����һ��
                TIM5CH2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2);//��ȡ��ǰ�Ĳ���ֵ.
                TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2);   //һ��Ҫ�����ԭ��������
                TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//��ʱ��2ͨ��1����Ϊ�����ز���

            }
        else                                //��δ��ʼ,��һ�β���������
            {
                TIM5CH2_CAPTURE_STA=0;          //���
                TIM5CH2_CAPTURE_VAL=0;
                TIM5CH2_CAPTURE_STA|=0X40;      //��ǲ�����������
                __HAL_TIM_DISABLE(&htim5);        //�رն�ʱ��2
                __HAL_TIM_SET_COUNTER(&htim5,0);
                TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2);   //һ��Ҫ�����ԭ��������
                TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);//��ʱ��2ͨ��1����Ϊ���½���? }           
				__HAL_TIM_ENABLE(&htim5);//ʹ�ܶ�ʱ��2
             }       
	}
}

void Ranging_TIM2_PEtask(void)  //�жϻص�������
{
			 if((TIM2CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
		{
            if(TIM2CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
            {
                if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
                {
                    TIM2CH1_CAPTURE_STA|=0X80;      //��ǳɹ�������һ��
                    TIM2CH1_CAPTURE_VAL=0XFFFFFFFF;
                }
                else TIM2CH1_CAPTURE_STA++;
            }    
		}
}
void Ranging_TIM5_PEtask(void)  //�жϻص�������
{
			 if((TIM5CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����
		{
            if(TIM5CH2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
            {
                if((TIM5CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
                {
                    TIM5CH2_CAPTURE_STA|=0X80;      //��ǳɹ�������һ��
                    TIM5CH2_CAPTURE_VAL=0XFFFFFFFF;
                }
                else TIM5CH2_CAPTURE_STA++;
            }    
		}
}

 
void Get_Ranging_Date1(void) //10ms���һ��
{	
	if(TIM2CH1_CAPTURE_STA&0X80)        //�ɹ�������һ��
    {
		temp1=TIM2CH1_CAPTURE_STA&0X3F; 
		temp1*=0XFFFFFFFF;               //���ʱ���ܺ�
		temp1+=TIM2CH1_CAPTURE_VAL;      //�õ��ܵĸߵ�ƽʱ��
		Temp1=temp1*17/1000;			
		TIM2CH1_CAPTURE_STA=0; 
	}
	if(Temp1>300)
		Temp1=30;
}
void Get_Ranging_Date2(void) //10ms���һ��
{	
	if(TIM5CH2_CAPTURE_STA&0X80)        //�ɹ�������һ��
    {
		temp2=TIM5CH2_CAPTURE_STA&0X3F; 
		temp2*=0XFFFFFFFF;               //���ʱ���ܺ�
		temp2+=TIM5CH2_CAPTURE_VAL;      //�õ��ܵĸߵ�ƽʱ��
		Temp2=temp2*17/1000;			
		TIM5CH2_CAPTURE_STA=0; 
	}
	if(Temp2>300)
		Temp2=30;
}


