//超声波测距 包括在中断的代码

#include "ranging_task.h"
#include "tim.h"
#include "stdio.h"

uint8_t   TIM2CH1_CAPTURE_STA=0;    //输入捕获状态   
uint8_t   TIM5CH2_CAPTURE_STA=0; 
uint32_t    TIM2CH1_CAPTURE_VAL;    //输入捕获值(TIM2/TIM5是32位)
uint32_t    TIM5CH2_CAPTURE_VAL; 
uint16_t Temp1=30;
uint16_t Temp2=30;
long long temp1=0;
long long temp2=0;
void Ranging_TIM2_ICtask(void) ////定时器输入捕获中断处理回调函数里面调用
{
	if((TIM2CH1_CAPTURE_STA&0X80)==0)//还未成功捕获
    {
        if(TIM2CH1_CAPTURE_STA&0X40)        //捕获到一个下降沿      
            {               
                TIM2CH1_CAPTURE_STA|=0X80;      //标记成功捕获到一次
                TIM2CH1_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);//获取当前的捕获值.
                TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1);   //一定要先清除原来的设置
                TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//定时器2通道1设置为上升沿捕获

            }
        else                                //还未开始,第一次捕获上升沿
            {
                TIM2CH1_CAPTURE_STA=0;          //清空
                TIM2CH1_CAPTURE_VAL=0;
                TIM2CH1_CAPTURE_STA|=0X40;      //标记捕获到了上升沿
                __HAL_TIM_DISABLE(&htim2);        //关闭定时器2
                __HAL_TIM_SET_COUNTER(&htim2,0);
                TIM_RESET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1);   //一定要先清除原来的设置
                TIM_SET_CAPTUREPOLARITY(&htim2,TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);//定时器2通道1设置为上下降沿? }           
				__HAL_TIM_ENABLE(&htim2);//使能定时器2
             }       
	}
}

void Ranging_TIM5_ICtask(void) ////定时器输入捕获中断处理回调函数里面调用
{
	if((TIM5CH2_CAPTURE_STA&0X80)==0)//还未成功捕获
    {
        if(TIM5CH2_CAPTURE_STA&0X40)        //捕获到一个下降沿      
            {               
                TIM5CH2_CAPTURE_STA|=0X80;      //标记成功捕获到一次
                TIM5CH2_CAPTURE_VAL=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_2);//获取当前的捕获值.
                TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2);   //一定要先清除原来的设置
                TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2,TIM_ICPOLARITY_RISING);//定时器2通道1设置为上升沿捕获

            }
        else                                //还未开始,第一次捕获上升沿
            {
                TIM5CH2_CAPTURE_STA=0;          //清空
                TIM5CH2_CAPTURE_VAL=0;
                TIM5CH2_CAPTURE_STA|=0X40;      //标记捕获到了上升沿
                __HAL_TIM_DISABLE(&htim5);        //关闭定时器2
                __HAL_TIM_SET_COUNTER(&htim5,0);
                TIM_RESET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2);   //一定要先清除原来的设置
                TIM_SET_CAPTUREPOLARITY(&htim5,TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);//定时器2通道1设置为上下降沿? }           
				__HAL_TIM_ENABLE(&htim5);//使能定时器2
             }       
	}
}

void Ranging_TIM2_PEtask(void)  //中断回调函数的
{
			 if((TIM2CH1_CAPTURE_STA&0X80)==0)//还未成功捕获
		{
            if(TIM2CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
            {
                if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
                {
                    TIM2CH1_CAPTURE_STA|=0X80;      //标记成功捕获了一次
                    TIM2CH1_CAPTURE_VAL=0XFFFFFFFF;
                }
                else TIM2CH1_CAPTURE_STA++;
            }    
		}
}
void Ranging_TIM5_PEtask(void)  //中断回调函数的
{
			 if((TIM5CH2_CAPTURE_STA&0X80)==0)//还未成功捕获
		{
            if(TIM5CH2_CAPTURE_STA&0X40)//已经捕获到高电平了
            {
                if((TIM5CH2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
                {
                    TIM5CH2_CAPTURE_STA|=0X80;      //标记成功捕获了一次
                    TIM5CH2_CAPTURE_VAL=0XFFFFFFFF;
                }
                else TIM5CH2_CAPTURE_STA++;
            }    
		}
}

 
void Get_Ranging_Date1(void) //10ms检测一次
{	
	if(TIM2CH1_CAPTURE_STA&0X80)        //成功捕获到了一次
    {
		temp1=TIM2CH1_CAPTURE_STA&0X3F; 
		temp1*=0XFFFFFFFF;               //溢出时间总和
		temp1+=TIM2CH1_CAPTURE_VAL;      //得到总的高电平时间
		Temp1=temp1*17/1000;			
		TIM2CH1_CAPTURE_STA=0; 
	}
	if(Temp1>300)
		Temp1=30;
}
void Get_Ranging_Date2(void) //10ms检测一次
{	
	if(TIM5CH2_CAPTURE_STA&0X80)        //成功捕获到了一次
    {
		temp2=TIM5CH2_CAPTURE_STA&0X3F; 
		temp2*=0XFFFFFFFF;               //溢出时间总和
		temp2+=TIM5CH2_CAPTURE_VAL;      //得到总的高电平时间
		Temp2=temp2*17/1000;			
		TIM5CH2_CAPTURE_STA=0; 
	}
	if(Temp2>300)
		Temp2=30;
}


