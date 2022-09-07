
// Define to prevent recursive inclusion

#define _MAIN_C_

#include "delay.h"
#include "led.h"
#include "uart.h"
#include "tim3_pwm_input.h"
#include "spi.h"
#include "can.h"
#include "MotorCan.h"

uint8_t SPI_sencallk[5]={0};
int16_t myspeed=0;
uint16_t torq=0;
int32_t  Posv=0;
CanPeliRxMsg Canval;
CanTxMsg  McanPeliTxBuff;
int main()
{
		DELAY_Init();
		LED_Init();
		Input_Init();
		UART1_ITRX_InIt(115200);
		TIM3_PWM_Input_Test();
		SPI_1_32bit_Init(SPI,32);
		CAN_Filter_20GroupInit(CAN_1M);
		MotorCanInit(&MtProperty1_L);
		MotorCanInit(&MtProperty1_R);
		while(1)
		{	
			if(Posv==0)
			{

				Posv=DreMoveZero(&MtProperty1_L);
				
			}
			if(torq==0)
			{
				torq=DreMoveZero(&MtProperty1_R);				
			}				
			
		 if(torq==1&&KnifeSelection2(&MtProperty1_R,1))
			{
					torq=2;
			}
			if(torq==2&&CloseKnife2(&MtProperty1_R,1))
			{
				torq=3;
			}
			if(torq==3&&KnifeSelection2(&MtProperty1_R,15))
			{
					torq=4;
			}
			if(torq==4&&CloseKnife2(&MtProperty1_R,15))
			{
				torq=1;
			}
//			
			if(Posv==1&&KnifeSelection2(&MtProperty1_L,1))
			{
					Posv=2;
			}
			if(Posv==2&&CloseKnife2(&MtProperty1_L,1))
			{
				Posv=3;
			}
			if(Posv==3&&KnifeSelection2(&MtProperty1_L,16))
			{
					Posv=4;
			}
			if(Posv==4&&CloseKnife2(&MtProperty1_L,16))
			{
				Posv=1;
			}
			
			ReadAnPackData(&MRevBuff);       //队列数据拿出解析
			if(MRevBuff.TaskTime>=30)       //定时查编码器位置 可不要
			{
					PollingMotorSta();	
					MRevBuff.TaskTime=0;
			}
		}
}



















//			if(MRevBuff.MLen(&MRevBuff)>0)
//			{
//				MRevBuff.DeQueue(&MRevBuff,&Canval);
//				McanPeliTxBuff.CANID     =Canval.ID;                                       //Can发送ID
//				McanPeliTxBuff.DLC       =Canval.DLC;                                      //数据长度
//				McanPeliTxBuff.CANIDtype =Canval.FF;                                       //ID类型（标准帧/扩展帧）
//				McanPeliTxBuff.RTR       =Canval.RTR;                                      //帧类型（远程/数据） 	
//				for(int i=0;i<McanPeliTxBuff.DLC;i++)
//				{
//					McanPeliTxBuff.Data[i]=Canval.Data[i];	
//				}				
//				while(CAN_GetFlagStatus(CAN1,CAN_STATUS_TS)){;}                    //等待总线空闲
//				while(CAN_GetFlagStatus(CAN1,CAN_STATUS_RS)){;}	
//				while(!CAN_GetFlagStatus(CAN1,CAN_STATUS_TCS)){;}		
//				Send_CANFrame(&McanPeliTxBuff);
//			}

