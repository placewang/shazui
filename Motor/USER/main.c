
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
//		MotorCanInit();
		
		while(1)
		{	

			ReadAnPackData(&MRevBuff);       //���������ó�����
			if(MRevBuff.TaskTime>=500)       //��ʱ�������λ�� �ɲ�Ҫ
			{
					PollingMotorSta();	
					MRevBuff.TaskTime=0;
			}
		}
}



















//			if(MRevBuff.MLen(&MRevBuff)>0)
//			{
//				MRevBuff.DeQueue(&MRevBuff,&Canval);
//				McanPeliTxBuff.CANID     =Canval.ID;                                       //Can����ID
//				McanPeliTxBuff.DLC       =Canval.DLC;                                      //���ݳ���
//				McanPeliTxBuff.CANIDtype =Canval.FF;                                       //ID���ͣ���׼֡/��չ֡��
//				McanPeliTxBuff.RTR       =Canval.RTR;                                      //֡���ͣ�Զ��/���ݣ� 	
//				for(int i=0;i<McanPeliTxBuff.DLC;i++)
//				{
//					McanPeliTxBuff.Data[i]=Canval.Data[i];	
//				}				
//				while(CAN_GetFlagStatus(CAN1,CAN_STATUS_TS)){;}                    //�ȴ����߿���
//				while(CAN_GetFlagStatus(CAN1,CAN_STATUS_RS)){;}	
//				while(!CAN_GetFlagStatus(CAN1,CAN_STATUS_TCS)){;}		
//				Send_CANFrame(&McanPeliTxBuff);
//			}

