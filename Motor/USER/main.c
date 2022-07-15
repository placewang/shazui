
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

int main()
{
		DELAY_Init();
		LED_Init();
		UART1_ITRX_InIt(115200);
		TIM3_PWM_Input_Test();
		SPI_1_32bit_Init(SPI,32);
		CAN_Filter_20GroupInit(CAN_1M);
		MotorCanInit();
	  MoveZero(0x01,200,100);
//	  MoveZero(0x02,100,200);
		while(1)
		{
				ReadAnPackData(&MRevBuff);
				MotorStaRenew();
//				if(MExeSta.MoveZeroStaY)
//				{
//				  	MoveToTargetPos(0x64,2008,0x02);
//						ClearMStat(0x02,1);
//				}
				if(MExeSta.MoveZeroStaX)
				{
				 	  MoveToTargetPos(1000,61405,0x01);
						ClearMStat(0x01,1);
				}	
			if(MExeSta.MoveTargetStaX)
			{
					MoveToTargetPos(1000,0,0x01);
					ClearMStat(0x01,0);
			}
			if(MRevBuff.TaskTime>=315)
			{
							PollingMotorSta();	
							MRevBuff.TaskTime=0;
			}
//			if(SK_IN6)
//				{
//					DELAY_Ms(1000);
//				}
//				else
//				{
//					DELAY_Ms(1000);
//				}
		}
}

