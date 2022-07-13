
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
	  MoveZero(0x02,100,200);
		while(1)
		{
			
				if(MExeSta.MoveZeroStaY)
					{
				  	MoveToTargetPos(0x64,1350,0x02);
						MExeSta.MoveZeroStaY=0;
					}	
//					if(MExeSta.MoveTargetStaY)
//					{
//						MoveToTargetPos(0x64,4038,0x02);
//						MExeSta.MoveTargetStaY=0;
//						MRevBuff.YMoveTargetval=0;
//					}
					ReadAnPackData(&MRevBuff);
					MotorStaRenew();
//					if(MRevBuff.TaskTime>=315)
//					{
//						MotorStaRenew();
//						PollingMotorSta();
//						ReadAnPackData(&MRevBuff);
//						MRevBuff.TaskTime=0;
//					}


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

