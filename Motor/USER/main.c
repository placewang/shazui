
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
		Input_Init();
		UART1_ITRX_InIt(115200);
		TIM3_PWM_Input_Test();
		SPI_1_32bit_Init(SPI,32);
		CAN_Filter_20GroupInit(CAN_1M);
		MotorCanInit();

		while(1)
		{
			if(Posv==0)
			{
				if(DreMoveZero())
				{
					Posv=1;
				}
			}
			if(Posv==1)
			{
					KnifeSelection(16);
					Posv=2;
			}
			if(Posv==2)
			{
				CloseKnife(1);
				Posv=3;
			}
			ReadAnPackData(&MRevBuff);
//			if(MRevBuff.TaskTime>=315)
//			{
//					PollingMotorSta();	
//					MRevBuff.TaskTime=0;
//			}
		}
}

