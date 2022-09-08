#include "MotorCan.h"
//#include "delay.h"


/********************************************************************************
Y轴（剪刀电机）回零函数-上位机主动查询外部光电传感器状态
左右X轴移动一次间隙量
@Mpy:   电机属性句柄
@torque:运动扭矩
@tarSeep:运动速度
@MID：   设备ID
********************************************************************************/
#if YMOVEZEROINQUIRE
signed char YMoveZero(MotorProperty* Mpy,const signed short torque,signed int tarSeep)
{
		if(Mpy==NULL)
		{
			return -1;
		}
		if(Mpy->Mst->step==0)
		{
				MoveSeepMode(Mpy,torque,tarSeep,1);
				if(Mpy->Mst->SenSorSta[1])
				{
					Mpy->Mst->step=1;
				}
				else
				{
					Mpy->Mst->step=3;
				}
		}
		else if((Mpy->Mst->step==1||Mpy->Mst->step==2)&&!Mpy->Mst->SenSorSta[1])
		{
			MoveSeepMode(Mpy,torque,0x00,1);                                              //停电机
      MorotSetEncodervalue(Mpy,1,0);                                                //设置编码器值0					
			MoveToTargetPos(Mpy,0x255,Mpy->Mcg->MotorYLACUNA,1);                          //移动至齿轮水平位
			Mpy->Mst->step=4;
		}
		else if(Mpy->Mst->step==4&&Mpy->Mst->MoveTargetSta[1])
		{
			Mpy->Mst->step=0;
			return 1;
		}
		else if(Mpy->Mst->step==3)
		{
			if(Mpy->Mst->SenSorSta[1])
			{
				Mpy->Mst->step =2;
			}
		}
		else if(Mpy->Mst->step>0&&Mpy->Mst->MoveTimeOut[1]>=100*30)                              //超时处理4S
		{
			MoveSeepMode(Mpy,torque,0x00,1); 
			EnableOrClearALarm(Mpy->Mcg->MotorID[1],0);
			EnableOrClearALarm(Mpy->Mcg->MotorID[1],3);
			Mpy->Mst->step=0;
			return -1;
		}	
		return 0;
}
#endif


/********************************************************************************
Y轴（剪刀电机）回零函数--电机本体依靠外部光电传感器
@Mpz:电机属性句柄
@torque:运动扭矩
@tarSeep:运动速度
@MID：   设备ID
********************************************************************************/
# if YMOVEZEROBACK
signed char YMoveZero(MotorProperty* Mpz,const signed short torque,signed int tarSeep)
{
		if(Mpz==NULL)
		{
			return -1;
		}
		if(Mpz->Mst->step==0)
		{
			MoveZero(Mpz,1,tarSeep,torque,Mpz->Mcg->ReturnZeroDirection[1],Mpz->Mcg->MotorYLACUNA);
			Mpz->Mst->step=1;
		}
		else if(!Mpz->Mst->MoveZeroSta[1]&&Mpz->Mst->step>0&&Mpz->Mst->MoveTimeOut[1]>=1000*2)
		{
			EnableOrClearALarm(Mpz->Mcg->MotorID[1],0);
			EnableOrClearALarm(Mpz->Mcg->MotorID[1],3);
			Mpz->Mst->step=0;
			return -1;			
		}		
		else if(Mpz->Mst->MoveZeroSta[1]&&Mpz->Mst->step==1)
		{
			MoveZero(Mpz,1,tarSeep,torque,Mpz->Mcg->ReturnZeroDirection[1],Mpz->Mcg->MotorYLACUNA);
			Mpz->Mst->step=2;
		}
		else if(Mpz->Mst->MoveZeroSta[1]&&Mpz->Mst->step==2)
		{
			MoveZero(Mpz,1,tarSeep,torque,Mpz->Mcg->ReturnZeroDirection[1],Mpz->Mcg->MotorYLACUNA);
			Mpz->Mst->step=3;
		}
		else if(Mpz->Mst->MoveZeroSta[1]&&Mpz->Mst->step==3)
		{
			MoveZero(Mpz,1,tarSeep,torque,Mpz->Mcg->ReturnZeroDirection[1],Mpz->Mcg->MotorYLACUNA-2200);
			Mpz->Mst->step=4;
		}
		else if(Mpz->Mst->MoveZeroSta[1]&&Mpz->Mst->step==4)
		{
			Mpz->Mst->step=0;
			return 1;
		}		
		return 0;
}


#endif

/******************************************************************
Y回零试错
@Mptry:   电机属性句柄
解决上电回零齿轮与齿条错位卡顿（循环6次移动齿轮位置错开卡顿）
********************************************************************/

signed char YMoveZeroTrial(MotorProperty* Mptry)
{
			signed   short  SeepYMZ=300;
			signed   short  TorqueYMZ=980;
			signed   short  seepXMZ=800;
			signed   short  TorqueXMZ=300;
			
			if(Mptry==NULL)
			{
				return -1;
			}		
			if(Mptry->Mst->TrialCount==12)
			{
				Mptry->Mst->TrialState=0;
				Mptry->Mst->TrialCount =0;
				return -1;
			}

			if(Mptry->Mst->TrialState==0)
			{
				 MoveZero(Mptry,0,seepXMZ,TorqueXMZ,Mptry->Mcg->ReturnZeroDirection[0],Mptry->Mcg->MotorXLACUNA);
				 Mptry->Mst->TrialState=2;
			}
			else if(Mptry->Mst->TrialState==2&&!Mptry->Mst->MoveZeroSta[0]&&Mptry->Mst->MoveTimeOut[0]>100*10*2)
			{
				EnableOrClearALarm(Mptry->Mcg->MotorID[0],0);
				EnableOrClearALarm(Mptry->Mcg->MotorID[0],3);
				Mptry->Mst->TrialState=-1;
			}
			else if(Mptry->Mst->TrialState==2&&Mptry->Mst->MoveZeroSta[0])
			{	
				Mptry->Mst->TrialState=YMoveZero(Mptry,TorqueYMZ,SeepYMZ);
				if(Mptry->Mst->TrialState==0)
				{
					Mptry->Mst->TrialState=2;
				}
			}	
		
			else if(Mptry->Mst->TrialState==-1)
			{
				MoveZero(Mptry,0x00,seepXMZ,TorqueXMZ,Mptry->Mcg->ReturnZeroDirection[0],0x280);
				Mptry->Mst->TrialState=3;
			}
			
			else if(Mptry->Mst->TrialState==3&&Mptry->Mst->MoveZeroSta[0])
			{
					MoveSeepMode(Mptry,0x800,0x90,1);
					Mptry->Mst->TrialState=4;
			}
			else if(Mptry->Mst->TrialState==4&&Mptry->Mst->MoveTimeOut[1]>100*4)
			{
					MoveSeepMode(Mptry,0x800,0x00,1);
					Mptry->Mst->TrialCount++;
					Mptry->Mst->TrialState=0;
			}
			else if(Mptry->Mst->TrialState==1)
			{
				Mptry->Mst->TrialState=0;
				Mptry->Mst->TrialCount =0;
				return 1;
			}				
	return 0;
}

/*
剪刀复位步骤1
@Mpr1:电机属性句柄
*/
signed char RestStep1(MotorProperty* Mpr1,signed short seepXMZ ,signed short seepYMZ,const signed short torque)
{
		if(Mpr1==NULL)
		{
				return -1;
		}
		if(Mpr1->Mst->stepnum==0)
		{
			MoveSeepMode(Mpr1,torque,seepYMZ*-1,1);         //推出剪刀
			Mpr1->Mst->stepnum=1;
		}
		else if(Mpr1->Mst->stepnum==1&&Mpr1->Mst->MoveTimeOut[1]>=100*1.5)
		{
				MoveSeepMode(Mpr1,torque,0x00,1);  
				if(Mpr1->Mst->Restnum==15)
				{
					MoveToTargetPos(Mpr1,seepXMZ,Mpr1->Mcg->XMoveStandard[Mpr1->Mst->Restnum]-Mpr1->Mcg->MotorXYLACUNA,0);                             //运动回初始位
				}
				else
				{
					MoveToTargetPos(Mpr1,seepXMZ,Mpr1->Mcg->XMoveStandard[Mpr1->Mst->Restnum]+Mpr1->Mcg->MotorXYLACUNA,0);                             //运动回初始位
				}	
				Mpr1->Mst->stepnum=2;
		}
		else if(Mpr1->Mst->stepnum==2&&Mpr1->Mst->MoveTargetSta[0])
		{
			MoveToTargetPos(Mpr1,0x155,Mpr1->Mcg->MotorYLACUNA,1);
			Mpr1->Mst->stepnum=3;
		}
		else if(Mpr1->Mst->stepnum==3&&Mpr1->Mst->MoveTargetSta[1])
		{
			MoveToTargetPos(Mpr1,seepXMZ,Mpr1->Mcg->XMoveStandard[Mpr1->Mst->Restnum],0);                             //运动回初始位
			Mpr1->Mst->stepnum=4;
		}
		
		else if(Mpr1->Mst->stepnum==4&&Mpr1->Mst->MoveTargetSta[0])
		{
			Mpr1->Mst->stepnum=0;
			Mpr1->Mst->ifstat=1;
			return 1;
		}
		return 0;
}

/*
剪刀复位步骤2
@Mpr2:电机属性句柄
*/
signed char RestStep2(MotorProperty* Mpr2,signed short seepXMZ ,signed short seepYMZ,const signed short torque)
{
		if(Mpr2==NULL)
		{
				return -1;
		}
		if(Mpr2->Mst->stepnum==0)
		{
			MoveSeepMode(Mpr2,torque,seepYMZ,1);         //收剪刀
			Mpr2->Mst->stepnum=1;
		}
		
		else if(Mpr2->Mst->stepnum==1&&Mpr2->Mst->MoveTimeOut[1]>=100*1.5)
		{
					MoveSeepMode(Mpr2,torque,0x00,1); 
			
				if(Mpr2->Mst->Restnum==15||!Mpr2->Mst->SenSorSta[0])
				{
					if(Mpr2->Mst->Restnum==0&&!Mpr2->Mst->SenSorSta[0])
					{
						MoveToTargetPos(Mpr2,seepXMZ, Mpr2->Mcg->XMoveStandard[Mpr2->Mst->Restnum]+Mpr2->Mcg->MotorXYLACUNA,0);  
					}
					else
					{
						MoveToTargetPos(Mpr2,seepXMZ, Mpr2->Mcg->XMoveStandard[Mpr2->Mst->Restnum]-Mpr2->Mcg->MotorXYLACUNA,0); 					
					}
					MoveToTargetPos(Mpr2,0x255,Mpr2->Mcg->MotorYLACUNA,1);                                              //移动至齿轮水平位	
					Mpr2->Mst->stepnum=4;
				}
				else
				{
					MoveToTargetPos(Mpr2,seepXMZ, Mpr2->Mcg->XMoveStandard[Mpr2->Mst->Restnum]+Mpr2->Mcg->MotorXYLACUNA,0);        
					Mpr2->Mst->stepnum=2;
				}	
				
		}
		else if(Mpr2->Mst->stepnum==2&&Mpr2->Mst->MoveTargetSta[0])
		{
			MoveToTargetPos(Mpr2,0x255,Mpr2->Mcg->MotorYLACUNA, 1);                                                   //移动至齿轮水平位	
			Mpr2->Mst->stepnum=3;
		}

		else if(Mpr2->Mst->stepnum==4&&Mpr2->Mst->MoveTargetSta[1]&&Mpr2->Mst->MoveTimeOut[1]>=100*1.5)
		{
			  Mpr2->Mst->ifstat=3;
		}		
		else if(Mpr2->Mst->stepnum==3&&Mpr2->Mst->MoveTargetSta[1]&&Mpr2->Mst->MoveTimeOut[1]>=100*1.3)
		{
				Mpr2->Mst->stepnum=0;
			  Mpr2->Mst->ifstat=2;
		}
		return 0;
}

/*
剪刀复位步骤3
@Mpr3:电机属性句柄
*/
signed char RestStep3(MotorProperty* Mpr3,signed short seepXMZ ,signed short seepYMZ)
{
		if(Mpr3==NULL)
		{
				return -1;
		}	
		if(Mpr3->Mst->stepnum==0)
		{
			MoveToTargetPos(Mpr3,seepXMZ, Mpr3->Mcg->XMoveStandard[Mpr3->Mst->Restnum+1],0);                             //下一位
			(Mpr3->Mst->Restnum)++;
			Mpr3->Mst->stepnum=1;
		}
		else if(Mpr3->Mst->stepnum==1&&(Mpr3->Mst->MoveTargetSta[0]||Mpr3->Mst->MoveTimeOut[1]>=100*20))
		{
			if(Mpr3->Mst->MoveTimeOut[1]>=100*20&&!Mpr3->Mst->MoveTargetSta[1])                                        //堵转超时
			{
				EnableOrClearALarm(Mpr3->Mcg->MotorID[1],0);
				EnableOrClearALarm(Mpr3->Mcg->MotorID[1],3);
			}
			Mpr3->Mst->stepnum=0;                                                                                      //循环返回step1 直到剪刀全部复位
			Mpr3->Mst->ifstat=0;
		}	
		return 0;
}



/**********************************************************************
剪刀复位
@Mps:电机属性句柄
***********************************************************************/
signed char ScissorsReset(MotorProperty* Mps)
{
		signed short seepX=2000;
		signed short seepY=500;
	  signed short torqueY=900;

		switch (Mps->Mst->ifstat)
		{
					case  0 :
						RestStep1(Mps,seepX,seepY,torqueY);
						break;
					case 1:
						RestStep2(Mps,seepX,seepY,torqueY);
						break;
					case 2:
   					RestStep3(Mps,seepX,seepY);
						break;
					case 3:

							Mps->Mst->ifstat=0;			
							Mps->Mst->Restnum=0;
							Mps->Mst->stepnum=0;
							if(!Mps->Mst->SenSorSta[0])
							{ 
								return 1;
							}
							else
							{
								return -1;
							}
//					break;
		}
	return 0;
}
/************************************************************************
设备回零
@Mpdev:电机属性句柄
**************************************************************************/
signed char DreMoveZero(MotorProperty* Mpdev)
{ 	
		signed   short  seepXMZ=800;
		signed   short  TorqueXMZ=930;
		signed   short  SeepYMZ=400;
		signed   short  TorqueYMZ=980;
	  signed   char   OriginalEncodedX1=0;
		signed   char   ScissorsresetSta =0;
	
		if(Mpdev==NULL)
		{
			return -1;
		}
		if(Mpdev->Mst->RunState==0)
		{
				clearM(Mpdev);
				Mpdev->Mst->RunState=1;
		}		
		else if(Mpdev->Mst->RunState==1)
		{
			OriginalEncodedX1=YMoveZeroTrial(Mpdev);
			if(OriginalEncodedX1==1)
			{
					MoveZero(Mpdev,0,seepXMZ,TorqueXMZ,Mpdev->Mcg->ReturnZeroDirection[0],((Mpdev->Mcg->MotorXLACUNA)-(Mpdev->Mcg->MotorXYLACUNA)));
					Mpdev->Mst->RunState=2;
			}
			else if(OriginalEncodedX1==-1)
			{
				RestMotorSta(Mpdev);
				clearM(Mpdev);
				Mpdev->Mst->RunState=0;
				return -1;
			}
		}
	  else if(Mpdev->Mst->MoveZeroSta[0]&&(Mpdev->Mst->RunState==2))
	  {
          
			MorotSetEncodervalue(Mpdev,0,0); 			                                 //撞零成功设置编码器值0		
			Mpdev->Mst->RunState=3;	
	  }
	  else if(Mpdev->Mst->RunState==3)
		{
					MoveToTargetPos(Mpdev,seepXMZ,Mpdev->Mcg->XMoveStandard[15],0);
					Mpdev->Mst->RunState=4;
		}
		else if(Mpdev->Mst->RunState==4&&!Mpdev->Mst->MoveTargetSta[0]&&Mpdev->Mst->MoveTimeOut[0]>1000*3)
		{
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],0);
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],3);
				clearM(Mpdev);
				Mpdev->Mst->RunState=0;
				return -1;
		}
    else if(Mpdev->Mst->MoveTargetSta[0]&&Mpdev->Mst->RunState==4)
		{	
				MoveToTargetPos(Mpdev,seepXMZ,((Mpdev->Mcg->XMoveStandard[0])+ (Mpdev->Mcg->MotorXYLACUNA)),0);
				Mpdev->Mst->RunState=5;
		}
		else if(Mpdev->Mst->RunState==5&&!Mpdev->Mst->MoveTargetSta[0]&&Mpdev->Mst->MoveTimeOut[0]>1000*3)
		{
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],0);
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],3);
				clearM(Mpdev);
				Mpdev->Mst->RunState=0;
				return -1;
		}
		else if(Mpdev->Mst->RunState==5&&Mpdev->Mst->MoveTargetSta[0])
		{
				clearM(Mpdev);
				Mpdev->Mst->RunState=6;
		}
		else if(Mpdev->Mst->RunState==6)
		{
				OriginalEncodedX1= YMoveZero(Mpdev,TorqueYMZ,SeepYMZ);
				if(OriginalEncodedX1==1)
				{
						Mpdev->Mst->RunState=7;
					  MoveToTargetPos(Mpdev,seepXMZ,Mpdev->Mcg->XMoveStandard[0],0);
				}
				else if(OriginalEncodedX1==-1)
			 {
						clearM(Mpdev);
						Mpdev->Mst->RunState=0;
						return -1;
				}
		}
		else if (Mpdev->Mst->RunState==7&&Mpdev->Mst->MoveTargetSta[0])
		{
				clearM(Mpdev);
				if(!Mpdev->Mst->SenSorSta[0])
				{
					Mpdev->Mst->RunState=0;
					return 1;
				}
				else
					Mpdev->Mst->RunState=9;	
		}
		else if(Mpdev->Mst->RunState==9)
		{
			 ScissorsresetSta=ScissorsReset(Mpdev);
			 if(ScissorsresetSta==1)
			 {
						clearM(Mpdev);
						Mpdev->Mst->RunState=0;
						return 1;
			 }
			 else if(ScissorsresetSta==-1)
			{
						clearM(Mpdev);
						Mpdev->Mst->RunState=0;
						return -1;
			}
		}
		return 0;
}


/*******************************************************************************
剪刀出刀  非阻塞
@Mpk:电机属性句柄
@KnifeNumber:剪刀编号1-16
********************************************************************************/
signed char KnifeSelection2(MotorProperty* Mpk,const short KnifeNumber)
{
		signed   short  seepXMZ=800;
		signed   short  SeepYMZ=400;
		signed   short  TorqueYMZ=900;
	
	if((KnifeNumber<=0||KnifeNumber>16)||Mpk==NULL)
	{
		return -1;
	}
	if(Mpk->Mst->OutKnifeStep==0)
	{
		MoveToTargetPos(Mpk,seepXMZ,(Mpk->Mcg->XMoveStandard[KnifeNumber-1]),0);
		Mpk->Mst->OutKnifeStep=1;
	}
	
	else if(Mpk->Mst->OutKnifeStep==1&&(Mpk->Mst->MoveTargetSta[0])&&(Mpk->Mst->MoveTimeOut[0]<1000*6))
	{
		MoveSeepMode(Mpk,TorqueYMZ,SeepYMZ*-1,1);                                                                      //推出剪刀
		Mpk->Mst->OutKnifeStep=2;
	}
	
	else if(Mpk->Mst->OutKnifeStep==1&&(!Mpk->Mst->MoveTargetSta[0])&&(Mpk->Mst->MoveTimeOut[0]>1000*6))             //超时检测
	{
		EnableOrClearALarm(Mpk->Mcg->MotorID[0],0);
		EnableOrClearALarm(Mpk->Mcg->MotorID[0],3);
		Mpk->Mst->OutKnifeStep=0;
		clearM(Mpk);
		return -1;
	}
	else if(Mpk->Mst->OutKnifeStep==2&&Mpk->Mst->MoveTimeOut[1]>1000*0.3)
	{
		MoveSeepMode(Mpk,TorqueYMZ,0x00,1);  
		EnableOrClearALarm(Mpk->Mcg->MotorID[1],0);
		EnableOrClearALarm(Mpk->Mcg->MotorID[1],3);
		Mpk->Mst->OutKnifeStep=3;
	}	
	
	else if(Mpk->Mst->OutKnifeStep==3)
	{					 
		if(KnifeNumber==1)                                      //移出齿轮啮合
		{
			MoveToTargetPos(Mpk,seepXMZ+200,((Mpk->Mcg->XMoveStandard[KnifeNumber-1])+(Mpk->Mcg->MotorXYLACUNA)),0);  
		}
		else
		{
			MoveToTargetPos(Mpk,seepXMZ+200,((Mpk->Mcg->XMoveStandard[KnifeNumber-1])-(Mpk->Mcg->MotorXYLACUNA)),0);  
		}
		Mpk->Mst->OutKnifeStep=4;
  }
	else if(Mpk->Mst->OutKnifeStep==4&&Mpk->Mst->MoveTimeOut[0]>1000*0.1)
	{
		MoveToTargetPos(Mpk,0x255,Mpk->Mcg->MotorYLACUNA, 1);                            //移动至齿轮水平位
		Mpk->Mst->OutKnifeStep=5;
	}
	else if(Mpk->Mst->OutKnifeStep==5&&Mpk->Mst->MoveTargetSta[1]&&Mpk->Mst->MoveTimeOut[1]>1000*0.1)
	{
		Mpk->Mst->OutKnifeStep=0;
		return 1;	
	}
	return 0;

}
/**************************************************************************
剪刀收刀 非阻塞方式
@Mpc:电机属性句柄
@KnifeNumber:剪刀编号1-16
0:默认当前剪刀号
****************************************************************************/
signed char CloseKnife2(MotorProperty* Mpc,const short KnifeNumber)
{
		signed   short  seepXMZ=1200;
		signed   short  SeepYMZ=800;
		signed   short  TorqueYMZ=1000;

	
		if((KnifeNumber<=0||KnifeNumber>16)||Mpc==NULL)
		{
			return -1;
		}
		if(Mpc->Mst->CloseKnife==0)
		{
			if(KnifeNumber==1)
			{
					MoveToTargetPos(Mpc,seepXMZ,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])+(Mpc->Mcg->MotorXYLACUNA)),0);  
			}
			else
			{
					MoveToTargetPos(Mpc,seepXMZ,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])-(Mpc->Mcg->MotorXYLACUNA)),0); 
			}
			Mpc->Mst->CloseKnife=1;
	  }
		else if(Mpc->Mst->CloseKnife==1&&!Mpc->Mst->MoveTargetSta[0]&&Mpc->Mst->MoveTimeOut[0]>1000*6)                  //超时检测
		{
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],0);
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],3);
			Mpc->Mst->CloseKnife=0;
			clearM(Mpc);
			return -1;
		}		
		else if(Mpc->Mst->CloseKnife==1&&Mpc->Mst->MoveTargetSta[0])
		{
				MoveToTargetPos(Mpc,0x255,Mpc->Mcg->MotorYLACUNA, 1);                                            //移动至齿轮水平位
				Mpc->Mst->CloseKnife=2;
		}
		else if(Mpc->Mst->CloseKnife==2&&Mpc->Mst->MoveTargetSta[1]&&Mpc->Mst->MoveTimeOut[1]>=1000*0.1)
		{
			MoveToTargetPos(Mpc,seepXMZ, Mpc->Mcg->XMoveStandard[KnifeNumber-1],0);
			Mpc->Mst->CloseKnife=3;		
		}
		else if(Mpc->Mst->CloseKnife==3&&!Mpc->Mst->MoveTargetSta[0]&&Mpc->Mst->MoveTimeOut[0]>1000*6)                   //超时检测
		{
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],0);
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],3);
			Mpc->Mst->CloseKnife=0;
			clearM(Mpc);
			return -1;
		}				
		else if(Mpc->Mst->CloseKnife==3&&Mpc->Mst->MoveTargetSta[0])
		{
			MoveSeepMode(Mpc,TorqueYMZ,SeepYMZ,1);                                                                //收剪刀
			Mpc->Mst->CloseKnife=4;
		}
		
		else if(Mpc->Mst->CloseKnife==4&&Mpc->Mst->MoveTimeOut[1]>1000*0.3)                                              //时间到停止
		{
			Mpc->Mst->CloseKnife=5;
			MoveSeepMode(Mpc,TorqueYMZ,0x00,1);
			EnableOrClearALarm(Mpc->Mcg->MotorID[1],0);
			EnableOrClearALarm(Mpc->Mcg->MotorID[1],3);
		}
		else if(Mpc->Mst->CloseKnife==5)
		{
				if(KnifeNumber==1)                                                           //移出齿轮啮合
				{
						MoveToTargetPos(Mpc,seepXMZ+200,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])+(Mpc->Mcg->MotorXYLACUNA)),0);  
				}
				else
				{
						MoveToTargetPos(Mpc,seepXMZ+200,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])-(Mpc->Mcg->MotorXYLACUNA)),0); 
				}
				Mpc->Mst->CloseKnife=6;
		}
		else if(Mpc->Mst->CloseKnife==6&&Mpc->Mst->MoveTargetSta[0]&&Mpc->Mst->MoveTimeOut[0]>=1000*0.2)
		{
			MoveToTargetPos(Mpc,0x255,Mpc->Mcg->MotorYLACUNA, 1);                            //移动至齿轮水平位
			Mpc->Mst->CloseKnife=7;
		}
		else if(Mpc->Mst->CloseKnife==7&&Mpc->Mst->MoveTargetSta[1]&&Mpc->Mst->MoveTimeOut[1]>=1000*0.2)
		{
			Mpc->Mst->CloseKnife=0;
			clearM(Mpc);
			return 1;
		}			
		return 0;
}








