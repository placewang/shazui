#include "MotorCan.h"


/********************************************************************************
Y�ᣨ������������㺯��--�����ⲿ��紫����
����X���ƶ�һ�μ�϶��
@Mpy:   ������Ծ��
@torque:�˶�Ť��
@tarSeep:�˶��ٶ�
@MID��   �豸ID
********************************************************************************/
signed char YMoveZero(MotorProperty* Mpy,const signed short torque,signed int tarSeep)
{
		static signed char step=0;
		if(step==0)
		{
				MoveSeepMode(Mpy,torque,tarSeep,1);
				if(Mpy->Mst->SenSorSta[1])
				{
					step=1;
				}
				else
				{
					step=3;
				}
		}
		else if((step==1||step==2)&&!Mpy->Mst->SenSorSta[1])
		{
			MoveSeepMode(Mpy,torque,0x00,1);                                              //ͣ���
      MorotSetEncodervalue(Mpy,1,0);                                                //���ñ�����ֵ0					
			MoveToTargetPos(Mpy,0x255,Mpy->Mcg->MotorYLACUNA,1);                          //�ƶ�������ˮƽλ
			step=4;
		}
		else if(step==4&&Mpy->Mst->MoveTargetSta[1])
		{
			step=0;
			return 1;
		}
		else if(step==3)
		{
			if(Mpy->Mst->SenSorSta[1])
			{
				step =2;
			}
		}
		else if(step>0&&Mpy->Mst->MoveTimeOut[1]>=100*30)                              //��ʱ����4S
		{
			MoveSeepMode(Mpy,torque,0x00,1); 
			EnableOrClearALarm(Mpy->Mcg->MotorID[1],0);
			EnableOrClearALarm(Mpy->Mcg->MotorID[1],3);
			step=0;
			return -1;
		}	
		return 0;
}


/******************************************************************
Y�����Դ�
@Mptry:   ������Ծ��
����ϵ��������������λ���٣�ѭ��6���ƶ�����λ�ô����٣�
********************************************************************/

signed char YMoveZeroTrial(MotorProperty* Mptry)
{
			signed   short  SeepYMZ=300;
			signed   short  TorqueYMZ=980;
			signed   short  seepXMZ=500;
			signed   short  TorqueXMZ=300;
			
			static signed   char   TrialState=0;
			static signed   char   TrialCount =0;
			if(Mptry==NULL)
			{
				return -1;
			}		
			if(TrialCount==12)
			{
				TrialState=0;
				TrialCount =0;
				return -1;
			}

			if(TrialState==0)
			{
				 MoveZero(Mptry,0,seepXMZ,TorqueXMZ,Mptry->Mcg->ReturnZeroDirection[0],Mptry->Mcg->MotorXLACUNA);
				 TrialState=2;
			}
			else if(TrialState==2&&!Mptry->Mst->MoveZeroSta[0]&&Mptry->Mst->MoveTimeOut[0]>100*10*2)
			{
				EnableOrClearALarm(Mptry->Mcg->MotorID[0],0);
				EnableOrClearALarm(Mptry->Mcg->MotorID[0],3);
				TrialState=-1;
			}
			else if(TrialState==2&&Mptry->Mst->MoveZeroSta[0])
			{	
				TrialState=YMoveZero(Mptry,TorqueYMZ,SeepYMZ);
				if(TrialState==0)
				{
					TrialState=2;
				}
			}	
		
			else if(TrialState==-1)
			{
				MoveZero(Mptry,0x00,seepXMZ,TorqueXMZ,Mptry->Mcg->ReturnZeroDirection[0],0x280);
				TrialState=3;
			}
			
			else if(TrialState==3&&Mptry->Mst->MoveZeroSta[0])
			{
					MoveSeepMode(Mptry,0x800,0x30,1);
					TrialState=4;
			}
			else if(TrialState==4&&Mptry->Mst->MoveTimeOut[1]>100*8)
			{
					MoveSeepMode(Mptry,0x100,0x00,1);
					TrialCount++;
					TrialState=0;
			}
			else if(TrialState==1)
			{
				TrialState=0;
				TrialCount =0;
				return 1;
			}				


	return 0;
}

/*
������λ����1
@Mpr1:������Ծ��
*/
signed char RestStep1(MotorProperty* Mpr1,char *stepnum,char * Restnum,char *ifstat,\
											signed short seepXMZ ,signed short seepYMZ,const signed short torque)
{
		if(Mpr1==NULL||stepnum==NULL||Restnum==NULL||ifstat==NULL)
		{
				return -1;
		}
		if(*stepnum==0)
		{
			MoveSeepMode(Mpr1,torque,seepYMZ*-1,1);         //�Ƴ�����
			*stepnum=1;
		}
		else if(*stepnum==1&&Mpr1->Mst->MoveTimeOut[1]>=100*1.5)
		{
				MoveSeepMode(Mpr1,torque,0x00,1);  
				if(*Restnum==15)
				{
					MoveToTargetPos(Mpr1,seepXMZ,Mpr1->Mcg->XMoveStandard[*Restnum]-Mpr1->Mcg->MotorXYLACUNA,0);                             //�˶��س�ʼλ
				}
				else
				{
					MoveToTargetPos(Mpr1,seepXMZ,Mpr1->Mcg->XMoveStandard[*Restnum]+Mpr1->Mcg->MotorXYLACUNA,0);                             //�˶��س�ʼλ
				}	
				*stepnum=2;
		}
		else if(*stepnum==2&&Mpr1->Mst->MoveTargetSta[0])
		{
			MoveToTargetPos(Mpr1,0x155,Mpr1->Mcg->MotorYLACUNA,1);
			*stepnum=3;
		}
		else if(*stepnum==3&&Mpr1->Mst->MoveTargetSta[1])
		{
			MoveToTargetPos(Mpr1,seepXMZ,Mpr1->Mcg->XMoveStandard[*Restnum],0);                             //�˶��س�ʼλ
			*stepnum=4;
		}
		
		else if(*stepnum==4&&Mpr1->Mst->MoveTargetSta[0])
		{
			*stepnum=0;
			*ifstat=1;
			return 1;
		}
		return 0;
}

/*
������λ����2
@Mpr2:������Ծ��
*/
signed char RestStep2(MotorProperty* Mpr2,char *stepnum,char * Restnum,char *ifstat,\
								signed short seepXMZ ,signed short seepYMZ,const signed short torque)
{
		if(Mpr2==NULL||stepnum==NULL||Restnum==NULL||ifstat==NULL)
		{
				return -1;
		}
		if(*stepnum==0)
		{
			MoveSeepMode(Mpr2,torque,seepYMZ,1);         //�ռ���
			*stepnum=1;
		}
		
		else if(*stepnum==1&&Mpr2->Mst->MoveTimeOut[1]>=100*1.5)
		{
					MoveSeepMode(Mpr2,torque,0x00,1); 
			
				if(*Restnum==15||!Mpr2->Mst->SenSorSta[0])
				{
					if(*Restnum==0&&!Mpr2->Mst->SenSorSta[0])
					{
						MoveToTargetPos(Mpr2,seepXMZ, Mpr2->Mcg->XMoveStandard[*Restnum]+Mpr2->Mcg->MotorXYLACUNA,0);  
					}
					else
					{
						MoveToTargetPos(Mpr2,seepXMZ, Mpr2->Mcg->XMoveStandard[*Restnum]-Mpr2->Mcg->MotorXYLACUNA,0); 					
					}
					MoveToTargetPos(Mpr2,0x255,Mpr2->Mcg->MotorYLACUNA,1);                                              //�ƶ�������ˮƽλ	
					*stepnum=4;
				}
				else
				{
					MoveToTargetPos(Mpr2,seepXMZ, Mpr2->Mcg->XMoveStandard[*Restnum]+Mpr2->Mcg->MotorXYLACUNA,0);        
					*stepnum=2;
				}	
				
		}
		else if(*stepnum==2&&Mpr2->Mst->MoveTargetSta[0])
		{
			MoveToTargetPos(Mpr2,0x255,Mpr2->Mcg->MotorYLACUNA, 1);                                                   //�ƶ�������ˮƽλ	
			*stepnum=3;
		}

		else if(*stepnum==4&&Mpr2->Mst->MoveTargetSta[1]&&Mpr2->Mst->MoveTimeOut[1]>=100*1.5)
		{
			  *ifstat=3;
		}		
		else if(*stepnum==3&&Mpr2->Mst->MoveTargetSta[1]&&Mpr2->Mst->MoveTimeOut[1]>=100*1.3)
		{
				*stepnum=0;
			  *ifstat=2;
		}
		return 0;
}

/*
������λ����3
@Mpr3:������Ծ��
*/
signed char RestStep3(MotorProperty* Mpr3,char *stepnum,char * Restnum,char *ifstat,\
								signed short seepXMZ ,signed short seepYMZ)
{
		if(Mpr3==NULL||stepnum==NULL||Restnum==NULL||ifstat==NULL)
		{
				return -1;
		}	
		if(*stepnum==0)
		{
			MoveToTargetPos(Mpr3,seepXMZ, Mpr3->Mcg->XMoveStandard[*Restnum+1],0);                             //��һλ
			(*Restnum)++;
			*stepnum=1;
		}
		else if(*stepnum==1&&(Mpr3->Mst->MoveTargetSta[0]||Mpr3->Mst->MoveTimeOut[1]>=100*20))
		{
			if(Mpr3->Mst->MoveTimeOut[1]>=100*20&&!Mpr3->Mst->MoveTargetSta[1])                                        //��ת��ʱ
			{
				EnableOrClearALarm(Mpr3->Mcg->MotorID[1],0);
				EnableOrClearALarm(Mpr3->Mcg->MotorID[1],3);
			}
			*stepnum=0;                                                                                      //ѭ������step1 ֱ������ȫ����λ
			*ifstat=0;
		}	
		return 0;
}



/**********************************************************************
������λ
@Mps:������Ծ��
***********************************************************************/
signed char ScissorsReset(MotorProperty* Mps)
{
		signed short seepX=2000;
		signed short seepY=500;
	  signed short torqueY=900;
		static   char   ifstat=0;			
		static   char   Restnum=0;
		static   char   stepnum=0;

		switch (ifstat)
		{
					case  0 :
						RestStep1(Mps,&stepnum,&Restnum,&ifstat,seepX,seepY,torqueY);
						break;
					case 1:
						RestStep2(Mps,&stepnum,&Restnum,&ifstat,seepX,seepY,torqueY);
						break;
					case 2:
   					RestStep3(Mps,&stepnum,&Restnum,&ifstat,seepX,seepY);
						break;
					case 3:
					if(!Mps->Mst->SenSorSta[0])
					{ 
							ifstat=0;			
							Restnum=0;
							stepnum=0;
							return 1;
					}
					break;
		}
	return 0;
}
/************************************************************************
�豸����
@Mpdev:������Ծ��
**************************************************************************/
signed char DreMoveZero(MotorProperty* Mpdev)
{ 	
		signed   short  seepXMZ=800;
		signed   short  TorqueXMZ=930;
		signed   short  SeepYMZ=400;
		signed   short  TorqueYMZ=980;
	  signed   char   OriginalEncodedX1=0;
		static   char   RunState=0;
		if(Mpdev==NULL)
		{
			return -1;
		}
		if(RunState==0)
		{
				clearM(Mpdev);
				RunState=1;
		}		
		else if(RunState==1)
		{
			OriginalEncodedX1=YMoveZeroTrial(Mpdev);
			if(OriginalEncodedX1==1)
			{
					MoveZero(Mpdev,0,seepXMZ,TorqueXMZ,Mpdev->Mcg->ReturnZeroDirection[0],((Mpdev->Mcg->MotorXLACUNA)-(Mpdev->Mcg->MotorXYLACUNA)));
					RunState=2;
			}
			else if(OriginalEncodedX1==-1)
			{
				RestMotorSta(Mpdev);
				clearM(Mpdev);
				RunState=0;
				return -1;
			}
		}
	  else if(Mpdev->Mst->MoveZeroSta[0]&&(RunState==2))
	  {
          
			MorotSetEncodervalue(Mpdev,0,0); 			                                 //ײ��ɹ����ñ�����ֵ0		
			RunState=3;	
	  }
	  else if(RunState==3)
		{
					MoveToTargetPos(Mpdev,seepXMZ,Mpdev->Mcg->XMoveStandard[15],0);
					RunState=4;
		}
		else if(RunState==4&&!Mpdev->Mst->MoveTargetSta[0]&&Mpdev->Mst->MoveTimeOut[0]>1000*3)
		{
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],0);
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],3);
				clearM(Mpdev);
				RunState=0;
				return -1;
		}
    else if(Mpdev->Mst->MoveTargetSta[0]&&RunState==4)
		{	
				MoveToTargetPos(Mpdev,seepXMZ,((Mpdev->Mcg->XMoveStandard[0])+ (Mpdev->Mcg->MotorXYLACUNA)),0);
				RunState=5;
		}
		else if(RunState==5&&!Mpdev->Mst->MoveTargetSta[0]&&Mpdev->Mst->MoveTimeOut[0]>1000*3)
		{
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],0);
				EnableOrClearALarm(Mpdev->Mcg->MotorID[0],3);
				clearM(Mpdev);
				RunState=0;
				return -1;
		}
		else if(RunState==5&&Mpdev->Mst->MoveTargetSta[0])
		{
				clearM(Mpdev);
				RunState=6;
		}
		else if(RunState==6)
		{
				OriginalEncodedX1= YMoveZero(Mpdev,TorqueYMZ,SeepYMZ);
				if(OriginalEncodedX1==1)
				{
						RunState=7;
					  MoveToTargetPos(Mpdev,seepXMZ,Mpdev->Mcg->XMoveStandard[0],0);
				}
				else if(OriginalEncodedX1==-1)
			 {
						clearM(Mpdev);
						RunState=0;
						return -1;
				}
		}
		else if (RunState==7&&Mpdev->Mst->MoveTargetSta[0])
		{
				clearM(Mpdev);
				if(!Mpdev->Mst->SenSorSta[0])
				{
					RunState=0;
					return 1;
				}
				else
					RunState=9;	
		}
		else if(RunState==9)
		{
			 if(ScissorsReset(Mpdev))
			 {
						clearM(Mpdev);
						RunState=0;
						return 1;
			 }
		}
		return 0;
}


/*******************************************************************************
��������  ������
@Mpk:������Ծ��
@KnifeNumber:�������1-16
********************************************************************************/
signed char KnifeSelection2(MotorProperty* Mpk,const short KnifeNumber)
{
		signed   short  seepXMZ=800;
		signed   short  SeepYMZ=400;
		signed   short  TorqueYMZ=900;
		static   char   OutKnifeStep=0;
	if((KnifeNumber<=0||KnifeNumber>16)||Mpk==NULL)
	{
		return -1;
	}
	if(OutKnifeStep==0)
	{
		MoveToTargetPos(Mpk,seepXMZ,(Mpk->Mcg->XMoveStandard[KnifeNumber-1]),0);
		OutKnifeStep=1;
	}
	
	else if(OutKnifeStep==1&&(Mpk->Mst->MoveTargetSta[0])&&(Mpk->Mst->MoveTimeOut[0]<1000*6))
	{
		MoveSeepMode(Mpk,TorqueYMZ,SeepYMZ*-1,1);                             //�Ƴ�����
		OutKnifeStep=2;
	}
	
	else if(OutKnifeStep==1&&(!Mpk->Mst->MoveTargetSta[0])&&(Mpk->Mst->MoveTimeOut[0]>1000*6))                      //��ʱ���
	{
		EnableOrClearALarm(Mpk->Mcg->MotorID[0],0);
		EnableOrClearALarm(Mpk->Mcg->MotorID[0],3);
		OutKnifeStep=0;
		clearM(Mpk);
		return -1;
	}
	else if(OutKnifeStep==2&&Mpk->Mst->MoveTimeOut[1]>1000*0.3)
	{
		MoveSeepMode(Mpk,TorqueYMZ,0x00,1);  
		EnableOrClearALarm(Mpk->Mcg->MotorID[1],0);
		EnableOrClearALarm(Mpk->Mcg->MotorID[1],3);
		OutKnifeStep=3;
	}	
	
	else if(OutKnifeStep==3)
	{					 
		if(KnifeNumber==1)                                      //�Ƴ���������
		{
			MoveToTargetPos(Mpk,seepXMZ+200,((Mpk->Mcg->XMoveStandard[KnifeNumber-1])+(Mpk->Mcg->MotorXYLACUNA)),0);  
		}
		else
		{
			MoveToTargetPos(Mpk,seepXMZ+200,((Mpk->Mcg->XMoveStandard[KnifeNumber-1])-(Mpk->Mcg->MotorXYLACUNA)),0);  
		}
		OutKnifeStep=4;
  }
	else if(OutKnifeStep==4&&Mpk->Mst->MoveTimeOut[0]>1000*0.1)
	{
		MoveToTargetPos(Mpk,0x255,Mpk->Mcg->MotorYLACUNA, 1);                            //�ƶ�������ˮƽλ
		OutKnifeStep=5;
	}
	else if(OutKnifeStep==5&&Mpk->Mst->MoveTargetSta[1]&&Mpk->Mst->MoveTimeOut[1]>1000*0.1)
	{
		OutKnifeStep=0;
		return 1;	
	}
	return 0;

}
/**************************************************************************
�����յ� ��������ʽ
@Mpc:������Ծ��
@KnifeNumber:�������1-16
0:Ĭ�ϵ�ǰ������
****************************************************************************/
signed char CloseKnife2(MotorProperty* Mpc,const short KnifeNumber)
{
		signed   short  seepXMZ=1200;
		signed   short  SeepYMZ=800;
		signed   short  TorqueYMZ=1000;
		static   char   CloseKnife=0;
	
		if((KnifeNumber<=0||KnifeNumber>16)||Mpc==NULL)
		{
			return -1;
		}
		if(CloseKnife==0)
		{
			if(KnifeNumber==1)
			{
					MoveToTargetPos(Mpc,seepXMZ,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])+(Mpc->Mcg->MotorXYLACUNA)),0);  
			}
			else
			{
					MoveToTargetPos(Mpc,seepXMZ,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])-(Mpc->Mcg->MotorXYLACUNA)),0); 
			}
			CloseKnife=1;
	  }
		else if(CloseKnife==1&&!Mpc->Mst->MoveTargetSta[0]&&Mpc->Mst->MoveTimeOut[0]>1000*6)                  //��ʱ���
		{
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],0);
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],3);
			CloseKnife=0;
			clearM(Mpc);
			return -1;
		}		
		else if(CloseKnife==1&&Mpc->Mst->MoveTargetSta[0])
		{
				MoveToTargetPos(Mpc,0x255,Mpc->Mcg->MotorYLACUNA, 1);                                            //�ƶ�������ˮƽλ
				CloseKnife=2;
		}
		else if(CloseKnife==2&&Mpc->Mst->MoveTargetSta[1]&&Mpc->Mst->MoveTimeOut[1]>=1000*0.1)
		{
			MoveToTargetPos(Mpc,seepXMZ, Mpc->Mcg->XMoveStandard[KnifeNumber-1],0);
			CloseKnife=3;		
		}
		else if(CloseKnife==3&&!Mpc->Mst->MoveTargetSta[0]&&Mpc->Mst->MoveTimeOut[0]>1000*6)                   //��ʱ���
		{
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],0);
			EnableOrClearALarm(Mpc->Mcg->MotorID[0],3);
			CloseKnife=0;
			clearM(Mpc);
			return -1;
		}				
		else if(CloseKnife==3&&Mpc->Mst->MoveTargetSta[0])
		{
			MoveSeepMode(Mpc,TorqueYMZ,SeepYMZ,1);                                                                //�ռ���
			CloseKnife=4;
		}
		
		else if(CloseKnife==4&&Mpc->Mst->MoveTimeOut[1]>1000*0.3)                                              //ʱ�䵽ֹͣ
		{
			CloseKnife=5;
			MoveSeepMode(Mpc,TorqueYMZ,0x00,1);
			EnableOrClearALarm(Mpc->Mcg->MotorID[1],0);
			EnableOrClearALarm(Mpc->Mcg->MotorID[1],3);
		}
		else if(CloseKnife==5)
		{
				if(KnifeNumber==1)                                                           //�Ƴ���������
				{
						MoveToTargetPos(Mpc,seepXMZ+200,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])+(Mpc->Mcg->MotorXYLACUNA)),0);  
				}
				else
				{
						MoveToTargetPos(Mpc,seepXMZ+200,((Mpc->Mcg->XMoveStandard[KnifeNumber-1])-(Mpc->Mcg->MotorXYLACUNA)),0); 
				}
				CloseKnife=6;
		}
		else if(CloseKnife==6&&Mpc->Mst->MoveTargetSta[0]&&Mpc->Mst->MoveTimeOut[0]>=1000*0.2)
		{
			MoveToTargetPos(Mpc,0x255,Mpc->Mcg->MotorYLACUNA, 1);                            //�ƶ�������ˮƽλ
			CloseKnife=7;
		}
		else if(CloseKnife==7&&Mpc->Mst->MoveTargetSta[1]&&Mpc->Mst->MoveTimeOut[1]>=1000*0.2)
		{
			CloseKnife=0;
			clearM(Mpc);
			return 1;
		}			
		return 0;
}








