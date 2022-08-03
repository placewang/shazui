#include "MotorCan.h"

MotorExeSTa MExeSta={0,0,0,0,0,0,0};

/*****************************************************************
����ƶ�λ�ñ궨��
******************************************************************/
static void MotorMoveStandard(void)
{
	int i=0;
	signed int standardvalY[16]={4463,4418,4410,4498,4540,4476,4483,4555,4545,4537,4532,4506,4508,4497,4471,4520};
	signed int standardvalX[16]={10,3960,8140,12263,16343,20416,24560,28607,32697,36810,40957,45093,49183,53341,57257,61418};
	for(i=0;i<16;i++)
	{
		MRevBuff.XMoveStandard[i]=standardvalX[i];
	}
	for(i=0;i<16;i++)
	{
		MRevBuff.YMoveStandard[i]=standardvalY[i];
	}
}

/******************************************************************
��ʼ�������������
*******************************************************************/
unsigned char MotorCanInit(void)
{
	unsigned char   MotorInitVal[8]={MCmd.Parameter,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	MotorMoveStandard();
	//��ʼ���������
	MotorInitVal[1]=MotorYID;
  MotorInitVal[2]=MSCmd.SetDir&0xff; 
	MotorInitVal[3]=MSCmd.SetDir>>8;
	MotorInitVal[4]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //�����˶���������ʱ�룩
  MotorInitVal[2]=MSCmd.SetFindZeroMode&0xff;
	MotorInitVal[3]=MSCmd.SetFindZeroMode>>8;
	MotorInitVal[4]=0x00;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //������ײ�㷽ʽ����
	MotorInitVal[2]=MSCmd.SetMaxSpeed&0xff;
	MotorInitVal[3]=MSCmd.SetMaxSpeed>>8;
	MotorInitVal[4]=0xFF;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //��������ٶ�255RPM
	MotorInitVal[2]=MSCmd.ElectronicGearRatio&0xff;
	MotorInitVal[3]=MSCmd.ElectronicGearRatio>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[6]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //���ӳ��ֱ�2��1  4096������תһȦ
	MotorInitVal[2]=MSCmd.VersionSwitch&0xff;
	MotorInitVal[3]=MSCmd.VersionSwitch>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[6]=0x00;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //Э��汾�л�Ϊ��ǿ
	//��ʼ��ƽ�Ƶ��
	memset(&MotorInitVal[2],0,6);
	MotorInitVal[1]=MotorXID;
  MotorInitVal[2]=MSCmd.SetDir&0xff; 
	MotorInitVal[3]=MSCmd.SetDir>>8;
	MotorInitVal[4]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //�����˶���������ʱ�룩
  MotorInitVal[2]=MSCmd.SetFindZeroMode&0xff;
	MotorInitVal[3]=MSCmd.SetFindZeroMode>>8;
	MotorInitVal[4]=0x00;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //������ײ�㷽ʽ����
	MotorInitVal[2]=MSCmd.SetMaxSpeed&0xff;
	MotorInitVal[3]=MSCmd.SetMaxSpeed>>8;
	MotorInitVal[4]=0xFF;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //��������ٶ�255RPM	
	MotorInitVal[2]=MSCmd.ElectronicGearRatio&0xff;
	MotorInitVal[3]=MSCmd.ElectronicGearRatio>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[6]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //���ӳ��ֱ�2��1  4096������תһȦ
	MotorInitVal[2]=MSCmd.VersionSwitch&0xff;
	MotorInitVal[3]=MSCmd.VersionSwitch>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[6]=0x00;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //Э��汾�л�Ϊ��ǿ
	EnableOrClearALarm(MotorXID,3);
	EnableOrClearALarm(MotorYID,3);
	return 0;
}
/************************************************************************
��ԭʼ����ֵ
@MID�����ID
*************************************************************************/
signed char ReadMotorOriginalEncodedVal(const unsigned short MID)
{
	unsigned char   MotorInitVal[8]={MCmd.OriginalEncodedVal,MID,0x00,0x00,0x00,0x00,0x00,0x00};
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
	if(MID==MotorXID)
	{MExeSta.OriginalEncodedX=1;}
	else if(MID==MotorYID)
	{MExeSta.OriginalEncodedY=1;}
	return 0;
}	
/*********************************************************************
ʹ��/ʧ��/�屨��
MID�����ID
Ner: 1:��ʹ��
		 0:��ʹ��
		 3:�������		
************************************************************************/
signed char EnableOrClearALarm(const unsigned short MID,const unsigned char Ner)
{
		unsigned char   MotorInitVal[8]={MCmd.Enable ,MID,Ner,0x00,0x00,0x00,0x00,0x00};
		if(Ner==3)
		{
			MotorInitVal[0]=MCmd.ClearALarm;
			MotorInitVal[2]=0;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		}
		else if(Ner==0||Ner==1)
		{
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		}
		else
			return -1;
		return 0;
}

/************************************************************************
������㣨ײ�㷽ʽ��
@MID:      �豸ID
@Seep:     �����ٶ�
@Torque��  ����ת��
@Dir:      ���㷽��
@offsetval:ײ���ƫ����
**************************************************************************/
signed char MoveZero(const unsigned short MID,const signed short Seep,\
										 const unsigned short Torque,unsigned char Dir,signed int offsetval)
{
		if(MID==MotorYID)
		{
				ClearMStat(MotorYID,1);	
		}
		else if(MID==MotorXID)
		{
				ClearMStat(MotorXID,1);	
		}
		unsigned char   MotorInitVal[8]={0x00,MID,0x00,0x00,0x00,0x00,0x00,0x00};
		MotorInitVal[0]=MCmd.Parameter;
		MotorInitVal[2]=MSCmd.SetDir&0xff; 
		MotorInitVal[3]=MSCmd.SetDir>>8;	
		MotorInitVal[6]=Dir;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //�����˶���������ʱ�룩
		MotorInitVal[6]=0x00;	
		MotorInitVal[2]=MSCmd.SetReturnZeroOffset&0xff;
		MotorInitVal[3]=MSCmd.SetReturnZeroOffset>>8;
		MotorInitVal[4]=offsetval&0xff;
		MotorInitVal[5]=(offsetval>>8)&0xff;
		MotorInitVal[6]=(offsetval>>16)&0xff;
		MotorInitVal[7]=offsetval>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //ײͣ���ƫ����	
		MotorInitVal[0]=MCmd.ZeroHit; 
		MotorInitVal[2]=Seep&0xff; 
		MotorInitVal[3]=Seep>>8;
		MotorInitVal[4]=Torque&0xff;
		MotorInitVal[5]=Torque>>8;
		MotorInitVal[6]=0;
		MotorInitVal[7]=0;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		MotorInitVal[0]=MCmd.MovezeroRev; 
		MotorInitVal[2]=0x01;
		MotorInitVal[3]=0x00;
		MotorInitVal[4]=0x00;
		MotorInitVal[5]=0x00;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //���㷵��
		return 0;		
}
/*****************************************************************************
�ƶ���Ŀ��λ��
@tarSeep :�˶��ٶ�
@valpos  :Ŀ��λ��
@MID     ���豸ID
********************************************************************************/
void MoveToTargetPos(const signed short tarSeep,signed int valpos, const unsigned short MID)
{
		unsigned char   MotorInitVal[8]={0x00,MID,0x00,0x00,0x00,0x00,0x00,0x00};
		if(MID==MotorYID)
		{
			MExeSta.MoveTimeOutY=0;
			MRevBuff.YMoveTargetval=valpos;
			MExeSta.MoveTargetStaY=0;	
		}
		else if(MID==MotorXID)
		{	
			MExeSta.MoveTimeOutX=0;
			MRevBuff.XMoveTargetval=valpos;
			MExeSta.MoveTargetStaX=0;		
		}
		MotorInitVal[0]=MCmd.MoveWith;
		MotorInitVal[2]=tarSeep&0xff; 
		MotorInitVal[3]=tarSeep>>8;
		MotorInitVal[4]=valpos&0xff;
		MotorInitVal[5]=(valpos>>8)&0xff;
		MotorInitVal[6]=(valpos>>16)&0xff;
		MotorInitVal[7]=valpos>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
}

/********************************************************************************
�ٶ�ģʽ���˶�
@torque:�˶�Ť��
@tarSeep:�˶��ٶ�
@MID��   �豸ID
********************************************************************************/
signed char MoveSeepMode(const signed short torque,signed int tarSeep, const unsigned short MID)
{
		unsigned char   MotorInitVal[8]={0x00,MID,0x00,0x00,0x00,0x00,0x00,0x00};
		if(MID==MotorYID)
		{
			MExeSta.MoveTimeOutY=0;	
		}
		else if(MID==MotorXID)
		{	
			MExeSta.MoveTimeOutX=0;		
		}
		MotorInitVal[0]=MCmd.SeepMode;
		MotorInitVal[2]=torque&0xff; 
		MotorInitVal[3]=torque>>8;
		MotorInitVal[4]=tarSeep&0xff;
		MotorInitVal[5]=(tarSeep>>8)&0xff;
		MotorInitVal[6]=(tarSeep>>16)&0xff;
		MotorInitVal[7]=tarSeep>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		return 0;
}

/********************************************************************************
Y�ᣨ������������㺯��--�����ⲿ��紫����
����X���ƶ�һ�μ�϶��
@torque:�˶�Ť��
@tarSeep:�˶��ٶ�
@MID��   �豸ID
********************************************************************************/
signed char YMoveZero(const signed short torque,signed int tarSeep, const unsigned short MID)
{
		unsigned char   MotorInitVal[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		static signed char step=0;
		if(step==0)
		{
				MoveSeepMode(torque,tarSeep,MID);
				if(MotorZeroIN)
				{
					step=1;
				}
				else
				{
					step=3;
				}
		}
		else if((step==1||step==2)&&!MotorZeroIN)
		{
			MoveSeepMode(torque,0x00,MID);                                            //ͣ���
			MotorInitVal[0]=MCmd.Parameter;
			MotorInitVal[1]=MotorYID;
			MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
			MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                //���ñ�����ֵ0					
			MoveToTargetPos(0x55,MotorYLACUNA, MotorYID);                             //�ƶ�������ˮƽλ
			step=4;
		}
		else if(step==4&&MExeSta.MoveTargetStaY)
		{
			step=0;
			return 1;
		}
		else if(step==3)
		{
			if(MotorZeroIN)
			{
				step =2;
			}
		}
		else if(step>0&&MExeSta.MoveTimeOutY>=100*40)                              //��ʱ����4S
		{
			MoveSeepMode(torque,0x00,MID);     
			EnableOrClearALarm(MotorYID,0);
			EnableOrClearALarm(MotorYID,3);
			step=0;
			return -1;
		}	
		return 0;
}

/*
������λ����1
*/
void RestStep1(char *stepnum,char * Restnum,char *ifstat,signed short seepXMZ ,signed short seepYMZ,const signed short torque)
{
		if(*stepnum==0)
		{
			MoveSeepMode(torque,seepYMZ*-1,MotorYID);         //�Ƴ�����
			*stepnum=1;
		}
		else if(*stepnum==1&&MExeSta.MoveTimeOutY>=100*4)
		{
				MoveSeepMode(torque,0x00,MotorYID);  
				if(*Restnum==15)
				{
					MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[*Restnum]-MotorXYLACUNA,MotorXID);                             //�˶��س�ʼλ
				}
				else
				{
					MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[*Restnum]+MotorXYLACUNA,MotorXID);                             //�˶��س�ʼλ
				}	
				EnableOrClearALarm(MotorYID,0);
				EnableOrClearALarm(MotorYID,3);
				*stepnum=2;
		}
		else if(*stepnum==2&&MExeSta.MoveTargetStaX)
		{
			MoveToTargetPos(0x155,MotorYLACUNA, MotorYID);
			*stepnum=3;
		}
		else if(*stepnum==3&&MExeSta.MoveTargetStaY)
		{
			MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[*Restnum],MotorXID);                             //�˶��س�ʼλ
			*stepnum=4;
		}
		
		else if(*stepnum==4&&MExeSta.MoveTargetStaX)
		{
			*stepnum=0;
			*ifstat=1;
		}
}
/*
������λ����2
*/
void RestStep2(char *stepnum,char * Restnum,char *ifstat,signed short seepXMZ ,signed short seepYMZ,const signed short torque)
{
		if(*stepnum==0)
		{
			MoveSeepMode(torque,seepYMZ,MotorYID);         //�ռ���
			*stepnum=1;
		}
		else if(*stepnum==1&&MExeSta.MoveTimeOutY>=100*4)
		{
				EnableOrClearALarm(MotorYID,0);
				EnableOrClearALarm(MotorYID,3);
				if(*Restnum==15||!MotorINPUT)
				{
					if(*Restnum==0&&!MotorINPUT)
					{
						MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[*Restnum]+MotorXYLACUNA,MotorXID);  
					}
					else
					{
						MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[*Restnum]-MotorXYLACUNA,MotorXID); 					
					}
					DELAY_Ms(200);
					MoveToTargetPos(0x355,MotorYLACUNA, MotorYID);                                              //�ƶ�������ˮƽλ	
					DELAY_Ms(100);
					*ifstat=3;
				}
				else
				{
					MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[*Restnum]+MotorXYLACUNA,MotorXID);                             //�˶��س�ʼλ
				}	
				*stepnum=2;
		}
		else if(*stepnum==2&&MExeSta.MoveTargetStaX)
		{
			MoveToTargetPos(0x155,MotorYLACUNA, MotorYID);                            //�ƶ�������ˮƽλ	
			*stepnum=3;
		}
		else if(*stepnum==3&&MExeSta.MoveTargetStaY)
		{
				*stepnum=0;
			  *ifstat=2;
		}
}
/*
������λ����3
*/
void RestStep3(char *stepnum,char * Restnum,char *ifstat,signed short seepXMZ ,signed short seepYMZ)
{
		if(*stepnum==0)
		{
			MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[*Restnum+1],MotorXID);                             //��һλ
			(*Restnum)++;
			*stepnum=1;
		}
		else if(*stepnum==1&&(MExeSta.MoveTargetStaX||MExeSta.MoveTimeOutY>=100*20))
		{
			if(MExeSta.MoveTimeOutY>=100*20&&!MExeSta.MoveTargetStaY)                                        //��ת��ʱ
			{
				EnableOrClearALarm(MotorYID,0);
				EnableOrClearALarm(MotorYID,3);
			}
			*stepnum=0;                                                                                      //ѭ������step1 ֱ������ȫ����λ
			*ifstat=0;
		}	
}

/**********************************************************************
������λ
***********************************************************************/
signed char ScissorsReset(void)
{
		signed short seepX=1800;
		signed short seepY=8000;
	  signed short torqueY=500;
		static   char   ifstat=0;			
		static   char   Restnum=0;
		static   char   stepnum=0;

		switch (ifstat)
		{
					case  0 :
						RestStep1(&stepnum,&Restnum,&ifstat,seepX,seepY,torqueY);
						break;
					case 1:
						RestStep2(&stepnum,&Restnum,&ifstat,seepX,seepY,torqueY);
						break;
					case 2:
   					RestStep3(&stepnum,&Restnum,&ifstat,seepX,seepY);
						break;
					case 3:
					if(!MotorINPUT)
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
/************************************************************
����״̬
*************************************************************/
void clearM(void)
{

		ClearMStat(MotorXID,1);
		ClearMStat(MotorYID,1);
		ClearMStat(MotorXID,0);
		ClearMStat(MotorYID,0);	
}

/************************************************************************
�豸����
**************************************************************************/
signed char DreMoveZero(void)
{ 	
		unsigned char   MotorInitVal[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		signed   short  seepXMZ=800;
		signed   short  TorqueXMZ=180;
		signed   short  SeepYMZ=2800;
		signed   short  TorqueYMZ=280;
		static signed   int    OriginalEncodedX1=0;
		static signed   int    OriginalEncodedX2=0;
		static   char   RunState=0;
		
		if(RunState==0)
		{
				clearM();
				RunState=1;
		}		
		else if(RunState==1)
		{
			MoveZero(MotorXID,seepXMZ,TorqueXMZ,0x01,0x210);
			RunState=2;
		}
	  else if(MExeSta.MoveZeroStaX&&RunState==2)
	  {
			MotorInitVal[0]=MCmd.Parameter;
			MotorInitVal[1]=MotorXID;
			MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
			MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                            //ײ��ɹ����ñ�����ֵ0		
			RunState=3;	
	  }
	  else if((MRevBuff.M1pos>=-10&&MRevBuff.M1pos<=10)&&RunState==3)
		{
			if(MExeSta.OriginalEncodedX==0)
			{ReadMotorOriginalEncodedVal(MotorXID);}                                             //���Ͷ�������ԭʼֵ
			if(MExeSta.OriginalEncodedX==2)
			{ 
				OriginalEncodedX1=MRevBuff.XoriginalEncodeval;                                     //���һ��ײ���ı�����ԭʼֵ 
				MoveZero(0x01,seepXMZ,TorqueXMZ,0x00,0xB0);                                        //����ײ�������г�0x110
				RunState=4;
			}				
		}
		
    else if(MExeSta.MoveZeroStaX&&RunState==4)
		{	
			if(MExeSta.OriginalEncodedX==0)
			{ReadMotorOriginalEncodedVal(MotorXID);}                                             //���Ͷ�������ԭʼֵ
		  if(MExeSta.OriginalEncodedX==2)
			{ 
				OriginalEncodedX2=MRevBuff.XoriginalEncodeval;                                      //��ڶ���ײ���ı�����ԭʼֵ 
				RunState=5;
			}	
		}
		else if(RunState==5)
		{
			//�����г�
			OriginalEncodedX1=OriginalEncodedX2-OriginalEncodedX1<0?OriginalEncodedX1-OriginalEncodedX2:OriginalEncodedX2-OriginalEncodedX1;
			OriginalEncodedX2=OriginalEncodedX1/32*2;
			if(OriginalEncodedX2+600>=61400&&OriginalEncodedX2-600<=61400)
			{
				memset(MotorInitVal,0,sizeof(MotorInitVal));
				MotorInitVal[0]=MCmd.Parameter;
				MotorInitVal[1]=MotorXID;
				MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
				MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
				MotorInitVal[4]=OriginalEncodedX2&0xff;
				MotorInitVal[5]=(OriginalEncodedX2>>8)&0xff;
				MotorInitVal[6]=(OriginalEncodedX2>>16)&0xff;
				MotorInitVal[7]=OriginalEncodedX2>>24;				
				MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                             //ײ��ɹ����ñ�����ֵ
				MRevBuff.XMoveStandard[15]=OriginalEncodedX2;
				RunState=6;
				clearM();
			}
			else
			{
				 clearM();
				 RunState=0;
				 return -1;
			}				
		}
		else if(RunState==6||RunState==7||RunState==8)
		{
			if(MExeSta.MoveTargetStaX==0&&RunState==6)
			{
					MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[0]+MotorXYLACUNA, MotorXID);
					RunState=7;
			}
			else if(MExeSta.MoveTargetStaX&&RunState==7)
			{
					if(YMoveZero(TorqueYMZ,SeepYMZ,MotorYID)==1)
					{
						RunState=8;
						MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[0],MotorXID);
						ClearMStat(MotorXID,0);
					}
					else if(YMoveZero(TorqueYMZ,SeepYMZ,MotorYID)==-1)
					{
						clearM();
						RunState=0;
						return -1;
					}
			}
			else if (RunState==8&&MExeSta.MoveTargetStaX)
			{
				clearM();
				if(!MotorINPUT)
				{
					RunState=0;
					return 1;
				}
				else
					RunState=9;	
			}
		}
		else if(RunState==9)
		{
			 if(ScissorsReset())
			 {
						clearM();
						RunState=0;
						return 1;
			 }
		}
		return 0;
}
/*******************************************************************************
��������
@KnifeNum:�������1-16
********************************************************************************/
signed char KnifeSelection(const short KnifeNum)
{
		signed   short  seepXMZ=800;
		signed   short  TorqueXMZ=180;
		signed   short  SeepYMZ=2800;
		signed   short  TorqueYMZ=280;
	
	if(KnifeNum<=0||KnifeNum>16)
	{
		return -1;
	}
	if(MotorINPUT)
	{
			MoveSeepMode(TorqueYMZ,SeepYMZ,MotorYID);         //�ռ���
			while(MExeSta.MoveTimeOutY<1000*0.7)
			{
			}
			MoveSeepMode(TorqueYMZ,0x00,MotorYID);
			EnableOrClearALarm(MotorYID,0);
			EnableOrClearALarm(MotorYID,3);			
			if(KnifeNum==1)
			{
				MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]+MotorXYLACUNA,MotorXID);  
			}
			else
			{
					MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]-MotorXYLACUNA,MotorXID); 
			}
			MoveToTargetPos(0x155,MotorYLACUNA, MotorYID);                            //�ƶ�������ˮƽλ
			DELAY_Ms(100);
	}	
	MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[KnifeNum-1], MotorXID);
	while(!MExeSta.MoveTargetStaX&&MExeSta.MoveTimeOutX<1000*6)
	{
		ReadAnPackData(&MRevBuff);
	}
	if(MExeSta.MoveTimeOutX>1000*6)
	{
		MoveSeepMode(TorqueYMZ,0x00,MotorYID);
		return -1;
	}
	MoveSeepMode(TorqueYMZ,SeepYMZ*-1,MotorYID);         //�Ƴ�����
	while(MExeSta.MoveTimeOutY<1000*1)
	{
	}
	MoveSeepMode(TorqueYMZ,0x00,MotorYID);
	EnableOrClearALarm(MotorYID,0);
	EnableOrClearALarm(MotorYID,3);	
	if(KnifeNum==1)
	{
		MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]+MotorXYLACUNA,MotorXID);  
	}
	else
	{
			MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]-MotorXYLACUNA,MotorXID); 
	}
	DELAY_Ms(100);
	MoveToTargetPos(0x155,MotorYLACUNA, MotorYID);                            //�ƶ�������ˮƽλ
	DELAY_Ms(100);
	return 1;	

}

/**************************************************************************
�����յ�
@KnifeNum:�������1-16
0:Ĭ�ϵ�ǰ������

****************************************************************************/


signed char CloseKnife(const short KnifeNum)
{
		signed   short  seepXMZ=800;
		signed   short  TorqueXMZ=180;
		signed   short  SeepYMZ=2800;
		signed   short  TorqueYMZ=280;
		if(!MotorINPUT||(KnifeNum<=0||KnifeNum>16))
		{
			return -1;
		}
		if(KnifeNum==1)
		{
				MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]+MotorXYLACUNA,MotorXID);  
		}
		else
		{
					MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]-MotorXYLACUNA,MotorXID); 
		}
		while(!MExeSta.MoveTargetStaX)
		{
				ReadAnPackData(&MRevBuff);
		}
		MoveToTargetPos(0x155,MotorYLACUNA, MotorYID);                            //�ƶ�������ˮƽλ
		DELAY_Ms(100);	
		MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1],MotorXID); 
		while(!MExeSta.MoveTargetStaX)
		{
			ReadAnPackData(&MRevBuff);
		}
		MoveSeepMode(TorqueYMZ,SeepYMZ,MotorYID);                                  //�ռ���
		while(MExeSta.MoveTimeOutY<1000*0.7)
		{
		}
		MoveSeepMode(TorqueYMZ,0x00,MotorYID);
		EnableOrClearALarm(MotorYID,0);
		EnableOrClearALarm(MotorYID,3);
		if(KnifeNum==1)
		{
				MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]+MotorXYLACUNA,MotorXID);  
		}
		else
		{
					MoveToTargetPos(seepXMZ, MRevBuff.XMoveStandard[KnifeNum-1]-MotorXYLACUNA,MotorXID); 
		}
		DELAY_Ms(100);
		MoveToTargetPos(0x155,MotorYLACUNA, MotorYID);                            //�ƶ�������ˮƽλ
		return 1;	
}
