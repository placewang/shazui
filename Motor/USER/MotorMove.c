#include "MotorCan.h"

MotorExeSTa MExeSta={0,0,0,0,0,0,0};

/*****************************************************************
����ƶ�λ�ñ궨��
******************************************************************/
static void MotorMoveStandard(void)
{
	int i=0;
	signed int standardvalY[16]={4463,4418,4410,4498,4540,4476,4483,4555,4545,4537,4532,4506,4508,4497,4471,4520};
	signed int standardvalX[16]={0,3960,8140,12263,16343,20416,24560,28607,32697,36810,40957,45093,49183,53341,57257,61418};
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
static signed char ReadMotorOriginalEncodedVal(const unsigned short MID)
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
void MoveToTargetPos(const signed tarSeep,signed int valpos, const unsigned short MID)
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

/*
������λ����1
*/
void RestStep1(char *stepnum,char * Restnum,char *ifstat,signed short seepXMZ ,signed short seepYMZ)
{
		if(!MExeSta.MoveTargetStaY&&*stepnum==0)
		{
			MoveToTargetPos(seepYMZ,MRevBuff.YMoveStandard[*Restnum]-500, MotorYID);         //�Ƴ�����
			*stepnum=1;
		}
		else if(*stepnum==1&&(MExeSta.MoveTargetStaY||MExeSta.MoveTimeOutY>=100*30))
		{
			if(MExeSta.MoveTimeOutY>=100*20&&!MExeSta.MoveTargetStaY)                        //��ת��ʱ
			{
				EnableOrClearALarm(MotorYID,0);
				EnableOrClearALarm(MotorYID,3);
			}
			if((*Restnum)==0)
			{
				MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[*Restnum]+MotorXYLACUNA,MotorXID);//ƽ�Ƴ���������
			}
			else
			{
			
				MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[*Restnum]-MotorXYLACUNA,MotorXID);//ƽ�Ƴ���������
			}
			*stepnum=2;
		}
		else if(*stepnum==2&&MExeSta.MoveTargetStaX)
		{ 		
				ClearMStat(MotorXID,0);
				ClearMStat(MotorYID,0);
				*stepnum=0;
				*ifstat=1;
		}

}
/*
������λ����2
*/
void RestStep2(char *stepnum,char * Restnum,char *ifstat,signed short seepXMZ ,signed short seepYMZ)
{
		if(!MExeSta.MoveTargetStaY&&*stepnum==0)
		{
			MoveToTargetPos(seepYMZ,MotorYbackKnife,MotorYID);                                //�˶�������ص�λ��
			*stepnum=1;
		}
		else if(*stepnum==1&&MExeSta.MoveTargetStaY)
		{
			MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[*Restnum],MotorXID);               //ƽ�ƻس�������
			*stepnum=2;
		}
		else if(*stepnum==2&&MExeSta.MoveTargetStaX)
		{ 		
			MoveToTargetPos(seepYMZ,MotorYbackKnifeIN,MotorYID);                               //����ص���λλ��
			*stepnum=3;
		}
		else if(*stepnum==3&&(MExeSta.MoveTargetStaY||MExeSta.MoveTimeOutY>=1000*20))
		{
			if(MExeSta.MoveTimeOutY>=1000*20&&!MExeSta.MoveTargetStaY)                          //��ת��ʱ
			{
				EnableOrClearALarm(MotorYID,0);
				EnableOrClearALarm(MotorYID,3);
			}
			if((*Restnum)==0)
			{
					MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[*Restnum]+MotorXYLACUNA,MotorXID);  //ƽ�Ƴ���������
			}
			else
			{
					MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[*Restnum]-MotorXYLACUNA,MotorXID);  //ƽ�Ƴ���������
			}
			*stepnum=4;
		}
		else if(*stepnum==4&&MExeSta.MoveTimeOutX)
		{
				ClearMStat(MotorXID,0);
				ClearMStat(MotorYID,0);
				*stepnum=0;
				*ifstat=2;                                                                        //��һ��
		}

}
/*
������λ����3
*/
void RestStep3(char *stepnum,char * Restnum,char *ifstat,signed short seepXMZ ,signed short seepYMZ)
{


		if(!MExeSta.MoveTargetStaY&&*stepnum==0)
		{
			MoveToTargetPos(seepYMZ,0x14,MotorYID);                                          //�˶��س�ʼλ
			*stepnum=1;
		}
		else if(*stepnum==1&&(MExeSta.MoveTargetStaY||MExeSta.MoveTimeOutY>=100*20))
		{
			if(MExeSta.MoveTimeOutY>=100*20&&!MExeSta.MoveTargetStaY)                        //��ת��ʱ
			{
				EnableOrClearALarm(MotorYID,0);
				EnableOrClearALarm(MotorYID,3);
			}
			if((*Restnum)==0)
			{
				MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[*Restnum]+MotorXYLACUNA,MotorXID);              //���һ��
			}
			else
		 {
				(*Restnum)--;
				MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[*Restnum],MotorXID);                            //ƽ����һ�Ѽ���λ��
			}
			*stepnum=2;
		}
		else if(*stepnum==2&&MExeSta.MoveTargetStaX)
		{
				ClearMStat(MotorXID,0);
				ClearMStat(MotorYID,0);
				*stepnum=0;
				*ifstat=0;                                                                        //ѭ������step1 ֱ������ȫ����λ
		}

}

/**********************************************************************
������λ
***********************************************************************/
signed char ScissorsReset(void)
{
		signed short seepX=500;
		signed short seepY=200;
		static   char   ifstat=0;			
		static   char   Restnum=15;
		static   char   stepnum=0;
		if(!MotorINPUT)
		{ 
		  return 1;
		}
		switch (ifstat)
		{
					case  0 :
						RestStep1(&stepnum,&Restnum,&ifstat,seepX,seepY);
						break;
					case 1:
//						RestStep2(&stepnum,&Restnum,&ifstat,seepX,seepY);
						break;
					case 2:
//   					RestStep3(&stepnum,&Restnum,&ifstat,seepX,seepY);
						break;
		}
	return 0;
}

/************************************************************************
�豸����
**************************************************************************/
signed char DreMoveZero(void)
{ 	
		unsigned char   MotorInitVal[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		signed   short  seepXMZ=800;
		signed   short  TorqueXMZ=180;
		signed   short  seepYMZ=100;
		signed   short  TorqueYMZ=280;
		static signed   int    OriginalEncodedX1=0;
		static signed   int    OriginalEncodedX2=0;
		static   char   RunState=0;
		
		if(RunState==0)
		{
			MoveZero(MotorYID,seepYMZ,TorqueYMZ,0x01,0x04);
			ClearMStat(MotorXID,1);
			ClearMStat(MotorYID,1);
			ClearMStat(MotorXID,0);
			ClearMStat(MotorYID,0);				
			RunState=1;
		}		
		else if(MExeSta.MoveZeroStaY&&RunState==1)
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
			MoveZero(MotorYID,seepYMZ,TorqueYMZ,0x01,0x04);
			RunState=3;	
	  }
	  else if(MExeSta.MoveZeroStaY&&(MRevBuff.M1pos>=-10&&MRevBuff.M1pos<=10)&&RunState==3)
		{
			if(MExeSta.OriginalEncodedX==0)
			{ReadMotorOriginalEncodedVal(MotorXID);}                                             //���Ͷ�������ԭʼֵ
			if(MExeSta.OriginalEncodedX==2)
			{ 
				OriginalEncodedX1=MRevBuff.XoriginalEncodeval;                                     //���һ��ײ���ı�����ԭʼֵ 
				MoveZero(0x01,seepXMZ,TorqueXMZ,0x00,0xB0);                                       //����ײ�������г�0x110
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
				ClearMStat(MotorXID,1);
				ClearMStat(MotorYID,1);	
			}
			else
					return -1;	
		}
		else if(RunState==6||RunState==7||RunState==8)
		{
			if(MExeSta.MoveZeroStaY==0&&MExeSta.MoveTargetStaX==0&&RunState==6)
			{
					MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[15]-MotorXYLACUNA, MotorXID);
					MoveZero(MotorYID,seepYMZ,TorqueYMZ,MotorYID,0x04);
					RunState=7;
			}
			else if(MExeSta.MoveZeroStaY&&MExeSta.MoveTargetStaX&&RunState==7)
			{
					RunState=8;
					ClearMStat(MotorYID,1);	
				  ClearMStat(MotorXID,0);
					MoveToTargetPos(seepXMZ,MRevBuff.XMoveStandard[15], MotorXID);
					memset(MotorInitVal,0,sizeof(MotorInitVal));
					MotorInitVal[0]=MCmd.Parameter;
					MotorInitVal[1]=MotorYID;
					MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
					MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
					MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                            //ײ��ɹ����ñ�����ֵ0				
			}
			else if (RunState==8&&MExeSta.MoveTargetStaX)
			{
					ClearMStat(MotorXID,1);
					ClearMStat(MotorYID,1);	
				  ClearMStat(MotorXID,0);
				  ClearMStat(MotorYID,0);
					RunState=9;	
					return 1;
			}
		}
		else if(RunState==9)
		{
			 ScissorsReset();
		}
		return 0;
}

