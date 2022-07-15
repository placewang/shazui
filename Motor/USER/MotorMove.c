#include  "MotorCan.h"

MotorExeSTa MExeSta={0,0,0,0,0};

/*****************************************************************
����ƶ�λ�ñ궨��
******************************************************************/
static void MotorMoveStandard(void)
{
	int i=0;
	signed int standardvalX[16]={4463,4418,4410,4498,4540,4476,4483,4555,4545,4537,4532,4506,4508,4497,4471,4531};
	signed int standardvalY[16]={0,3960,8140,12263,16343,20416,24560,28607,32697,36810,40957,45093,49183,53341,57257,61418};
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
	MotorInitVal[2]=MSCmd.SetReturnZeroOffset&0xff;
	MotorInitVal[3]=MSCmd.SetReturnZeroOffset>>8;
	MotorInitVal[4]=0x02;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //ײͣ���ƫ����		
	MotorInitVal[2]=MSCmd.ElectronicGearRatio&0xff;
	MotorInitVal[3]=MSCmd.ElectronicGearRatio>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[6]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //���ӳ��ֱ�2��1  4096������תһȦ
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
	MotorInitVal[2]=MSCmd.SetReturnZeroOffset&0xff;
	MotorInitVal[3]=MSCmd.SetReturnZeroOffset>>8;
	MotorInitVal[4]=0x10;
	MotorInitVal[5]=0x02;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //ײͣ���ƫ����		
	MotorInitVal[2]=MSCmd.ElectronicGearRatio&0xff;
	MotorInitVal[3]=MSCmd.ElectronicGearRatio>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[5]=0x00;
	MotorInitVal[6]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //���ӳ��ֱ�2��1  4096������תһȦ
	
	return 0;
}

/************************************************************************
������㣨ײ�㷽ʽ��
**************************************************************************/
signed char MoveZero(const unsigned short MID,const signed short Seep,const unsigned short Torque)
{
		unsigned char   MotorInitVal[8]={0x00,MID,0x00,0x00,0x00,0x00,0x00,0x00};

		MotorInitVal[0]=MCmd.ZeroHit; 
		MotorInitVal[2]=Seep&0xff; 
		MotorInitVal[3]=Seep>>8;
		MotorInitVal[4]=Torque&0xff;
		MotorInitVal[5]=Torque>>8;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		MotorInitVal[0]=MCmd.MovezeroRev; 
		MotorInitVal[2]=0x01;
		MotorInitVal[3]=0x00;
		MotorInitVal[4]=0x00;
		MotorInitVal[5]=0x00;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
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
		MExeSta.MoveStart=1;
		unsigned char   MotorInitVal[8]={0x00,MID,0x00,0x00,0x00,0x00,0x00,0x00};
		MotorInitVal[0]=MCmd.MoveWthout;
		MotorInitVal[2]=tarSeep&0xff; 
		MotorInitVal[3]=tarSeep>>8;
		MotorInitVal[4]=valpos&0xff;
		MotorInitVal[5]=valpos>>8;
		MotorInitVal[6]=valpos>>16;
		MotorInitVal[7]=valpos>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		valpos= valpos<0?(valpos+0):(valpos-0);                          //ж����еӲ���������һ��
		MotorInitVal[4]=valpos&0xff;
		MotorInitVal[5]=valpos>>8;
		MotorInitVal[6]=valpos>>16;
		MotorInitVal[7]=valpos>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		if(MID==MotorYID)
		{
			MRevBuff.YMoveTargetval=valpos;
		}
		else if(MID==MotorXID)
		{	
			MRevBuff.XMoveTargetval=valpos;	
		}
}

/*
���־״̬		
@MID:�豸ID
zero: ��0ֻ��������־
*/

void ClearMStat(const unsigned short MID,char zero)
{
		if(MID==MotorYID&&zero==0)
		{
			MExeSta.MoveTargetStaY=0;	
			MRevBuff.YMoveTargetval=0;
			MExeSta.MoveStart=0;
		}
		else if (MID==MotorXID&&zero==0)
		{
			MExeSta.MoveTargetStaX=0;
			MExeSta.MoveStart=0;
			MRevBuff.XMoveTargetval=0;
		}
		else if(MID==MotorYID&&zero!=0)
		{
				MExeSta.MoveZeroStaY=0;	
		}
		else if (MID==MotorXID&&zero!=0)
		{
				MExeSta.MoveZeroStaX=0;	
		}
}	

/*
״̬ˢ��
*/

void  MotorStaRenew(void)
{
		unsigned char   MotorInitVal[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		if(MRevBuff.Stop2pos==1)
		{
			  MotorInitVal[0]=MCmd.Parameter;
				MotorInitVal[1]=0x02;
				MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
				MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
				MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                            //ײ��ɹ����ñ�����ֵ
				MRevBuff.Stop2pos=0;
				MExeSta.MoveZeroStaY=1;			
		}
		else if(MRevBuff.Stop1pos==1)
		{
				MotorInitVal[0]=MCmd.Parameter;
				MotorInitVal[1]=0x01;
				MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
				MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
				MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                            //ײ��ɹ����ñ�����ֵ
				MRevBuff.Stop1pos=0;
				MExeSta.MoveZeroStaX=1;
		}
		else if(MExeSta.MoveStart&&MRevBuff.M2pos>=(MRevBuff.YMoveTargetval-10)&&MRevBuff.M2pos<=(MRevBuff.YMoveTargetval+10))
		{
				MExeSta.MoveTargetStaY=1;
				MExeSta.MoveStart=0;
		}	
		else if(MExeSta.MoveStart&&MRevBuff.M1pos>=(MRevBuff.XMoveTargetval-10)&&MRevBuff.M1pos<=(MRevBuff.XMoveTargetval+10))
		{
				MExeSta.MoveTargetStaX=1;
				MExeSta.MoveStart=0;
		}
}	





