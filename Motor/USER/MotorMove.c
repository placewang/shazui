#include  "MotorCan.h"

MotorExeSTa MExeSta={0,0,0,0,0};
/******************************************************************
��ʼ�������������
*******************************************************************/
unsigned char MotorCanInit(void)
{
	unsigned char   MotorInitVal[8]={MCmd.Parameter,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
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
void MoveToTargetPos(const signed tarSeep,const signed int valpos, const unsigned short MID)
{
		MExeSta.MoveStart=1;
		MRevBuff.YMoveTargetval=valpos;	
		unsigned char   MotorInitVal[8]={0x00,MID,0x00,0x00,0x00,0x00,0x00,0x00};
		MotorInitVal[0]=MCmd.MoveWthout;
		MotorInitVal[2]=tarSeep&0xff; 
		MotorInitVal[3]=tarSeep>>8;
		MotorInitVal[4]=valpos&0xff;
		MotorInitVal[5]=valpos>>8;
		MotorInitVal[6]=valpos>>16;
		MotorInitVal[7]=valpos>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
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





