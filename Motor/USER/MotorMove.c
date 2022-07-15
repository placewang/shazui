#include  "MotorCan.h"

MotorExeSTa MExeSta={0,0,0,0,0};

/*****************************************************************
电机移动位置标定量
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
初始化电机工作参数
*******************************************************************/
unsigned char MotorCanInit(void)
{
	
	unsigned char   MotorInitVal[8]={MCmd.Parameter,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	MotorMoveStandard();
	//初始化出刀电机
	MotorInitVal[1]=MotorYID;
  MotorInitVal[2]=MSCmd.SetDir&0xff; 
	MotorInitVal[3]=MSCmd.SetDir>>8;
	MotorInitVal[4]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置运动正方向（逆时针）
  MotorInitVal[2]=MSCmd.SetFindZeroMode&0xff;
	MotorInitVal[3]=MSCmd.SetFindZeroMode>>8;
	MotorInitVal[4]=0x00;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置以撞零方式回零
	MotorInitVal[2]=MSCmd.SetMaxSpeed&0xff;
	MotorInitVal[3]=MSCmd.SetMaxSpeed>>8;
	MotorInitVal[4]=0xFF;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置最高速度255RPM
	MotorInitVal[2]=MSCmd.SetReturnZeroOffset&0xff;
	MotorInitVal[3]=MSCmd.SetReturnZeroOffset>>8;
	MotorInitVal[4]=0x02;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //撞停后的偏移量		
	MotorInitVal[2]=MSCmd.ElectronicGearRatio&0xff;
	MotorInitVal[3]=MSCmd.ElectronicGearRatio>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[6]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //电子齿轮比2：1  4096个脉冲转一圈
	//初始化平移电机
	memset(&MotorInitVal[2],0,6);
	MotorInitVal[1]=MotorXID;
  MotorInitVal[2]=MSCmd.SetDir&0xff; 
	MotorInitVal[3]=MSCmd.SetDir>>8;
	MotorInitVal[4]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置运动正方向（逆时针）
  MotorInitVal[2]=MSCmd.SetFindZeroMode&0xff;
	MotorInitVal[3]=MSCmd.SetFindZeroMode>>8;
	MotorInitVal[4]=0x00;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置以撞零方式回零
	MotorInitVal[2]=MSCmd.SetMaxSpeed&0xff;
	MotorInitVal[3]=MSCmd.SetMaxSpeed>>8;
	MotorInitVal[4]=0xFF;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置最高速度255RPM
	MotorInitVal[2]=MSCmd.SetReturnZeroOffset&0xff;
	MotorInitVal[3]=MSCmd.SetReturnZeroOffset>>8;
	MotorInitVal[4]=0x10;
	MotorInitVal[5]=0x02;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //撞停后的偏移量		
	MotorInitVal[2]=MSCmd.ElectronicGearRatio&0xff;
	MotorInitVal[3]=MSCmd.ElectronicGearRatio>>8;
	MotorInitVal[4]=0x02;
	MotorInitVal[5]=0x00;
	MotorInitVal[6]=0x01;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //电子齿轮比2：1  4096个脉冲转一圈
	
	return 0;
}

/************************************************************************
电机回零（撞零方式）
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
移动到目标位置
@tarSeep :运动速度
@valpos  :目标位置
@MID     ：设备ID
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
		valpos= valpos<0?(valpos+0):(valpos-0);                          //卸掉机械硬力反向回退一点
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
清标志状态		
@MID:设备ID
zero: 非0只清楚回零标志
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
状态刷新
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
				MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                            //撞零成功设置编码器值
				MRevBuff.Stop2pos=0;
				MExeSta.MoveZeroStaY=1;			
		}
		else if(MRevBuff.Stop1pos==1)
		{
				MotorInitVal[0]=MCmd.Parameter;
				MotorInitVal[1]=0x01;
				MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
				MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
				MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                            //撞零成功设置编码器值
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





