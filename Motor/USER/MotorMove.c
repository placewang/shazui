#include "MotorCan.h"

//电机报警
MotorAlarm   MErrState={0};                                                                             
MotorAlarm   RMErrState={0};                                                                             

//电机状态
MotorExeSTa MExeSta ={0,0,0,0,0,0,0,0,0,{0},{0},{0},{0},{0}};
MotorExeSTa RMExeSta={0,0,0,0,0,0,0,0,0,{0},{0},{0},{0},{0}};

//左模块电机移动位置标定量
const  signed int standardvalY_L[KnifeNum]={4463,4418,4410,4498,4540,4476,4483,4555,4545,4537,4532,4506,4508,4497,4471,4520};
const signed int standardvalX_L[KnifeNum]={10,3960,8140,12263,16343,20416,24560,28607,32697,36810,40957,45093,49183,53341,57257,61418};

//右模块电机移动位置标定量
const	signed int standardvalY_R[KnifeNum]={4463,4418,4410,4498,4540,4476,4483,4555,4545,4537,4532,4506,4508,4497,4471,4520};
const	signed int standardvalX_R[KnifeNum]={10,3960,8140,12263,16343,20416,24560,28607,32697,36810,40957,45093,49183,53341,57257,61418};
//{1271,5440,9440,13563,17543,21416,25660,29707,39917,37910,42057,46073,50183,54378,58527,62541};

//电机属性配置
MotorConfig  MtCgL={{0x00,0x00},2000,-1800,0xc00,{0x00,2500},{0x00,0x01},{0x00,0x01},{0xff,0xff},{0x01681000,0x01681000},standardvalY_L,standardvalX_L,{0x02,0x08}};
MotorConfig  MtCgR={{0x00,0x01},2000,-1800,0xc00,{0x00,2500},{0x00,0x00},{0x00,0x01},{0xff,0xff},{0x01681000,0x01681000},standardvalY_R,standardvalX_R,{0x04,0x62}};

//左电机属性句柄
MotorProperty  MtProperty1_L={&MtCgL,&MErrState,&MExeSta};
//左电机属性句柄
MotorProperty  MtProperty1_R={&MtCgR,&RMErrState,&RMExeSta};


/*
使能/失能/清报警
MID：电机ID
Ner: 1:上使能
		 0:下使能
		 3:清除报警		
*/
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

/*
复位模块状态
@Mp:电机属性句柄
*/
signed char RestMotorSta(MotorProperty* Ms)
{
	 EnableOrClearALarm(Ms->Mcg->MotorID[0],0);
	 EnableOrClearALarm(Ms->Mcg->MotorID[0],3);
	 EnableOrClearALarm(Ms->Mcg->MotorID[1],0);
	 EnableOrClearALarm(Ms->Mcg->MotorID[1],3);			
	 return 0;
}
/*	
清标志状态		
@Mp:电机属性句柄
@St:模块状态
@MID:设备ID(0/1:X/Y轴)
@zero 
				0:清运动状态
				1:清回零标志
				3:清报警		
*/
signed char ClearMStat(MotorProperty* Mp,const unsigned short MID,char zero)
{
		if(Mp==NULL||(MID!=0&&MID!=1))
		{
				return -1;
		}
		if(zero==0)
		{
			 Mp->Mst->MoveTargetSta[MID]=0;	
		}
		else if(zero==1)
		{
			Mp->Mst->MoveZeroSta[MID]=0;
		}
		else if(zero==3)
		{
			Mp->Mer->MotorErrID[MID]     =0;
			Mp->Mer->MotorEorrStat[MID]  =0;
			Mp->Mer->MotorAlarmGrade[MID]=0;
			return 1;
		}
		else
		{
			return -1;
		}
		return 0;
}	
/*
清电机模块状态
@Mm:电机属性句柄
*/
void clearM(MotorProperty* Mm)
{
		ClearMStat(Mm,0,1);
		ClearMStat(Mm,1,1);
		ClearMStat(Mm,0,0);
		ClearMStat(Mm,1,0);	
}
/************************************************************************
电机回零（撞零方式）
@Mpy:      电机属性句柄
@MID:      设备ID(0/1:X/Y轴)
@Seep:     回零速度
@Torque：  回零转矩
@Dir:      回零方向
@offsetval:撞零后偏移量
**************************************************************************/
signed char MoveZero(MotorProperty* Mpy,const unsigned short MID,const signed short Seep,\
										 const unsigned short Torque,unsigned char Dir,signed int offsetval)
{
		if(Mpy==NULL||(MID!=0&&MID!=1))
		{
				return -1;
		}
		ClearMStat(Mpy,MID,1);
		unsigned char   MotorInitVal[8]={0};
		Mpy->Mst->MoveTimeOut[MID]=0;
		MotorInitVal[0]=MCmd.Parameter;
		MotorInitVal[1]=Mpy->Mcg->MotorID[MID];
		MotorInitVal[2]=MSCmd.SetDir&0xff; 
		MotorInitVal[3]=MSCmd.SetDir>>8;
		MotorInitVal[4]=(Mpy->Mcg->MoveMentDirection[MID])&0xff;		
		MotorInitVal[6]=Dir;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置运动正方向（逆时针）
		MotorInitVal[2]=MSCmd.SetReturnZeroOffset&0xff;
		MotorInitVal[3]=MSCmd.SetReturnZeroOffset>>8;
		MotorInitVal[4]=offsetval&0xff;
		MotorInitVal[5]=(offsetval>>8)&0xff;
		MotorInitVal[6]=(offsetval>>16)&0xff;
		MotorInitVal[7]=offsetval>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //撞停后的偏移量	
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
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //回零返回
		return 1;		
}
/*****************************************************************************
移动到目标位置
@Mpt:      电机属性句柄
@MID:      设备ID(0/1:X/Y轴)
@tarSeep :运动速度
@valpos  :目标位置
********************************************************************************/
signed char MoveToTargetPos(MotorProperty* Mpt,const signed short tarSeep,signed int valpos, const unsigned int MID)
{
		unsigned char   MotorInitVal[8]={0x00};
		if(Mpt==NULL||(MID!=0&&MID!=1))
		{
				return -1;
		}
		Mpt->Mst->MoveTimeOut[MID]=0;
		ClearMStat(Mpt,MID,0);
		MotorInitVal[0]=MCmd.MoveWith;
		MotorInitVal[1]=Mpt->Mcg->MotorID[MID];
		MotorInitVal[2]=tarSeep&0xff; 
		MotorInitVal[3]=tarSeep>>8;
		MotorInitVal[4]=valpos&0xff;
		MotorInitVal[5]=(valpos>>8)&0xff;
		MotorInitVal[6]=(valpos>>16)&0xff;
		MotorInitVal[7]=valpos>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
}

/********************************************************************************
速度模式下运动
@Mpv:      电机属性句柄
@MID:      设备ID(0/1:X/Y轴)
@torque:运动扭矩
@tarSeep:运动速度
********************************************************************************/
signed char MoveSeepMode(MotorProperty* Mpv,const signed short torque,signed int tarSeep, const unsigned int MID)
{
		unsigned char   MotorInitVal[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		if(Mpv==NULL||(MID!=0&&MID!=1))
		{
				return -1;
		}
		Mpv->Mst->MoveTimeOut[MID]=0;
		MotorInitVal[0]=MCmd.SeepMode;
		MotorInitVal[1]=Mpv->Mcg->MotorID[MID];
		MotorInitVal[2]=torque&0xff; 
		MotorInitVal[3]=torque>>8;
		MotorInitVal[4]=tarSeep&0xff;
		MotorInitVal[5]=(tarSeep>>8)&0xff;
		MotorInitVal[6]=(tarSeep>>16)&0xff;
		MotorInitVal[7]=tarSeep>>24;
		MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
		return 0;
}
/*
设置编码器值
@Mpos:     电机属性句柄
@MID:      设备ID(0/1:X/Y轴)
*/
signed char MorotSetEncodervalue(MotorProperty* Mpos,const unsigned short MID,const signed int posval)
{
	unsigned char   MotorInitVal[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	if(Mpos==NULL||(MID!=0&&MID!=1))
	{
			return -1;
	}
	MotorInitVal[0]=MCmd.Parameter;
	MotorInitVal[1]=Mpos->Mcg->MotorID[MID];
	MotorInitVal[2]=MSCmd.SetEncodervalue&0xff;
	MotorInitVal[3]=MSCmd.SetEncodervalue>>8;
	MotorInitVal[4]=(posval)&0xff;
	MotorInitVal[5]=((posval)>>8)&0xff;
	MotorInitVal[6]=((posval)>>16)&0xff;
	MotorInitVal[7]=((posval)>>24)&0xff;	
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);      
	return 0;
}
/******************************************************************
初始化电机工作参数
@Mpi:电机属性句柄
*******************************************************************/
unsigned char MotorCanInit(MotorProperty* Mpi)
{
	unsigned char MotorInitVal[8]={MCmd.Parameter,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	char i=0;
	if(Mpi==NULL)
	{
		return 0;
	}
	for(i=0;i<2;i++)
	{
			memset(&MotorInitVal[2],0,6);
			MotorInitVal[1]=Mpi->Mcg->MotorID[i];
			MotorInitVal[2]=MSCmd.VersionSwitch&0xff;
			MotorInitVal[3]=MSCmd.VersionSwitch>>8;
			MotorInitVal[4]=0x02;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //协议版本切换为恒强			
			MotorInitVal[2]=MSCmd.SetDir&0xff; 
			MotorInitVal[3]=MSCmd.SetDir>>8;
			MotorInitVal[4]=(Mpi->Mcg->MoveMentDirection[i])&0xff;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置运动正方向（逆时针）
			MotorInitVal[2]=MSCmd.SetFindZeroMode&0xff;
			MotorInitVal[3]=MSCmd.SetFindZeroMode>>8;
			MotorInitVal[4]=(Mpi->Mcg->MoveToZero[i])&0xff;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置以撞零方式回零
			MotorInitVal[2]=MSCmd.SetMaxSpeed&0xff;
			MotorInitVal[3]=MSCmd.SetMaxSpeed>>8;
			MotorInitVal[4]=(Mpi->Mcg->TopSpeed[i])&0xff;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);                  //设置最高速度255RPM
			MotorInitVal[2]=MSCmd.ElectronicGearRatio&0xff;
			MotorInitVal[3]=MSCmd.ElectronicGearRatio>>8;
			MotorInitVal[4]=(Mpi->Mcg->ElectronicGearRatio[i])&0xff;
			MotorInitVal[5]=((Mpi->Mcg->ElectronicGearRatio[i])>>8)&0xff;
			MotorInitVal[6]=((Mpi->Mcg->ElectronicGearRatio[i])>>16)&0xff;
			MotorInitVal[7]=((Mpi->Mcg->ElectronicGearRatio[i])>>24)&0xff;
			MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);			         //电子齿轮比  4096个脉冲转一圈
		
			EnableOrClearALarm(Mpi->Mcg->MotorID[i],3);
	}
	return 1;
}


