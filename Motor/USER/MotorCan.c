#include  "MotorCan.h"
#include  "can.h"
                                                                               
MotorCmd     MCmd={0x01,0x02,0x07,0x0A,0x8A,0x71,0x72,0x12,0x76,0x75,0x08,0x78,0x73,0x0B};
MotorSubCmd	 MSCmd={0x8001,0x8002,0x8003,0x8004,0x8009,0x800B,0x8008,0x800C,0x8040,0x8041, 0x0201,0x8005,0x800e};
MotorRevBuff MRevBuff={0,0,0,0,{0},MRevbuffLen,DeQueue,EnQueue};




/*
判断缓存区是否满
*/
int MRevbuffFull(MotorRevBuff *buff)
{
	return (buff->subscript+1)%MOTORBUFFLEN==buff->out;
}


/*
判断缓存区是否空
*/
int MRevbuffEmpty(MotorRevBuff *buff)
{
	return buff->subscript==buff->out;
}


/*
计算缓存队列长度
*/
unsigned int MRevbuffLen(MotorRevBuff *buff)
{
	return (buff->subscript-buff->out+MOTORBUFFLEN )%MOTORBUFFLEN;
}

/*
入队
*/
int EnQueue(MotorRevBuff* q, MotorCanRevBuff *data){
    if(MRevbuffFull(q)){
        return False;
    }
    // 存
    q ->RevBuff[q ->subscript] = *data;
    // 下一包
    q -> subscript = (q -> subscript+1 )%MOTORBUFFLEN;
    return True;
}


/*
出队 
*/
int DeQueue(MotorRevBuff* q, MotorCanRevBuff *val){
    if(MRevbuffEmpty(q)){
        return False;
    }
    // 出
    *val = q->RevBuff[q ->out];
    // 下一包
    q->out = (q ->out+ 1)%MOTORBUFFLEN;
    return True;
}


/****************************************************************************
	Can数据发送接口/若移植变更 Send_CANFrame(&McanPeliTxBuff)硬件接口函数;
	dataval:要发送的数据
	num:    数据大小或者长度
	Dev_num:设备接收CANID  
*****************************************************************************/
signed char MotorSendCanData(const unsigned char * dataval,\
														 const unsigned int    num,\
														 const unsigned int    Dev_num)
{
	CanTxMsg  McanPeliTxBuff;
	unsigned char i;
	if(dataval==NULL||num==0)
	{
		return -1;
	}
//	ClearMStat(dataval[1] ,3);
	while(CAN_GetFlagStatus(CAN1,CAN_STATUS_TS)){;}                    //等待总线空闲
	while(CAN_GetFlagStatus(CAN1,CAN_STATUS_RS)){;}	
	while(!CAN_GetFlagStatus(CAN1,CAN_STATUS_TCS)){;}		
	McanPeliTxBuff.CANID     =Dev_num;                                 //Can发送ID
	McanPeliTxBuff.DLC       =num;                                     //数据长度
	McanPeliTxBuff.CANIDtype =0;                                       //ID类型（标准帧/扩展帧）
	McanPeliTxBuff.RTR       =0;                                       //帧类型（远程/数据） 	
	for(i=0;i<num;i++)
	{
		McanPeliTxBuff.Data[i]=dataval[i];	
	}
	Send_CANFrame(&McanPeliTxBuff);	
	return 0;
}
/*****************************************************************
接收数据包解析	
@Dpack :要解析的数据
******************************************************************/
signed char DataAnalyze(const unsigned char *Dpack)
{
	if(Dpack==NULL)
	{
		 return -1;
	}
	//报错报文
	
	if(Dpack[0]==0xF0&&Dpack[1]==MtCgL.MotorID[0])
	{
		MErrState.MotorErrID[0]     =Dpack[1];
		MErrState.MotorEorrStat[0]  =Dpack[2];
		MErrState.MotorAlarmGrade[0]=Dpack[3];
		EnableOrClearALarm(MtCgL.MotorID[0],3);
	}
	else if(Dpack[0]==0xF0&&Dpack[1]==MtCgL.MotorID[1])
	{
		MErrState.MotorErrID[1]     =Dpack[1];
		MErrState.MotorEorrStat[1]  =Dpack[2];
		MErrState.MotorAlarmGrade[1]=Dpack[3];
		EnableOrClearALarm(MtCgL.MotorID[1],3);
	}	
	if(Dpack[0]==0xF0&&Dpack[1]==MtCgR.MotorID[0])
	{
		RMErrState.MotorErrID[0]     =Dpack[1];
		RMErrState.MotorEorrStat[0]  =Dpack[2];
		RMErrState.MotorAlarmGrade[0]=Dpack[3];
		EnableOrClearALarm(MtCgR.MotorID[0],3);
	}
	else if(Dpack[0]==0xF0&&Dpack[1]==MtCgR.MotorID[1])
	{
		RMErrState.MotorErrID[1]     =Dpack[1];
		RMErrState.MotorEorrStat[1]  =Dpack[2];
		RMErrState.MotorAlarmGrade[1]=Dpack[3];
		EnableOrClearALarm(MtCgR.MotorID[1],3);
	}		
	//编码器/传感器返回值
	else if(Dpack[0]==0x71&&Dpack[1]==MtCgL.MotorID[0])
	{
			MExeSta.Mpos[0]=Dpack[4];
			MExeSta.Mpos[0]|=Dpack[5]<<8;
			MExeSta.Mpos[0]|=Dpack[6]<<16;
			MExeSta.Mpos[0]|=Dpack[7]<<24;
			MExeSta.SenSorSta[0] =(0x20&Dpack[3])>>5;                               
	}
	else if(Dpack[0]==0x71&&Dpack[1]==MtCgL.MotorID[1])
	{
			MExeSta.Mpos[1]=Dpack[4];
			MExeSta.Mpos[1]|=Dpack[5]<<8;
			MExeSta.Mpos[1]|=Dpack[6]<<16;
			MExeSta.Mpos[1]|=Dpack[7]<<24;
			MExeSta.SenSorSta[1] =(0x20&Dpack[3])>>5;                               
	}	
	else if(Dpack[0]==0x71&&Dpack[1]==MtCgR.MotorID[0])
	{
			RMExeSta.Mpos[0]=Dpack[4];
			RMExeSta.Mpos[0]|=Dpack[5]<<8;
			RMExeSta.Mpos[0]|=Dpack[6]<<16;
			RMExeSta.Mpos[0]|=Dpack[7]<<24;
			RMExeSta.SenSorSta[0] =(0x20&Dpack[3])>>5;                               
	}
	else if(Dpack[0]==0x71&&Dpack[1]==MtCgR.MotorID[1])
	{
			RMExeSta.Mpos[1]=Dpack[4];
			RMExeSta.Mpos[1]|=Dpack[5]<<8;
			RMExeSta.Mpos[1]|=Dpack[6]<<16;
			RMExeSta.Mpos[1]|=Dpack[7]<<24;
			RMExeSta.SenSorSta[1] =(0x20&Dpack[3])>>5;                               
	}		
  //运动到位返回	
	else if(Dpack[0]==0x15&&Dpack[1]==MtCgL.MotorID[0])
	{
			MExeSta.MoveTargetSta[0]=1;	
	}
	else if(Dpack[0]==0x15&&Dpack[1]==MtCgL.MotorID[1])
	{
			MExeSta.MoveTargetSta[1]=1;
	}
	else if(Dpack[0]==0x15&&Dpack[1]==MtCgR.MotorID[0])
	{
			RMExeSta.MoveTargetSta[0]=1;	
	}
	else if(Dpack[0]==0x15&&Dpack[1]==MtCgR.MotorID[1])
	{
			RMExeSta.MoveTargetSta[1]=1;
	}
	//归零完成返回
	else if(Dpack[0]==0x78&&Dpack[1]==MtCgL.MotorID[0]&&Dpack[2]==0x01&&Dpack[3]==0)
	{
			MExeSta.MoveZeroSta[0]=1;	
	}
	else if(Dpack[0]==0x78&&Dpack[1]==MtCgL.MotorID[1]&&Dpack[2]==0x01&&Dpack[3]==0)
	{
			MExeSta.MoveZeroSta[1]=1;	
	}	
	else if(Dpack[0]==0x78&&Dpack[1]==MtCgR.MotorID[0]&&Dpack[2]==0x01&&Dpack[3]==0)
	{
			RMExeSta.MoveZeroSta[0]=1;	
	}
	else if(Dpack[0]==0x78&&Dpack[1]==MtCgR.MotorID[1]&&Dpack[2]==0x01&&Dpack[3]==0)
	{
			RMExeSta.MoveZeroSta[1]=1;	
	}	
	
	return 1;
}

/***********************************************************************
获取设备返回数据（电机返回数据字节统一有无用数据，长度>=4做过滤）
@MotorRevBuff:缓存结构
@Data:返回数据包缓存
***********************************************************************/
signed char ReadAnPackData(MotorRevBuff *buff)
{	
	MotorCanRevBuff PData;
	if(buff==NULL)
	{
		return -1;
	}
	if(buff->MLen(buff)>0)
	{
			buff->DeQueue(buff,&PData);
			if(PData.DLC>=4)
			{
				 if(DataAnalyze(PData.Data))
				 {
						return 1;
				 }
		  }	
	}
	return 0;
}

/**********************************************************************
获取电机当前转速，转矩，位置
@ID：设备ID
**********************************************************************/
signed char GetMotorSpeed_Torque_Pos(const unsigned int  ID)
{
	unsigned char   MotorInitVal[8]={MCmd.RotatingSpeed,ID,0x00,0x00,0x00,0x00,0x00,0x00};
//	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
//	MotorInitVal[0]=MCmd.Torque;
//	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
	MotorInitVal[0]=MCmd.GetEncodervalue;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
	return 0;
}
/*
轮询查电机位置,到位传感器状态
*/

void PollingMotorSta(void)
{
//左模块	
	GetMotorSpeed_Torque_Pos(MtCgL.MotorID[0]);
	GetMotorSpeed_Torque_Pos(MtCgL.MotorID[1]);
//右模块	
	GetMotorSpeed_Torque_Pos(MtCgR.MotorID[0]);
	GetMotorSpeed_Torque_Pos(MtCgR.MotorID[1]);

}


















