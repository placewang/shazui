#include  "MotorCan.h"
#include  "can.h"
                                                                               
MotorCmd     MCmd={0x01,0x02,0x07,0x0A,0x8A,0x71,0x72,0x12,0x76,0x75,0x08,0x78};
MotorSubCmd	 MSCmd={0x8001,0x8002,0x8003,0x8004,0x8009,0x800B,0x8008,0x800C,0x8040,0x8041, 0x0201,0x8005};

MotorRevBuff MRevBuff={0,0,0,{0},0,0,0,0,0,0,0,0,0,0,0,{0},{0},MRevbuffFull,MRevbuffEmpty,MRevbuffLen};
MotorAlarm   MEState ={1,16,17,18,19,20,21,22,33,34,35,41,36,0,0,0};

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
	if(Dpack!=NULL)
	{
			if(Dpack[0]==0xF0)
			{
				MEState.MotorErrID     =Dpack[1];
				MEState.MotorEorrStat  =Dpack[2];
				MEState.MotorAlarmGrade=Dpack[3];
			}
			else if(Dpack[0]==0x75&&Dpack[1]==MotorXID)
			{
				MRevBuff.M1Seep=Dpack[4];
				MRevBuff.M1Seep|=Dpack[5]<<8;
			}
			else if(Dpack[0]==0x75&&Dpack[1]==MotorYID)
			{
				MRevBuff.M2Seep=Dpack[4];
				MRevBuff.M2Seep|=Dpack[5]<<8;
			}
			else if(Dpack[0]==0x76&&Dpack[1]==MotorXID)
			{
				MRevBuff.M1torq =Dpack[4];
				MRevBuff.M1torq|=Dpack[5]<<8;
			}
			else if(Dpack[0]==0x76&&Dpack[1]==MotorYID)
			{
				MRevBuff.M2torq =Dpack[4];
				MRevBuff.M2torq|=Dpack[5]<<8;
			}
			else if(Dpack[0]==0x71&&Dpack[1]==MotorXID)
			{
				MRevBuff.M1pos=Dpack[4];
				MRevBuff.M1pos|=Dpack[5]<<8;
				MRevBuff.M1pos|=Dpack[6]<<16;
				MRevBuff.M1pos|=Dpack[7]<<24;
			}
			else if(Dpack[0]==0x71&&Dpack[1]==MotorYID)
			{
				MRevBuff.M2pos=Dpack[4];
				MRevBuff.M2pos|=Dpack[5]<<8;
				MRevBuff.M2pos|=Dpack[6]<<16;
				MRevBuff.M2pos|=Dpack[7]<<24;
			}
			else if(Dpack[0]==0x78&&Dpack[1]==MotorXID&&Dpack[2]==0x01&&Dpack[3]==0)
			{
				MRevBuff.Stop1pos=1;
			}
			else if(Dpack[0]==0x78&&Dpack[1]==MotorYID&&Dpack[2]==0x01&&Dpack[3]==0)
			{
				MRevBuff.Stop2pos=1;
			}
			else
			{
				return 0;
			}		
	}
	else
	{
		
		return 0;
	}
	return 1;
}

/***********************************************************************
获取设备返回数据
@MotorRevBuff:缓存结构
@Data:返回数据包缓存
***********************************************************************/
signed char ReadAnPackData(MotorRevBuff *buff)
{
	int i=0;
	int len=8;
	unsigned char Data[8]={0};
	
	while(CAN_GetFlagStatus(CAN1,CAN_STATUS_TS)){;}
	while(CAN_GetFlagStatus(CAN1, CAN_IT_RI)){;}		
	if(buff->MLen(buff)>=8)
	{
					for(i=0;i<len;i++)
					{
						Data[i]=buff->RevBuff[buff->out];
						buff->out=(buff->out+1)%MOTORBUFFLEN;
					}
					if(DataAnalyze(Data))
					{
						return 1;
					}	
	}
	return 0;
}

/**********************************************************************
获取电机当前转速，转矩，位置
@bff:  数据接受结构体
@speed:返回速度值缓存
@torque:返回转矩值缓存
@ID：设备ID
**********************************************************************/
signed char GetMotorSpeed_Torque_Pos(MotorRevBuff *bff,const unsigned int  ID)
{
	unsigned char   MotorInitVal[8]={MCmd.RotatingSpeed,ID,0x00,0x00,0x00,0x00,0x00,0x00};
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
	MotorInitVal[0]=MCmd.Torque;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
	MotorInitVal[0]=MCmd.GetEncodervalue ;
	MotorSendCanData(MotorInitVal,MOTORDATALEN,MOTORREVCANID);
	return 0;
}
/****************************************************************************************
获取电机当前运动状态
@Mbff:  数据接受结构体
@ID：设备MID
return 0:停止  1：运动
****************************************************************************************/
unsigned char GetMotorStartOrStop(const unsigned int MID)
{
			
	if(MID==MotorYID)
	{
			if((MRevBuff.M1Seep>=-30&&MRevBuff.M1Seep<=20)&&( MRevBuff.M1torq >=0&&MRevBuff.M1torq<=500))
			{
				return 0;
			}
	}
	if(MID==MotorXID)
	{
			if((MRevBuff.M2Seep>-30&&MRevBuff.M2Seep<20)&&( MRevBuff.M2torq >0&&MRevBuff.M2torq<500))
			{
				return 0;
			}
	}
	return 1;
}

/*
轮询查电机状态及速度，位置，扭矩
*/

void PollingMotorSta(void)
{
	GetMotorSpeed_Torque_Pos(&MRevBuff,MotorYID);
	GetMotorSpeed_Torque_Pos(&MRevBuff,MotorXID);
//	ReadAnPackData(&MRevBuff);
}
