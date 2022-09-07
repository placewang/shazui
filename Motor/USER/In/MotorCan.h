#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include "can.h"

#define  True  1
#define  False 0
#define  MOTORBUFFLEN                     80 
#define  MOTORREVCANID                    0x000001
#define  MOTORDATALEN                     8
#define  KnifeNum                         16           //单个模块剪刀数量	

typedef  CanPeliRxMsg  MotorCanRevBuff;

//电机主命令
typedef struct {
	unsigned char ZeroHit;																//撞零
	unsigned char Parameter;															//参数设置一级指令			
	unsigned char Enable;																	//电机使能/失能
	unsigned char MoveWthout;															//位置模式运动
	unsigned char MoveWith;																//位置模式运动完成后返回当前位置	
	unsigned char GetEncodervalue;												//读当前编码器位置	
	unsigned char Read;																		//读电机一级指令
	unsigned char Limit;																	//运动极限设置
	unsigned char Torque;                                 //读转矩
	unsigned char RotatingSpeed;                          //读转速
	unsigned char ClearALarm;                             //清报警
	unsigned char MovezeroRev;                            //归零完成后自动回复
	unsigned char OriginalEncodedVal;                     //读原始编码器值
	unsigned char SeepMode;                               //速度模式	
}MotorCmd;

//电机子命令
typedef struct {
	unsigned short SetEncodervalue;                      //设置编码器值                         
	unsigned short SetDir;	                             //设置电机运转正方向
	unsigned short SetReturnZeroOffset;                  //设置电机撞停的偏移量
	unsigned short SetHitstopPosValue;		               //设置电机撞停后编码器值
	unsigned short SetFindZeroMode; 			               //设置电机回零方式
	unsigned short SetMaxSpeed;						               //设置电机最高转速
	unsigned short SetPosvalue;						               //设置电机位置
	unsigned short SetBaud;							                 //设置通讯波特率
	unsigned short SetStartStiffness;                    //设置电机启动刚性
	unsigned short SetStopStiffness;	                   //设置电机停止刚性
	unsigned short SetLimit;                             //设置电机运动极限（大小）
	unsigned short ElectronicGearRatio;                  //设置电机电子齿轮比
  unsigned short VersionSwitch;                        //协议版本切换
}MotorSubCmd;

typedef struct MotorRev
{
	unsigned char  Revouttime;                           //接收计时
	unsigned int   TaskTime;                             //任务执行间隔
	unsigned int   subscript;                            //CAN指令入队下标  
	unsigned int   out;													         //CAN指令出队下标			
	MotorCanRevBuff  RevBuff[MOTORBUFFLEN];              //Can指令接收缓存队列	
	unsigned int (*MLen)(struct MotorRev *);	           //计算循环队列长度
	int (*DeQueue)(struct MotorRev *, MotorCanRevBuff *);//出队
	int (*EnQueue)(struct MotorRev *, MotorCanRevBuff *);//入队
}MotorRevBuff;

typedef struct executionstat
{
	  unsigned char  MoveZeroSta[2];                     //电机回零状态（0:x电机（平移）1:Y电机（剪刀）下同）
		unsigned char  MoveTargetSta[2];                   //电机返回运动至目标位置
	  unsigned char  SenSorSta[2];                       //到位传感器(剪刀电路接在X电机上）
		signed   int   Mpos[2];											       //电机实时位置
		unsigned short MoveTimeOut[2];                     //电机运动计时
}MotorExeSTa;


typedef struct Alarm
{
	unsigned char  MotorErrID[2];								         //电机ID
	unsigned char  MotorAlarmGrade[2];                   //电机报警等级		
	unsigned char  MotorEorrStat[2];                     //电机报错状态(非零代表错误类型1电机过流 2电机过压3电机欠压\
																																			4电机过载5电机过速6电机堵转\
																																			7电机左限位8电机右限位9电机撞零超时\
																																			10电机找边超时11电机打开刹车错误12步进电机锁定误差过大\
																																			)	
}MotorAlarm;


typedef struct MotorConfig
{
	unsigned char ReturnZeroDirection[2];                   //回零方向
  signed int  MotorXYLACUNA;                              //剪刀电机在平移位置上的两刀之间的间隙偏移量
  signed int  MotorYLACUNA;                               //剪刀电机回零偏移量
  signed int  MotorXLACUNA;                               //平移电机回零偏移量	
	signed int  MoveMentDirection[2];                       //运动正方向
	signed int  MoveToZero[2];                              //回零方式
	signed int  TopSpeed[2];                                //最高速度
	signed int  ElectronicGearRatio[2];                     //电子齿轮比

	const signed int  *YMoveStandard;                       //剪刀电机选刀位置存储
  const signed int  *XMoveStandard;                       //平移电机出刀位置存储
	unsigned int MotorID[2];                                //电机ID
}MotorConfig;


//电机属性句柄

typedef struct Property
{               
	MotorConfig *  Mcg;
	MotorAlarm  *  Mer;
	MotorExeSTa *  Mst;
}MotorProperty;



/***********************MotorCan.c**************************************/
extern MotorRevBuff MRevBuff;   
extern MotorCmd     MCmd;
extern MotorSubCmd	MSCmd;
/***********************MotorMove.c**************************************/
extern MotorExeSTa  MExeSta;
extern MotorExeSTa  RMExeSta;
extern MotorConfig  MtCgL;
extern MotorConfig  MtCgR;
extern MotorAlarm   MErrState;
extern MotorAlarm   RMErrState;     
extern MotorProperty  MtProperty1_L;
extern MotorProperty  MtProperty1_R;

/***********************MotorCan.c**************************************/
unsigned int MRevbuffLen(MotorRevBuff *);
int DeQueue(MotorRevBuff *, MotorCanRevBuff *); //出队
int EnQueue(MotorRevBuff*, MotorCanRevBuff *);  //入队
void PollingMotorSta(void);
signed char ReadAnPackData(MotorRevBuff *);
signed char MotorSendCanData(const unsigned char * ,const unsigned int,const unsigned int );
/***********************MotorMove.c**************************************/
signed char EnableOrClearALarm(const unsigned short,const unsigned char);
signed char RestMotorSta(MotorProperty* );
signed char ClearMStat(MotorProperty*,const unsigned short ,char );
void clearM(MotorProperty* );
signed char MoveSeepMode(MotorProperty*,const signed short,signed int, const unsigned int);
signed char MoveToTargetPos(MotorProperty*,const signed short ,signed int , const unsigned int );
signed char MoveZero(MotorProperty*,const unsigned short,const signed short,const unsigned short ,unsigned char,signed int);
unsigned char MotorCanInit(MotorProperty* );
signed char MorotSetEncodervalue(MotorProperty*,const unsigned short,const signed int);
/**************************************MotorfeatureSet.c*************************************************/
signed char DreMoveZero(MotorProperty* );
signed char YMoveZero(MotorProperty* ,const signed short,signed int );
signed char YMoveZeroTrial(MotorProperty* );
signed char ScissorsReset(MotorProperty* );
signed char KnifeSelection2(MotorProperty* ,const short );
signed char CloseKnife2(MotorProperty* ,const short );
#endif
















