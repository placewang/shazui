#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H
#include "delay.h"
#define  MOTORBUFFLEN                     80 
#define  MOTORREVCANID                    0x000001
#define  MOTORDATALEN                     8

#define  MotorYID                         0x02          //剪刀电机ID
#define  MotorXID                         0x01					//平移电机ID
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
}MotorSubCmd;

typedef struct MotorRev
{
	
	signed   char  Stop1pos;									  //撞停平移返回
	signed   char  Stop2pos;										//撞停剪刀返回
	unsigned char  Revouttime;                  //接收超时 
	unsigned char  RevBuff[MOTORBUFFLEN];       //Can指令接受缓存	
	signed   short M1Seep;                      //平移电机实时速度
	unsigned short M1torq;                      //平移电机实时转矩
	signed   short M2Seep;                      //剪刀电机实时速度
	unsigned short M2torq;                      //剪刀电机实时转矩
	unsigned int   subscript;
	unsigned int   out;
	unsigned int   TaskTime;                    //任务执行时间
	signed   int   M1pos;											  //平移电机实时位置
	signed   int   M2pos;											  //剪刀电机实时位置
	signed   int   YMoveTargetval;              //剪刀电机运动目标值
	signed   int   XMoveTargetval;              //平移电机运动目标值
	int (*MFull)(struct MotorRev *);            //判断缓存满函数
	int (*MEmpty)(struct MotorRev *);           //判断缓存满函数
	unsigned int (*MLen)(struct MotorRev *);	  //计算循环队列长度
}MotorRevBuff;

typedef struct executionstat
{
		unsigned char  MoveZeroStaY;                  //剪刀电机回零状态
		unsigned char  MoveZeroStaX;                  //平移电机回零状态	
		unsigned char  MoveTargetStaY;                //剪刀电机运动至目标位置
		unsigned char  MoveTargetStaX;                //平移电机运动到目标位置
		unsigned char  MoveStart;                     //运动开始标志 
}MotorExeSTa;

typedef struct Alarm
{
	unsigned char  MCUCommunication;           //芯片内部通讯报警
	unsigned char  Power;                      //电源报警
	unsigned char  CurrentPeak;                //超过电流峰值报警
	unsigned char  Stall;                      //堵转报警  
	unsigned char  MCUMemory;                  //芯片内存错误报警
	unsigned char  MCU;                        //芯片内部报警
	unsigned char  Overload;                   //过载报警
	unsigned char  Overvoltage;                //过压报警	
	unsigned char  Motor;                      //电机处于报警状态
	unsigned char  SelfRunningFollowUp;        //自跑式跟随报警
	unsigned char  Limit;                      //限位报警
	unsigned char  Rake;                       //耙子电机未在线
	unsigned char  Overheating;                //电机过温
	unsigned char  MotorEorrStat;              //电机报错状态
	unsigned char  MotorErrID;								 //电机报警编号（设备报警）
	unsigned char  MotorAlarmGrade;            //电机报警等级	
//	unsigned char  (*MESta)(MotorRevBuff *,struct Alarm *);
}MotorAlarm;


extern MotorRevBuff MRevBuff;   
extern MotorAlarm   MEState;
extern MotorCmd     MCmd;
extern MotorSubCmd	MSCmd;
extern MotorExeSTa  MExeSta;


int MRevbuffFull(MotorRevBuff *);
int MRevbuffEmpty(MotorRevBuff *);
unsigned int MRevbuffLen(MotorRevBuff *);

unsigned char MotorCanInit(void);

signed char MoveZero(const unsigned short MID,const signed short Seep,const unsigned short Torque);
signed char MotorSendCanData(const unsigned char * ,const unsigned int ,const unsigned int );
void PollingMotorSta(void);
void  MotorStaRenew(void);
unsigned char GetMotorStartOrStop(const unsigned int);
signed char ReadAnPackData(MotorRevBuff *);

void MoveToTargetPos(const signed ,const signed int , const unsigned short );


#endif
















