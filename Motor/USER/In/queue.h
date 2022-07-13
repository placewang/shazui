#ifndef __QUEUE__H
#define __QUEUE__H

#define True  1
#define False 0




#define QUEUE_DATA_T  unsigned char         //�����������Ͷ���

typedef unsigned char ElemType;

// ����ѭ�����нṹ��
typedef struct Queue{
	
	unsigned int front;	  //����ͷ�±�
	unsigned int rear;	  //����β�±�
	unsigned int MAXSIZE; //���л��泤�ȣ���ʼ��ʱ��ֵ��
	QUEUE_DATA_T *data;
}Queue;



extern Queue Can_revQueuebuff;


void InitQueue(Queue* q,QUEUE_DATA_T * buffer, unsigned int len);

int DeQueue(Queue* , ElemType *);
int EnQueue(Queue* , ElemType );
int QueueLength(Queue *q);

#endif




