#include "queue.h"

/*
 * ��ʼ��ѭ������ 
*/
void InitQueue(Queue* q,QUEUE_DATA_T * buffer, unsigned int len)
{
    q -> front =0;
		q -> rear = 0;
		q->MAXSIZE=len;
		q->data=buffer;
}

/*
 *  ѭ����������
 *  q ѭ������
*/
int QueueFull(Queue *q){
    return (q -> rear + 1) % q->MAXSIZE == q -> front;
}

/*  ѭ�������п�q ѭ������*/
int QueueEmpty(Queue *q){

    return q -> front == q -> rear;
}

/*
 *  ����ѭ�����г���
 *  q ѭ������
*/
int QueueLength(Queue *q){
    return (q -> rear - q -> front + q->MAXSIZE) % q->MAXSIZE;
}

/*
 *  ѭ������ ���
 *  q ѭ������
 *  data ���Ԫ��
*/
int EnQueue(Queue* q, ElemType data){
    if(QueueFull(q)){
        return False;
    }
    // ��β���
    q ->data[q -> rear] = data;
    // ���¶�βָ��
    q -> rear = (q -> rear+1 )%q->MAXSIZE;;
    return True;
}

/*
 *  ѭ������ ����
 *  q ѭ������
 *  *val ���������Ԫ�ص����� 
*/
int DeQueue(Queue* q, ElemType *val){
    if(QueueEmpty(q)){
        return False;
    }
    // ��ͷԪ�س���
    *val = q ->data[q -> front];
    // ���¶�ͷָ��
    q -> front = (q -> front + 1) % q->MAXSIZE;
    return True;
}


