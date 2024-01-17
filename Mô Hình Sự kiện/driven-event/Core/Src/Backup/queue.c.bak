/*
 * queue.c
 *
 *  Created on: Jan 6, 2024
 *      Author: admin
 */
#include<stdio.h>
#define MAX_EVENTS 10
enum SystemEvent {
	EVENT01,EVENT02,EVENT03,EVENT04,EVENT05,EVENT06,EVENT07,
	EVENT08,EVENT09,EVENT10};
typedef struct{
	enum SystemEvent event[MAX_EVENTS];
	int head;
	int tail;

}EventQueue;
void EventQueue_Init(EventQueue *queue){
	queue->head=0;
	queue->tail=0;


}


int EventQueue_isEmpty(EventQueue *queue){
	return queue->head == queue->tail;
}
void EventQueue_Put(EventQueue* queue,enum SystemEvent event){
	queue->event[queue->tail]=event;
	queue->tail= (queue->tail+1)%MAX_EVENTS;
}
enum SystemEvent EventQueue_Get(EventQueue* queue){

	enum SystemEvent event=queue->event[queue->head];
	queue->head=(queue->head+1)%MAX_EVENTS;
	return event;

}
