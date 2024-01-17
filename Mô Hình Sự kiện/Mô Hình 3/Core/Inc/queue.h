/*
 * queue.h
 *
 *  Created on: Jan 6, 2024
 *      Author: admin
 */

#ifndef QUEUE_INC_QUEUE_H_
#define QUEUE_INC_QUEUE_H_
#include<stdio.h>
#define MAX_EVENTS 10
enum SystemEvent {
		EVENT01,EVENT02,EVENT03,EVENT04,EVENT05,EVENT06,EVENT07,EVENT08,EVENT09,EVENT10
};
typedef struct{
	enum SystemEvent event[MAX_EVENTS];
	int head;
	int tail;

}EventQueue;

void EventQueue_Init(EventQueue *queue);
int EventQueue_isEmpty(EventQueue *queue);
void EventQueue_Put(EventQueue* queue,enum SystemEvent event);
enum SystemEvent EventQueue_Get(EventQueue* queue);

#endif /* QUEUE_INC_QUEUE_H_ */
