#include "linkedqueue.h"

extern int QueueInit(void)
{
	QueueIn = QueueOut = 0;
	return 0;
}

extern int QueuePut(int newentry)
{
	if(QueueIn == ((QueueOut - 1 + QUEUE_SIZE) % QUEUE_SIZE))
	{
		return -1;	//Queue full
	}

	Queue[QueueIn] = newentry;
	QueueIn = (QueueIn + 1) % QUEUE_SIZE;
	return 0;	//no errors
}

extern int QueueGet(int *old)
{
	if(QueueIn == QueueOut)
	{
		return -1;	//queue empty
	}

	*old = Queue[QueueOut];
	QueueOut = (QueueOut + 1) % QUEUE_SIZE;
	return 0;	//no errors
}


