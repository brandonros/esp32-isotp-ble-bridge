#ifndef TASKS_H
#define TASKS_H

#define TASK_THREAD_SAFE
#define _TASK_INLINE
#include <TaskScheduler.h>

void twai_rx_task_callback();
void twai_alerts_task_callback();
void isotp_poll_task_callback();
void isotp_receive_task_callback();
void periodic_messages_task_callback();

extern Scheduler ts;

extern Task twai_alerts_task;
extern Task twai_rx_task;
extern Task isotp_poll_task;
extern Task isotp_receive_task;
extern Task periodic_messages_task;

#endif
