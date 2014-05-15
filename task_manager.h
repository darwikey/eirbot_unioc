#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <stdio.h>
#include <stdlib.h>


typedef unsigned char uint8_t;
#define MAX_ACTION 10
#define FAILED 128
#define DONE 64
#define HIGH_PRIORITY 2
#define LOW_PRIORITY 1


typedef struct task_manager
{
  uint8_t(*action[MAX_ACTION])(uint8_t type);
  uint8_t flags[MAX_ACTION];
  uint8_t par[MAX_ACTION];
}task_manager_t;


void initTaskManager(task_manager_t *tm);
void addTask(task_manager_t *tm, uint8_t(*action)(uint8_t), uint8_t priority, uint8_t param);
uint8_t doNext(task_manager_t *tm);

#endif