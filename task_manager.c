#include "task_manager.h"

uint8_t perfectRoad = 1;

uint8_t isFailed = 0;


void rmFailedFlag(task_manager_t *tkm);
uint8_t allRemainFailed(task_manager_t *tkm);

void initTaskManager(task_manager_t *tkm)
{
  for (uint8_t i = 0; i < MAX_ACTION; i++)
  {
    tkm->action[i] = NULL;
    tkm->flags[i] = 0;
    tkm->par[i] = 0;
  }
}

void addTask(task_manager_t *tkm, uint8_t(*action)(uint8_t), uint8_t priority, uint8_t param)
{
  uint8_t i = 0;
  while (tkm->action[i] != NULL && i < MAX_ACTION)i++;
  if (i < MAX_ACTION)
  {
    tkm->action[i] = action;
    tkm->flags[i] += priority;
    tkm->par[i] = param;
  }
}

uint8_t doNextTask(task_manager_t *tkm)
{
  
  int8_t next = -1;//mettre un int8_t
  uint8_t i = 0;
  uint8_t finished = 0;
  isFailed=0;

  if (allRemainFailed(tkm))
  { 
    rmFailedFlag(tkm);
    perfectRoad = 0;
  }
  
  for (i = 0; i < MAX_ACTION; i++)
  {

    if (!(tkm->flags[i] > DONE) && tkm->action[i] != NULL)
    {
      if (perfectRoad == 1)
      {//just take the next in the order
        next = i;
        finished = 1;
        break;
      }
      else
      {
       // by priority
        if (next == -1)
        {
           next = i;
           finished = 1;
        }//les failed on moins de priorite que les pas failed
        if (next != -1 &&
          (tkm->flags[i] % FAILED > tkm->flags[next] % FAILED || (!(tkm->flags[i] & FAILED) && tkm->flags[next] & FAILED)))
        {
           next = i;
           finished = 1;
        }
      }
    }
  }
  if (finished)
  {
     printf("action n: %d \n", next);
     isFailed = 0;
     tkm->flags[next] |= tkm->action[next](tkm->par[next]);
     printf("retourne %d \n", tkm->flags[next]);
    // //if (tkm->flags[next] & FAILED)perfectRoad = 2;
     return 1;
  }
  else return 0;//we have done everything
}

void actionFailed(void)
{
  isFailed = 1;
}

uint8_t actionIsFailed(void)
{
  return isFailed;
}


uint8_t allRemainFailed(task_manager_t *tkm)
{
  uint8_t allFailed = 1;
  for (uint8_t i = 0; i < MAX_ACTION; i++)
  {
    if (!(tkm->flags[i] & FAILED) && !(tkm->flags[i] & DONE) && tkm->action[i]  )
    {
      allFailed = 0;
    }
  }
  return allFailed;
}

void rmFailedFlag(task_manager_t *tkm)
{
  for (uint8_t i = 0; i < MAX_ACTION; i++)
  {
    tkm->flags[i] &= 0xff - FAILED;
  }
}