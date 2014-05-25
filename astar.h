#ifndef ALGO_H
#define ALGO_H
#include <stdint.h>
#include "position_manager.h"
#include "trajectory_manager.h"
#include "task_manager.h"
#include "avoidance.h"

#define DIST  10
#define OBSTACLE 1
#define GOAL 2
#define START 3
#define OPENLIST 4
#define CLOSEDLIST 5
#define UNIT 20
#define LENGTH (300 + UNIT)
#define G_LENGTH 15
#define WIDTH (200 + UNIT)
#define G_WIDTH 10
#define G_SIZE G_WIDTH*G_LENGTH
#define OUT 0
#define STACK_SIZE 20
#define DRIVE 1
#define ROTATE 2

typedef struct node node;
struct node
{
  uint8_t parent;//1o
  //uint8_t remainDist;//1o//hcost
  uint8_t crossedDist;//1o//gcost
  //int allDist;//0o//fcost
  int8_t type;//1o
  uint8_t coor;//1o
};




typedef struct mvStackElement mvStackElement;
struct mvStackElement
{
  float val;
  uint8_t type;
};

typedef struct mvStack mvStack;
struct mvStack
{
  uint8_t top;
  mvStackElement items[STACK_SIZE];
};

typedef struct coordinate coordinate;
struct coordinate
{
  uint8_t x;
  uint8_t y;
};


uint8_t aStarLoop(void);
uint8_t findDist(node node1, node node2);
int8_t isVoid(int8_t list);
void initNeighbors(uint8_t current, uint8_t *neighbors);
uint8_t findBest(uint8_t openlist);
//int findDistC(node node1, node node2);
coordinate getCoor(node n);
void initObstacle(void);
void polishing(mvStack *s);
int8_t astarMv(void);
void printGraphe(void);
void putObstacle(uint8_t coor);
void deleteObstacle(uint8_t coor);


uint8_t get_startCoor(void);
uint8_t get_goalCoor(void);
void set_startCoor(uint8_t coor);
void set_goalCoor(uint8_t coor);

//
mvStack initStack(void);
uint8_t stack_is_empty(mvStack *s);
uint8_t stack_full(mvStack *s);
uint8_t stack_size(mvStack *s);
void stack_clear(mvStack *s);
void push(mvStack *s,mvStackElement item);
mvStackElement pop(mvStack *s);
void stopAstarMovement(void);
//
#endif
