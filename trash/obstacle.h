#ifndef _OBSTACLE_H_
#define _OBSTACLE_H_
#include "tests.h"
#ifndef TEST
#include <aversive.h>
#endif
/*
   +-> x
   |
   v y
          (0,0)
|------\  +  /------|
|H      \___/      H|
|H                 H|
|H                 H|
|H                 H|
|___________________|

H : assiete
 */

enum figure {RECT, CIRCLE};

typedef struct obstacle obstacle;
struct obstacle {
  enum figure fig;
  int16_t x;
  int16_t y;
  int16_t dx;
  int16_t dy;
  int16_t dr;
};

// Configuration
#define TABLE_WIDTH 300
#define TABLE_HEIGHT 200
#define GATEAU_RAY 70

#define ASSIETE_W 17
#define ASSIETE_DIST_X (20 - ASSIETE_W/2)
#define ASSIETE_DIST_Y (25 - ASSIETE_W/2)
#define ASSIETE_DIST_INTER (60 - 25 - ASSIETE_W)

#define INFINITE 1000
#define OBS_DODGE 1
#define OBS_IGNORE 2

// Macros Utiles
#define MK_RECT_WH(obs, _x, _y, w, h) obs.fig = RECT; obs.x = _x; obs.y = _y; obs.dx = w; obs.dy = h
#define MK_RECT_2P(obs, x1, y1, x2, y2) obs.fig = RECT; obs.x = x1; obs.y = y1; obs.dx = x2 - x1; obs.dy = y2 - y1
#define MK_CIRCLE(obs, _x, _y, r) obs.fig = CIRCLE; obs.x = _x; obs.y = _y; obs.dr = r
#define MK_COPY(obs1,obs2) obs1.fig = obs2.fig; obs1.x = obs2.x ; obs1.y = obs2.y ; obs1.dr = obs2.dr
// Obstacles
enum obstacle_id {
  NORTH_BOUND,
  OUEST_BOUND,
  SOUTH_BOUND,
  EAST_BOUND,
  CAKE,

  MAX_OBSTACLE
};

extern obstacle obstacles[MAX_OBSTACLE];


void init_obstacle(void);

// TODO: les fonctions sont à faire !!!

uint8_t known_obstacle(int16_t x, int16_t y);

uint8_t is_obstacle(obstacle* obs, int16_t x, int16_t y);


#endif //_OBSTACLE_H_
