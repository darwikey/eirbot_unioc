#include "obstacle.h"

obstacle obstacles[MAX_OBSTACLE];

void init_obstacle() {
  //// Murs de la table
  //MK_RECT_2P(obstacles[NORTH_BOUND], -INFINITE, -INFINITE, INFINITE, 0);
  //MK_RECT_2P(obstacles[OUEST_BOUND], -INFINITE, -INFINITE, - TABLE_WIDTH/2, INFINITE);
  //MK_RECT_2P(obstacles[SOUTH_BOUND], -INFINITE, TABLE_HEIGHT, INFINITE, INFINITE);
  //MK_RECT_2P(obstacles[EAST_BOUND], TABLE_WIDTH/2, -INFINITE, INFINITE, INFINITE);
  
  //// Gateau
  //MK_CIRCLE(obstacles[CAKE], 0, 0, GATEAU_RAY);
  //// Assietes
  //OSEF 
}

uint8_t known_obstacle(int16_t x, int16_t y) {
  uint8_t ret = 0;

  for(uint8_t i = 0; i < MAX_OBSTACLE; i++) {
    if(ret = is_obstacle(&obstacles[i], x, y))
		return ret;
  }
  
  return ret;
}

uint8_t is_obstacle(obstacle* obs, int16_t x, int16_t y) {
  if(obs->fig == RECT) {
    return ((obs->x <= x) && (x <= obs->x + obs->dx) && (obs->y <= y) && (y <= obs->y + obs->dy))*2;
  }
  else if(obs->fig == CIRCLE) {
    int16_t _x = obs->x - x;
    int16_t _y = obs->y - y;
    return (_x*_x + _y*_y) <= obs->dr*obs->dr;
  }

  return 0;
}
