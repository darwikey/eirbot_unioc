#include "evitement.h"

#include <math.h>

#define MAX_CHEMIN 50
chemin buffer[MAX_CHEMIN];
uint8_t buff_size;

uint16_t dist(struct pos* p1, struct pos* p2) {
	uint16_t x, y;
	x = (p1->x - p2->x) / 16;
	y = (p1->y - p2->y) / 16;

	return x*x+y*y;
}

#define SET_XY(pos, _x, _y) \
	pos.x = _x; pos.y = _y; pos.a = 0

#define DIST_PTS 30
#define EPSILON (40/16)

chemin* evitement_get_chemin(struct obstacle obs[], uint8_t obs_num, struct pos* cur, struct pos* target) {

  // Si target est un obstacle, pas de chemin
  PRINT_DBG("Determine si target est un obstacle\n");
  for(int8_t k = 0 ; k < obs_num ; k++) {
    if(is_obstacle(&obs[k], target->x, target->y)) {
      PRINT_DBG("target est un obstacle");
      return NULL;
    }
  }

  // Initialisation
  buff_size = 0;
  SET_XY(buffer[0].target, cur->x, cur->y);
  buffer[0].target.x = cur->x;
  buffer[0].target.y = cur->y;
  buffer[0].next = NULL;
  
  PRINT_DBG("buffer[0] = (%d, %d)\n", buffer[0].target.x, buffer[0].target.y);
  PRINT_DBG("target = (%d, %d)\n", target->x, target->y);
  PRINT_DBG("distance de target : %d\n", dist(&buffer[buff_size].target, target));
  while(buff_size < MAX_CHEMIN && dist(&buffer[buff_size].target, target) > EPSILON*EPSILON) {
    // Calcul du point le plus proche
    PRINT_DBG("Calcul du point le plus proche\n");

    struct pos min;
    min.x = buffer[buff_size].target.x;
    min.y = buffer[buff_size].target.y;
    uint16_t min_dist = 0xffff;//dist(&min, target);
    PRINT_DBG("min_dist = %d\n", min_dist);
    for(int8_t i = -1 ; i < 2 ; i++) {
      for(int8_t j = -1 ; j < 2 ; j++) {
	// ne reste pas sur place
	if(!(i == 0 && j == 0)) {
	 
	  struct pos pts; 
	  pts.x = buffer[buff_size].target.x + i * DIST_PTS;
	  pts.y = buffer[buff_size].target.y + j * DIST_PTS;
	  
	  // teste si on ne retourne pas en arriere
	  uint8_t ret = (pts.x == buffer[buff_size - 2].target.x &&
			 pts.y == buffer[buff_size - 2].target.y);

	  //  teste si le point n'est pas un obstacle
	  if(!known_obstacle(pts.x, pts.y) && !ret) {
	    uint8_t isobs = 0;
	    for(int8_t k = 0 ; k < obs_num && !isobs ; k++)
	      isobs = isobs || is_obstacle(&obs[k], pts.x, pts.y);

	    if(!isobs) {
	      // verifie si c'est le minimum	    
	      uint16_t d = dist(&pts, target);
	      PRINT_DBG("dist = %d\n", d);
	      if(d < min_dist) {
		PRINT_DBG("C'est le min !!\n");
		min.x = pts.x;
		min.y = pts.y;
		min_dist = d;
	      }
	    }
	  }
	  else {
	    PRINT_DBG("(%d, %d) => OBSTACLE !\n", pts.x, pts.y);
	  }
	}
      }
    }

    // Ajoute le point selectionne au chemin
    buffer[buff_size].target.x = min.x;
    buffer[buff_size].target.y = min.y;
    buffer[buff_size].next = &buffer[buff_size+1];
    PRINT_DBG("buffer[%d] = (%d, %d)\n", buff_size, buffer[buff_size].target.x, buffer[buff_size].target.y);
    buff_size++; 
    buffer[buff_size].target.x = buffer[buff_size-1].target.x;
    buffer[buff_size].target.y = buffer[buff_size-1].target.y;

  }

  // Ajoute le point target si besoin
  if(buffer[buff_size].target.x == target->x &&
     buffer[buff_size].target.y == target->y) {
    buffer[buff_size].next = NULL;
  }
  else {
    buffer[buff_size].target.x = target->x;
    buffer[buff_size].target.y = target->y;
    buffer[buff_size].next = NULL;
    buff_size++;
    PRINT_DBG("Ajustement de la derniere position\n");
  }
  
  return &buffer[0];
}
