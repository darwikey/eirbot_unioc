#ifndef EVITEMENT_H
#define EVITEMENT_H

#include "tests.h"

#include "obstacle.h"

typedef struct chemin chemin;
struct chemin {
  struct pos target;
  chemin* next;
};

chemin* evitement_get_chemin(struct obstacle obs[], uint8_t obs_num, struct pos* cur, struct pos* target); 
#endif//EVITEMENT_H
