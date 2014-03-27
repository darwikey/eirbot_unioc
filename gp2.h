#ifndef GP2_H
#define GP2_H

#include "tests.h"
#include <math.h>

#include <stdint.h>

enum gp2_type
  {
    GP2_RIGHT,GP2_LEFT
  };

// Donne la distance en cm de l'objet capté enfonction de la valeur de l'adc
uint16_t gp2_get_dist(enum gp2_type);

int gp2_get_coor_obstacle(enum gp2_type type, uint16_t distance_max);

#endif//GP2_H
