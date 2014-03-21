#ifndef GP2_H
#define GP2_H

#include "tests.h"
#include <math.h>

#ifndef TEST
#include <adc.h>
#endif

enum gp2_type
  {
    GP2_RIGHT,
    GP2_LEFT
  };

// Donne la distance en cm de l'objet capté enfonction de la valeur de l'adc
uint16_t gp2_get_dist(enum gp2_type);

#endif//GP2_H
