#ifndef GP2_H
#define GP2_H

#include "tests.h"
#include <math.h>

#ifndef TEST
#include <adc.h>
#endif

//EIRBUG

#define VOLT2ADC 2/10 // coefficient de convertion en 1/V

#define GP2_SAMPLES (10)

typedef struct dist_adc dist_adc;
struct dist_adc {
  uint16_t dist;
  uint16_t adc;
};

typedef struct gp2_cfg_t gp2_cfg_t;
struct gp2_cfg_t {
  uint8_t size;
  struct dist_adc points[GP2_SAMPLES];
  int16_t x;
  int16_t y;
  int16_t a; 
};

// Configure l'approximation de la courbe du GP2
void gp2_init(struct gp2_cfg_t* gp2);

// Configure l'approximation de la courbe du GP2 et sa position par rapport au robot
void gp2_init_xya(struct gp2_cfg_t* gp2, int16_t x, int16_t y, int16_t a);

// Ajoute des poins à la courbe du gp2
void gp2_add_point(struct gp2_cfg_t* gp2, uint16_t dist, uint16_t volt);

// Donne la distance en cm de l'objet capté enfonction de la valeur de l'adc
uint16_t gp2_get_dist(struct gp2_cfg_t* gp2, uint16_t adc);

// Donne la position relative de l'objet détecté par rapport au gp2
void gp2_get_obstacle(struct gp2_cfg_t* gp2, uint16_t adc, int16_t* x, int16_t* y, int16_t* a);

// Donne la plus grand distance detectable par le gp2
// TODO !!
uint16_t gp2_too_far(struct gp2_cfg_t* gp2);

#endif//GP2_H
