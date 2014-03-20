#include "gp2.h"
#include <uart.h>

void gp2_init(struct gp2_cfg_t* gp2) {
  gp2->size = 0;
}

void gp2_init_xya(struct gp2_cfg_t* gp2, int16_t x, int16_t y, int16_t a)
{
  gp2_init(gp2);
  gp2->x = x;	  
  gp2->y = y;	  
  gp2->a = a;	  
}

void gp2_add_point(struct gp2_cfg_t* gp2, uint16_t dist, uint16_t volt) {
  struct dist_adc e;
 e.dist = dist;
 e.adc = volt * VOLT2ADC;
  struct dist_adc tmp;

  if(gp2->size >= GP2_SAMPLES)
    return;

  for(int i = 0 ; i < gp2->size ; i++) {
    if(gp2->points[i].adc >= e.adc) {
	tmp.dist = gp2->points[i].dist;
	tmp.adc = gp2->points[i].adc;
	gp2->points[i].dist = e.dist;
	gp2->points[i].adc = e.adc;
	e.adc = tmp.adc;
	e.dist = tmp.dist;
    }
  }
  gp2->points[gp2->size].dist = e.dist;
  gp2->points[gp2->size].adc = e.adc;
  gp2->size++;
}


uint16_t gp2_get_dist(struct gp2_cfg_t* gp2, uint16_t adc) {
  for(int i = 0 ; i < gp2->size ; i++) {
    if(gp2->points[i].adc >= adc) {
      return gp2->points[i].dist;
    }
  }
  return 0;
}

void gp2_get_obstacle(struct gp2_cfg_t* gp2, uint16_t adc, int16_t* x, int16_t* y, int16_t* a)
{
  uint16_t dist = gp2_get_dist(gp2, adc);
  //*x = gp2->x + cos(((float)gp2->a*3.14)/180.)*dist;
  //*y = gp2->y + sin(((float)gp2->a*3.14)/180.)*dist;
  if(gp2->a == 180 || gp2->a == -180)
	  *x = gp2->x - dist;
  else
	  *x = gp2->x + dist;
  *y = gp2->y;
  *a = gp2->a;
}


// TODO !!!!
uint16_t gp2_too_far(struct gp2_cfg_t* gp2) {
  return 0;
}
