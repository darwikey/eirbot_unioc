#ifndef POSITION_MANAGER_H
#define POSITION_MANAGER_H

#define ROBOT_PM_UPDATE_TIME 15000

#define ROBOT_IMP_PI  (49270.0/5.235) //(49270.0/5.870) // (49270.0/5.824) 
#define ROBOT_IMP_CM 111

#include <aversive.h>
#include "unioc_config.h"
//#include "notification_manager.h"
#include "fxx.h"
struct pos {
  int16_t x;
  int16_t y;
  int16_t a;
};

typedef struct 
{
  //fpga brut
  int32_t distance;
  int32_t angle;
  int32_t x;
  int32_t y;
  
  //infos deduites
  int32_t angle_mod_2pi;
  int32_t vitesse_distance;
  int32_t vitesse_angle;

  //var de calculs
  int32_t histo_distance;
  int32_t histo_angle;

  //constantes en fxx
  fxx c_imp2deg;
  fxx c_imp2cm;
  fxx c_delta_call;


  uint8_t cal;
  int32_t cal_sec;

  //notification_manager_t * not;
  

  // EIRBUG
  //// Absolute Coordinate System Begining Position
  int16_t acs_angle_begin;
  int16_t acs_x_begin;
  int16_t acs_y_begin;
  ////
}position_manager_t;

// EIRBUG
void position_abs(position_manager_t* pm, int16_t* x, int16_t* y, int16_t* a);

void position_begin_set(position_manager_t* pm, int16_t x, int16_t y, int16_t a);

////

void position_init(position_manager_t *);//,notification_manager_t *);
void position_update_low_level(void *);
int32_t position_get_angle(position_manager_t *);
int32_t position_get_angle_mod2pi(position_manager_t *);
fxx position_get_angle_mod2pi_deg(position_manager_t *);
fxx position_get_angle_deg(position_manager_t *pm);
int32_t position_get_vitesse_angulaire(position_manager_t *);
fxx position_get_vitesse_angulaire_degs(position_manager_t *);
int32_t position_get_distance(position_manager_t *);
fxx position_get_distance_cm(position_manager_t *);
int32_t position_get_vitesse(position_manager_t *);
fxx position_get_vitesse_ms(position_manager_t *);
int32_t position_get_x(position_manager_t *);
int32_t position_get_y(position_manager_t *);
fxx position_get_x_cm(position_manager_t *);
fxx position_get_y_cm(position_manager_t *);
uint8_t position_get_coor(position_manager_t *);
void position_set_xya_cm_deg(position_manager_t *,fxx x,fxx y, fxx a);

int32_t position_cm2imp(position_manager_t *,fxx cm);
int32_t position_deg2imp(position_manager_t *,fxx deg);
int32_t position_rad2imp(position_manager_t *,fxx deg);

#endif //POSITION_MANAGER_H
