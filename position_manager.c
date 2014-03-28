#include <scheduler.h>
#include <wait.h>
#include <math.h>
#include <uart.h>
#include "position_manager.h"
#include "astar.h"


// EIRBUG
void position_abs(position_manager_t* pm, int16_t* x, int16_t* y, int16_t* a) {
	*x = pm->acs_x_begin + fxx_to_double(position_get_x_cm(pm));
        *y = pm->acs_y_begin - fxx_to_double(position_get_y_cm(pm));
	*a = pm->acs_angle_begin + fxx_to_double(position_get_angle_mod2pi_deg(pm));	
}


void position_begin_set(position_manager_t* pm, int16_t x, int16_t y, int16_t a) {
  pm->acs_x_begin = x;
  pm->acs_y_begin = y;
  pm->acs_angle_begin = a;
}

////

void position_init(position_manager_t * pm)//,notification_manager_t *not)
{
  //pm->not = not;
  
  //init constantes
  // c_imp2deg = 180/ROBOT_IMP_PI
  pm->c_imp2deg = fxx_div(fxx_from_double(180.0),fxx_from_double(ROBOT_IMP_PI));
  pm->c_imp2cm = fxx_inv(fxx_from_double(ROBOT_IMP_CM));
  pm->c_delta_call = fxx_from_double(ROBOT_PM_UPDATE_TIME);

  //connerie a pas faire : oublier le L... (atmega = 16bits)
  U_PM_2PI = ROBOT_IMP_PI*2L; 

  //init fpga
//  U_PM_CONFIG = (ROBOT_ENC_LEFT) | (ROBOT_ENC_RIGHT<<4) | (ROBOT_ENC_LEFT_INVERT << 3) | (ROBOT_ENC_RIGHT_INVERT << 7);


  //init uC
  pm->histo_distance = 0;
  pm->histo_angle = 0;
  U_PM_X = 0;
  U_PM_Y = 0;
  U_PM_FLAGS = U_PM_FLAGS_SET_REGISTERS;
  nop();
  U_PM_FLAGS = 0;


  pm->cal = 0;
  pm->cal_sec = 0;

  position_update_low_level((void *)pm);
  scheduler_add_periodical_event(position_update_low_level,(void*)pm,ROBOT_PM_UPDATE_TIME/SCHEDULER_UNIT);
}


void position_set_xya_cm_deg(position_manager_t *pm,fxx x,fxx y, fxx a)
{
  U_PM_X = position_cm2imp(pm,x);
  U_PM_Y = position_cm2imp(pm,y);
  pm->cal_sec = position_deg2imp(pm,a);
  U_PM_ANGLE = pm->cal_sec;
  U_PM_FLAGS = U_PM_FLAGS_SET_REGISTERS;
  wait_ms(10);
  U_PM_FLAGS = 0;
  pm->angle = U_PM_ANGLE;
}
void position_update_low_level(void * arg)
{
  static uint8_t tmp = 0;
  tmp = (tmp + 1)%20;
  //_SFR_MEM8(0x8000) = tmp>10;
  //ancienne led, devenu obslete car codeur

  position_manager_t * pm = (position_manager_t *)arg;

  U_PM_FLAGS = U_PM_FLAGS_LOCK;

//  U_PM_FLAGS = U_PM_FLAGS_SET_REGISTERS;
  nop();

  pm->angle = U_PM_ANGLE;
  pm->distance = U_PM_DISTANCE;
  pm->x = U_PM_X;
  pm->y = U_PM_Y;
  
  U_PM_FLAGS = 0;

  pm->angle_mod_2pi = (pm->angle % (int32_t)(2L * ROBOT_IMP_PI));
  if(pm->angle_mod_2pi < 0)
  {
    pm->angle_mod_2pi += 2L * ROBOT_IMP_PI;
  }
  pm->vitesse_distance = pm->distance - pm->histo_distance;
  pm->histo_distance = pm->vitesse_distance;
  pm->vitesse_angle = pm->angle - pm->histo_angle;
  pm->histo_angle = pm->vitesse_angle;

  //notification_notify(pm->not,NOTIFICATION_POSITION_NEW_COORD,pm);

  if(PINF & 0x04)
  {
    cbi(PORTE,7);
  }
  else
  {
    sbi(PORTE,7);
  }
  
  if(PINF & 0x08)
  {
    cbi(PORTE,5);
  }
  else
  {
    sbi(PORTE,5);
  }

  if((PINF & 0x0C) == 0)
  {
    if(pm->cal == 0)
    {
      //notification_notify(pm->not,NOTIFICATION_POSITION_CALAGE,NULL);
    }
    pm->cal = 1;
  }
  else if((PINF & 0x0C) == 0x0C)
  {
    pm->cal = 0;
  }
}

int32_t position_get_angle(position_manager_t *pm)
{
  return pm->angle;
}

int32_t position_get_angle_mod2pi(position_manager_t *pm)
{
  return pm->angle_mod_2pi;
}

fxx position_get_angle_mod2pi_deg(position_manager_t *pm)
{
  return fxx_mul(fxx_from_double(pm->angle_mod_2pi),pm->c_imp2deg);
}
fxx position_get_angle_deg(position_manager_t *pm)
{
  return fxx_mul(fxx_from_double(pm->angle),pm->c_imp2deg);
}

int32_t position_get_vitesse_angulaire(position_manager_t *pm)
{
  return pm->vitesse_angle;
}

fxx position_get_vitesse_angulaire_degs(position_manager_t *pm)
{
  return fxx_div(fxx_mul(fxx_from_double(pm->vitesse_angle),pm->c_imp2deg),pm->c_delta_call);
}

int32_t position_get_distance(position_manager_t *pm)
{
  return pm->distance;
}
fxx position_get_distance_cm(position_manager_t *pm)
{
  return fxx_mul(fxx_from_double(pm->distance),pm->c_imp2cm);
}

int32_t position_get_vitesse(position_manager_t *pm)
{
  return pm->vitesse_distance;
}

fxx position_get_vitesse_ms(position_manager_t *pm)
{
  return fxx_div(fxx_mul(fxx_from_double(pm->vitesse_distance),pm->c_imp2cm),pm->c_delta_call);
}

int32_t position_get_x(position_manager_t *pm)
{
  return pm->x;
}

int32_t position_get_y(position_manager_t *pm)
{
  return pm->y;
}

fxx position_get_x_cm(position_manager_t *pm)
{
  return fxx_mul(fxx_from_double(pm->x),pm->c_imp2cm);
}

fxx position_get_y_cm(position_manager_t *pm)
{
  return fxx_mul(fxx_from_double(pm->y),pm->c_imp2cm);
}

// renvoie la position courante dans le graphe
uint8_t position_get_coor(position_manager_t *pm)
{
      double posx = pm->y * pm->c_imp2cm ;
      double posy = pm->x * pm->c_imp2cm ;
            
      // Calcule la coordonné la plus proche d'un noeud
      uint8_t shift_x = ((int)posx) % UNIT >= UNIT/2;
      uint8_t shift_y = ((int)posy) % UNIT >= UNIT/2;
      
      //Now the value of x and y is in UNIT
      uint8_t x = posx / UNIT + shift_x;
      uint8_t y = posy / UNIT + shift_y;
      
      return x + G_LENGTH * y;
}

// renvoie la position courante dans le graphe
uint8_t position_get_coor_eps(position_manager_t *pm, double *eps)
{
      double posx = pm->y * pm->c_imp2cm ;
      double posy = pm->x * pm->c_imp2cm ;
            
      // Calcule la coordonné la plus proche d'un noeud
      uint8_t shift_x = ((int)posx) % UNIT >= UNIT/2;
      uint8_t shift_y = ((int)posy) % UNIT >= UNIT/2;
      
      //Now the value of x and y is in UNIT
      uint8_t x = posx / UNIT + shift_x;
      uint8_t y = posy / UNIT + shift_y;
      
      *eps = fabs(posx - (double)x*UNIT) + fabs(posx - (double)x*UNIT);

      return x + G_LENGTH * y;
}


int32_t position_cm2imp(position_manager_t *pm,fxx cm)
{
  return fxx_to_double(fxx_div(cm,pm->c_imp2cm));
}

int32_t position_deg2imp(position_manager_t *pm,fxx deg)
{
  return fxx_to_double(fxx_div(deg,pm->c_imp2deg));
}
int32_t position_rad2imp(position_manager_t *pm,fxx rad)
{
  return position_deg2imp(pm,fxx_div(fxx_mul(rad,fxx_from_double(180)),fxx_from_double(M_PI)));
}


void position_set_x_cm(position_manager_t *pm,double x_cm)
{
  fxx x = fxx_from_double(x_cm); 
  pm->x = fxx_div(x,pm->c_imp2cm);
}

void position_set_y_cm(position_manager_t *pm,double y_cm)
{
  fxx y = fxx_from_double(y_cm);
  pm->y = fxx_div(y,pm->c_imp2cm);
}


void position_set_angle_deg(position_manager_t *pm,double deg)
{
 fxx a = fxx_from_double(deg);
  pm->angle = a;
}
