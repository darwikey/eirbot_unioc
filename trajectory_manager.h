///////////////////////////////////////////////////////////////////////
// fonction trajectory_goto_reculon_xy ajouté
//
//////////////////////////////////////////////

#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#undef TRAJECTORY_DEBUG

//il est recommande de prendre une puissance de 2 pour facilite les modulos
#define TRAJECTORY_NB_POINT_MAX 16

#include "fxx.h"
#include "position_manager.h"
#include "asserv_manager.h"
//#include "notification_manager.h"

//distance en cm de precision finale pour tous les goto sauf goto_a 
#define DEFAULT_PREC_DISTANCE (1.0)
//ecart angulaire (precision finale) en deg pour u goto_a
#define DEFAULT_PREC_ANGLE (1.0)
//ecart angulaire (precision finale) en deg pour un xy ou un xya
#define DEFAULT_PREC_ANGLE_P (1.0)


#define ALREADY_INIT 1

typedef enum 
{
  D, AREL, A, XY, XYA, WAIT, STEP, D_NOA, XYPION, XYBACK, XYPIONBACK
}trajectory_order_type_t;

typedef enum
{
  END = -2, 
  BEFORE_LAST_STEP = -1,
  NOW = 0
}trajectory_order_when_t;

typedef struct 
{
  union
  {
    struct
    {
      int32_t x;
      int32_t y;
      int32_t a;
      uint32_t prec_xy;
      uint32_t prec_a;
      uint32_t prec_ap;
    }xya;

    struct
    {
      int32_t x;
      int32_t y;
      uint32_t prec_xy;
      uint32_t prec_ap;
    }xy;

    struct 
    {
      int32_t a;
      uint32_t prec_a;
    }a;

    struct
    {
      int32_t d;
      uint32_t prec_d;
    }d;
  };
  int32_t startingD;
  int32_t startingA;
  uint8_t flags;
  trajectory_order_type_t type;
}trajectory_dest_t;

typedef struct 
{
  trajectory_dest_t points[TRAJECTORY_NB_POINT_MAX];
  uint32_t current;
  uint32_t last;

  position_manager_t *pm;
  asserv_manager_t *ass;
  //notification_manager_t *not;

  // EIRBUG
  //// Utile pour GOTO_XY
  uint8_t directed_flag;
  ////
}trajectory_manager_t;

void trajectory_init(trajectory_manager_t *,position_manager_t *,asserv_manager_t *);//,notification_manager_t *not);
void trajectory_reinit(trajectory_manager_t *);

//EIRBUG
int8_t trajectory_goto_pos(trajectory_manager_t*, trajectory_order_when_t when, int16_t x, int16_t y, int16_t a);
////

int8_t trajectory_goto_d(trajectory_manager_t *,trajectory_order_when_t when,double d);
int8_t trajectory_goto_d_noa(trajectory_manager_t *,trajectory_order_when_t when,double d);
int8_t trajectory_goto_a(trajectory_manager_t *,trajectory_order_when_t when,double a);
int8_t trajectory_goto_arel(trajectory_manager_t *,trajectory_order_when_t when,double a);
int8_t trajectory_goto_xy(trajectory_manager_t *,trajectory_order_when_t when,double x,double y);
void trajectory_goto_reculon_xy(trajectory_manager_t *, trajectory_order_when_t chen, double x, double y);//Nouvelle fonction pour se déplacer plus simplement à reculons
int8_t trajectory_goto_xya(trajectory_manager_t *,trajectory_order_when_t when,double x,double y,double a);
int8_t trajectory_goto_step(trajectory_manager_t *,trajectory_order_when_t when);
int8_t trajectory_goto_wait(trajectory_manager_t *,trajectory_order_when_t when);
void trajectory_tick(void *);
void trajectory_pause(trajectory_manager_t *);
void trajectory_resume(trajectory_manager_t *);
uint8_t trajectory_is_paused(trajectory_manager_t *);
uint8_t trajectory_is_ended(trajectory_manager_t *t);
void trajectory_removeStep(trajectory_manager_t *);

int8_t get_couleur_depart(void);
void traj_set_couleur_depart(int8_t couleur);
// void trajectory_goto_xy_pion(trajectory_manager_t *, trajectory_order_when_t when, double x, double y);
// void trajectory_goto_xy_backward(trajectory_manager_t *t,trajectory_order_when_t when, double x, double y);
// void trajectory_goto_xy_pion_backward(trajectory_manager_t *t,trajectory_order_when_t when, double x, double y);
//corrige l'erreur des coordonnées pour ammener le pion au centre de la case
void disableSpinning(void);
void enableSpinning(void);

void trajectory_end(trajectory_manager_t* t);

#endif //TRAJECTORY_MANAGER_H
