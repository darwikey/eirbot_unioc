#ifndef ASSERV_MANAGER_H
#define ASSERV_MANAGER_H

#define ROBOT_ASSERV_UPDATE_TIME 15000

#include <aversive.h>
#include "unioc_config.h"
#include "position_manager.h"

#include  "diff.h"
#include  "pid.h"
#include  "quadramp.h"
#include  <control_system_manager.h>
// Distance entre les deux codeurs
#define   MOT_TRACK_CM (27.2) 
#define TOLERANCE_DERAPAGE 1.0
#define RAPPORT_STATIC_DERAPAGE 2.1504

int8_t patinage;

typedef struct 
{
  struct cs csm_distance1;
  struct cs csm_distance2;
  struct cs csm_angle1;
  struct cs csm_angle2;
  struct cs csm_motD;
  struct cs csm_motG;
  struct pid_filter pid_motD;
  struct pid_filter pid_motG;
  struct diff diff_motD_retour;
  struct diff diff_motG_retour;
  struct diff diff_motD_consigne;
  struct diff diff_motG_consigne;
//  struct intg intg_distance;
  struct pid_filter pid_distance;
  struct pid_filter pid_angle;
  struct quadramp_filter qramp_distance;
  struct quadramp_filter qramp_angle;
  position_manager_t *pos_man;

  uint8_t no_angle;
}asserv_manager_t;

void asserv_init(asserv_manager_t *,position_manager_t *p);

void asserv_set_distance(asserv_manager_t *,int32_t);
void asserv_set_angle(asserv_manager_t *,int32_t);
void asserv_stop(asserv_manager_t *);
void asserv_set_no_angle(asserv_manager_t *);
void asserv_set_vitesse_low(asserv_manager_t *);
void asserv_set_vitesse_normal(asserv_manager_t *);
void asserv_set_vitesse_ultrafast(asserv_manager_t *);
void antipatinage(void);

#endif //ASSERV_MANAGER_H
