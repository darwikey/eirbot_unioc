#include "antipatinage.h"
#include "scheduler.h"
#include <stdio.h>
#include <stdlib.h>
#include "unioc_config.h"
#include "trajectory_manager.h"

// valeur booléenne à vraie lors d'un patinage
uint8_t patinage = 0;
extern trajectory_manager_t traj;
s08 id = 0;

void antipatinage_init(void)
{
  id = scheduler_add_periodical_event(antipatinage_scheduler,NULL, 1000000/SCHEDULER_UNIT);

}


void antipatinage_scheduler(void* p) 
{
  scheduler_del_event(id);
  //printf("...antipatinage...\n");
  
  const int32_t epsEnc = 10;
  const int32_t epsMoteur = 100;
  
  static int32_t old_encodeur_gauche = 0.0;
  static int32_t old_encodeur_droit = 0.0;
  static int32_t old_moteur_gauche = 0.0;
  static int32_t old_moteur_droit = 0.0;
  
  int32_t encodeur_gauche = U_ENC0;
  int32_t encodeur_droit = U_ENC2;
  int32_t moteur_gauche = U_ENC1;
  int32_t moteur_droit = U_ENC3;

  printf("encD %ld    motD %ld    encG %ld    motG %ld\n", encodeur_droit, moteur_droit, encodeur_gauche, moteur_gauche);   

  if(abs(encodeur_gauche - old_encodeur_gauche) < epsEnc && abs(moteur_gauche -old_moteur_gauche) > epsMoteur)
    {
      printf("PATINAGE!!!!\n");
      patinage = 1;
    }
  if(abs(encodeur_droit - old_encodeur_droit) <  epsEnc && abs(moteur_droit -old_moteur_droit) > epsMoteur)
    {
      printf("PATINAGE!!!!\n");
      patinage = 1;
    }

  if(patinage)
    trajectory_pause(&traj);

  old_encodeur_gauche = encodeur_gauche;
  old_encodeur_droit = encodeur_droit;
  old_moteur_gauche = moteur_gauche;
  old_moteur_droit = moteur_droit;

  id = scheduler_add_periodical_event(antipatinage_scheduler,NULL, 1000000/SCHEDULER_UNIT);
}

uint8_t roue_patine(void)
{
  return patinage;
}
