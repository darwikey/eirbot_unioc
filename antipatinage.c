  #include "antipatinage.h"
#include "scheduler.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "unioc_config.h"
#include "trajectory_manager.h"
#include "asserv_manager.h"
#include "task_manager.h"

// valeur booléenne à vraie lors d'un patinage
uint8_t patinage = 0;
extern trajectory_manager_t traj;
extern asserv_manager_t asserv;
s08 id = 0;

uint8_t ratio = 60;

void antipatinage_init(void)
{
  // id = scheduler_add_periodical_event(antipatinage_scheduler,NULL, 1000000/SCHEDULER_UNIT);
  // id = __scheduler_add_event(SCHEDULER_PERIODICAL, antipatinage_scheduler, NULL, 10000/SCHEDULER_UNIT, 2);
}


void antipatinage_scheduler(void) 
{
  //scheduler_del_event(id);
  //printf("...antipatinage...\n");

  //const int32_t epsEnc = 200;
  //const int32_t epsMoteur = 400;

  static int32_t old_encodeur_gauche = 0.0;
  static int32_t old_encodeur_droit = 0.0;
  static int32_t old_moteur_gauche = 0.0;
  static int32_t old_moteur_droit = 0.0;
  
  int32_t encodeur_gauche = U_ENC0;
  int32_t encodeur_droit = U_ENC2;
  int32_t moteur_gauche = U_ENC1;
  int32_t moteur_droit = U_ENC3;

  int32_t ratio_droit = abs((moteur_droit - old_moteur_droit));
  int32_t ratio_gauche = abs((moteur_gauche - old_moteur_gauche));

  int8_t avance = 0;
  


  static uint8_t k = 0;

  //printf("diffencD %ld    diffmotD %ld    diffencG %ld    diffmotG %ld\n", encodeur_droit - old_encodeur_droit, moteur_droit - old_moteur_droit, encodeur_gauche - old_encodeur_gauche, moteur_gauche - old_moteur_gauche);

  printf("encD %ld    motD %ld    encG %ld    motG %ld\n", encodeur_droit, moteur_droit, encodeur_gauche, moteur_gauche);   
  //wait_ms(10);
  //printf("ratio Droit %ld ratio Gauche %ld \n",ratio_droit,ratio_gauche);

  //if(abs(encodeur_gauche - old_encodeur_gauche) < epsEnc && abs(moteur_gauche -old_moteur_gauche) > epsMoteur)
  if(ratio_gauche > RATIO * abs(encodeur_gauche - old_encodeur_gauche) && ratio_gauche >DIST_MIN)
  {
    printf("PATINAGE gauche!!!!\n");
    // printf("ratio Droit %ld ratio Gauche %ld \n",ratio_droit,ratio_gauche);
    // printf("encD %ld    motD %ld    encG %ld    motG %ld\n", encodeur_droit, moteur_droit, encodeur_gauche, moteur_gauche);
    printf("diffencD %d    diffmotD %d    diffencG %d    diffmotG %d\n", abs(encodeur_droit - old_encodeur_droit), abs((moteur_droit - old_moteur_droit)), abs(encodeur_gauche - old_encodeur_gauche), abs((moteur_gauche - old_moteur_gauche)));
    k++;
    if(moteur_gauche - old_moteur_gauche > 0)
    {
      avance = 1;
    }
  }//abs(encodeur_droit - old_encodeur_droit) > 2
  else if(ratio_droit > RATIO * abs(encodeur_droit - old_encodeur_droit) && ratio_gauche >DIST_MIN)
  { 
    printf("PATINAGE droite!!!!\n");
    // printf("ratio Droit %ld ratio Gauche %ld \n",ratio_droit,ratio_gauche);
    printf("diffencD %d    diffmotD %d    diffencG %d    diffmotG %d\n", abs(encodeur_droit - old_encodeur_droit), abs((moteur_droit - old_moteur_droit)), abs(encodeur_gauche - old_encodeur_gauche), abs((moteur_gauche - old_moteur_gauche)));
    k++;
    if(moteur_droit - old_moteur_droit > 0)
    {
      avance = 1;
    }
  }
  else
  {
    k = 0;
  }

  if(k > 3)patinage = 1;


  if(patinage)  
  {
    trajectory_reinit(&traj);
    asserv_stop(&asserv);
    if(avance)
    {
      trajectory_goto_d(&traj, END, -10);
    }
    else
    {
      trajectory_goto_d(&traj, END, 10); 
    }
    //while(traj.last != traj.current);
    //trajectory_reinit(&traj);


    actionFailed();
    patinage = 0;
  }

  old_encodeur_gauche = encodeur_gauche;
  old_encodeur_droit = encodeur_droit;
  old_moteur_gauche = moteur_gauche;
  old_moteur_droit = moteur_droit;
  
  antipatinage_init(); 
}

uint8_t roue_patine(void)
{
  return patinage;
}
