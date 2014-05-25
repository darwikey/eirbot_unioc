/////////////////////////////////////////////////////////////////////////////////
//            ASSERV
// 
// Derniere modification:
// Vincent, le 17/05/2011 a 20h
// 
// Commentaire: 
// ajustement des coef d'asserv => ok
//
// 
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


#include <stdio.h>
#include <math.h>
#include <scheduler.h>
#include "asserv_manager.h"
//#include <wait.h>

void asserv_update_low_level(void * p);

void asserv_init_gain(asserv_manager_t * t)
{
//TEST DISTANCE: 1m
//valeurs trouvees:  1000 0     50
//                   0    0     2500  
//                   11

//                   800  0     50    
//                   0    0     2500  
//                   11              

//                   900  10    800   
//                   0    10000 5000  
//                   8

//                   170  10   800   
//                   0    1000 1500  
//                   9               

  pid_set_gains(&t->pid_motD, 1000, 0, 0);//1000, 0, 50);//1000, 0, 0 
  pid_set_maximums(&t->pid_motD, 0, 0, 2500);//0, 0, 2500
  pid_set_out_shift(&t->pid_motD, 11);// 11

  pid_set_gains(&t->pid_motG, 1000, 0, 0);//900, 0, 50);//1000, 0, 0
  pid_set_maximums(&t->pid_motG, 0, 0, 2500);//0, 0, 2500
  pid_set_out_shift(&t->pid_motG, 11);//11

  pid_set_gains(&t->pid_distance, 1000, 10, 800);//900, 10, 800);//1000,10,800
  pid_set_maximums(&t->pid_distance, 0, 10000,5000);//0,10000,5000
  pid_set_out_shift(&t->pid_distance, 8);//8


  pid_set_gains(&t->pid_angle, 100, 30, 600);//100,30,800
  pid_set_maximums(&t->pid_angle, 0, 1000, 1500);//0,1000,1500
  pid_set_out_shift(&t->pid_angle, 9);//9


  //pid_set_derivate_filter(&t->pid_distance,4);

  quadramp_set_2nd_order_vars(&t->qramp_distance, 3, 3 ) ;//6,6 // Acceleration
  quadramp_set_1st_order_vars(&t->qramp_distance, 50, 50);//70,70 // Vitesse

  quadramp_set_2nd_order_vars(&t->qramp_angle, 4, 4);//4,4
  quadramp_set_1st_order_vars(&t->qramp_angle, 200, 200);//400,400


}

void asserv_left_pwm(void* p, int32_t val)
{
  /*if(val > 600)
    {
      val = 600;
    }
  if(val < -600)
    {
      val = -600;
      }*/
  U_MOT_G_RATIO = val;
}
void asserv_right_pwm(void* p, int32_t val)
{
  /*if(val > 600)
    {
      val = 600;
    }
  if(val < -600)
    {
      val = -600;
      }*/

  U_MOT_D_RATIO = -val;
}
int32_t asserv_left_mot_encoder(void* p)
{
  return ((int32_t)U_ENC1);
}
int32_t asserv_right_mot_encoder(void* p)
{
  return ((int32_t)U_ENC3);
}
int32_t asserv_get_distance(void *p)
{
  asserv_manager_t *t = p;
  //printf("test %ld %p\n",position_get_distance(t->pos_man),t);
  return position_get_distance(t->pos_man);
}
int32_t asserv_get_angle(void *p)
{
  asserv_manager_t *t = p;
  return position_get_angle(t->pos_man);
}


void asserv_init(asserv_manager_t * t, position_manager_t * p)
{
  //moteurs a 20kHz
  U_MOT_D_PERIODE = 2500;
  U_MOT_G_PERIODE = 2500;
  U_MOT_FLAGS = U_MOT_MODE_ATMEGA;


  t->pos_man = p;

  pid_init (&t->pid_motD);

  
  pid_init (&t->pid_motG);


  diff_init(&t->diff_motD_retour);
  diff_set_delta(&t->diff_motD_retour, 1);// TIMER_UNIT * LOW_ASSERV_DELTA);

diff_init(&t->diff_motG_retour);
  diff_set_delta(&t->diff_motG_retour, 1);//TIMER_UNIT * LOW_ASSERV_DELTA);

diff_init(&t->diff_motD_consigne);
  diff_set_delta(&t->diff_motD_consigne, 1);//TIMER_UNIT * LOW_ASSERV_DELTA);

diff_init(&t->diff_motG_consigne);
  diff_set_delta(&t->diff_motG_consigne, 1);//TIMER_UNIT * LOW_ASSERV_DELTA);

//  intg_init(&t->intg_distance);
//  intg_set_delta(&t->intg_distance, 1);//TIMER_UNIT * LOW_ASSERV_DELTA);

cs_init(&t->csm_motD);
cs_init(&t->csm_motG);

  // Filtre en consigne
cs_set_consign_filter(&t->csm_motD, NULL, NULL);
cs_set_consign_filter(&t->csm_motG, NULL, NULL);

  // Filtre en correction
cs_set_correct_filter(&t->csm_motD, &pid_do_filter, &t->pid_motD);
cs_set_correct_filter(&t->csm_motG, &pid_do_filter, &t->pid_motG);

  // Filtre de retour
cs_set_feedback_filter(&t->csm_motD, &diff_do_filter, &t->diff_motD_retour);
cs_set_feedback_filter(&t->csm_motG, &diff_do_filter, &t->diff_motG_retour); 

  // Process out
cs_set_process_out(&t->csm_motD, &asserv_right_mot_encoder, &t);
cs_set_process_out(&t->csm_motG, &asserv_left_mot_encoder, &t);

  // Process in
cs_set_process_in(&t->csm_motD, &asserv_right_pwm,  NULL);
cs_set_process_in(&t->csm_motG, &asserv_left_pwm, NULL);


quadramp_init(&t->qramp_distance);
quadramp_init(&t->qramp_angle);


  //test
  //quadramp_set_2nd_order_vars(&t->qramp_distance, 10, 5); // 3 3 
  //quadramp_set_1st_order_vars(&t->qramp_distance, 5, 5); // 1200 500


pid_init (&t->pid_distance);

//  pid_set_gains(&t->pid_distance, 500, 0, 200);
pid_init (&t->pid_angle);
cs_init(&(t->csm_distance1));
cs_init(&t->csm_distance2);
cs_init(&t->csm_angle1);
cs_init(&t->csm_angle2);

  // Filtre en consigne
cs_set_consign_filter(&(t->csm_distance1), NULL, NULL);
cs_set_consign_filter(&t->csm_distance2, NULL, NULL);
cs_set_consign_filter(&t->csm_angle1, NULL, NULL);
cs_set_consign_filter(&t->csm_angle2, NULL, NULL);

  // Filtre en correction
cs_set_correct_filter(&(t->csm_distance1), &quadramp_do_filter, &t->qramp_distance);
cs_set_correct_filter(&t->csm_distance2, &pid_do_filter, &t->pid_distance);
cs_set_correct_filter(&t->csm_angle1, &quadramp_do_filter, &t->qramp_angle);
cs_set_correct_filter(&t->csm_angle2, &pid_do_filter, &t->pid_angle);

  // Filtre de retour
cs_set_feedback_filter(&(t->csm_distance1), NULL, NULL);
cs_set_feedback_filter(&t->csm_distance2, NULL, NULL);
cs_set_feedback_filter(&t->csm_angle1, NULL, NULL);
cs_set_feedback_filter(&t->csm_angle2, NULL, NULL);

  // Process out
cs_set_process_out(&(t->csm_distance1), NULL,NULL);
cs_set_process_out(&t->csm_distance2, &asserv_get_distance, (void*)t);
cs_set_process_out(&t->csm_angle1, NULL, NULL);
cs_set_process_out(&t->csm_angle2, &asserv_get_angle, (void*)t);

  // Process in
cs_set_process_in(&(t->csm_distance1), NULL,  NULL);
cs_set_process_in(&t->csm_distance2, NULL,  NULL);
cs_set_process_in(&t->csm_angle1, NULL, NULL);
cs_set_process_in(&t->csm_angle2, NULL, NULL);


asserv_init_gain(t);

asserv_set_distance(t,0);
asserv_set_angle(t,0);


t->no_angle = 0;

static int8_t is_scheduler = 0;
if (is_scheduler == 0)
{
  scheduler_add_periodical_event(&asserv_update_low_level,(void*)t,ROBOT_ASSERV_UPDATE_TIME/SCHEDULER_UNIT);
  is_scheduler = 1;
}
//asserv_right_pwm(0,1000);
  //asserv_left_pwm(0, 1000);

}
void asserv_send_motors_consigns(asserv_manager_t * t,int32_t d, int32_t theta){

#ifndef MOT_TRACK_CM
#error MOT_TRACK_CM was not declared, please include trajectory_manager_config.h
#endif

  int32_t d_D,d_G;

  //d = 0;

  //theta = 0;
  d_D = d + (MOT_TRACK_CM*theta)/2;
  d_G = d - (MOT_TRACK_CM*theta)/2;


  //printf("d=%ld a=%ld mD=%ld mG=%ld\n",d,theta,d_D,d_G);
  cs_set_consign(&t->csm_motD, d_D);
  cs_set_consign(&t->csm_motG, d_G);

  return;
}


void asserv_update_low_level(void * p)
{
  //printf("asserv update\n");

  asserv_manager_t *t = p;
  
  cs_manage(&(t->csm_distance1));
  cs_set_consign(&t->csm_distance2, cs_get_out(&(t->csm_distance1)));
  
  if(t->no_angle == 0)
  {
    cs_manage(&t->csm_angle1);
    cs_set_consign(&t->csm_angle2, cs_get_out(&t->csm_angle1));
  }
  // Integration &t-> PID
  cs_manage(&t->csm_distance2);

  // Angle
  cs_manage(&t->csm_angle2);

  //printf("in %ld  err %ld out %ld\n",cs_get_consign(&t->csm_angle2),cs_get_error(&t->csm_angle2),cs_get_out(&t->csm_angle2));
  //printf("in %ld  err %ld out %ld\n",cs_get_consign(&t->csm_distance2),cs_get_error(&t->csm_distance2),cs_get_out(&t->csm_distance2));
  asserv_send_motors_consigns(t,cs_get_out(&t->csm_distance2), cs_get_out(&t->csm_angle2));

  cs_manage(&t->csm_motD);
  cs_manage(&t->csm_motG);
  sbi(PORTE,4);
}

void asserv_set_distance(asserv_manager_t *t,int32_t d)
{
  cs_set_consign(&(t->csm_distance1),d);
}
void asserv_set_angle(asserv_manager_t *t,int32_t a)
{
  if(t->no_angle == 1)
  {
    t->no_angle = 0;
    //quadramp_reset(&t->qramp_angle,a); 
  }
  cs_set_consign(&(t->csm_angle1),a);
}
void asserv_set_no_angle(asserv_manager_t *t)
{
  t->no_angle = 1;
}
void asserv_stop(asserv_manager_t *t)
{
  cs_set_consign(&(t->csm_distance1),position_get_distance(t->pos_man));
  cs_set_consign(&t->csm_angle1,position_get_angle(t->pos_man));
  U_MOT_G_RATIO = 0;
  U_MOT_D_RATIO = 0;
}
void asserv_set_vitesse_normal(asserv_manager_t *t)
{
  quadramp_set_2nd_order_vars(&t->qramp_distance, 3, 3); // 3 3 
  quadramp_set_1st_order_vars(&t->qramp_distance, 50, 50); // 1200 500

 quadramp_set_2nd_order_vars(&t->qramp_angle, 4, 4);//4,4
  quadramp_set_1st_order_vars(&t->qramp_angle, 150, 150);//200,200
}

void asserv_set_vitesse_fast(asserv_manager_t *t)
{
  quadramp_set_2nd_order_vars(&t->qramp_distance, 3, 3); // 3 3 
  quadramp_set_1st_order_vars(&t->qramp_distance, 100, 100); // 1200 500

 quadramp_set_2nd_order_vars(&t->qramp_angle, 4, 4);//4,4
  quadramp_set_1st_order_vars(&t->qramp_angle, 300, 300);//200,200

}
void asserv_set_vitesse_ultrafast(asserv_manager_t *t)
{
  quadramp_set_2nd_order_vars(&t->qramp_distance, 3000, 3000); // 3 3 
  quadramp_set_1st_order_vars(&t->qramp_distance, 10000, 10000); // 1200 500

}


void asserv_set_vitesse_low(asserv_manager_t *t)
{
  quadramp_set_2nd_order_vars(&t->qramp_distance, 1, 1); // 3 3 
  quadramp_set_1st_order_vars(&t->qramp_distance, 10, 10); // 1200 500

  quadramp_set_2nd_order_vars(&t->qramp_angle, 1, 1);//4,4
  quadramp_set_1st_order_vars(&t->qramp_angle, 20, 20);//400,400

}

void quadramp_reset(asserv_manager_t *t)
{
  t->qramp_angle.previous_var = 0;
  t->qramp_angle.previous_out = 0;
  t->qramp_angle.previous_in = 0;
  

  // t->qramp_distance.previous_var = 0;
  // t->qramp_distance.previous_out = 0;
  // t->qramp_distance.previous_in = 0;
}

void pid_reset(asserv_manager_t *t)
{
  // t->pid_distance.integral = 0;
  // t->pid_distance.last_in = 0;
  t->pid_angle.integral = 0;
  t->pid_angle.last_in = 0;
  t->pid_motD.integral = 0;
  t->pid_motD.last_in = 0;
  t->pid_motG.last_in = 0;
  t->pid_motG.integral = 0;
}

void control_reset(asserv_manager_t *t)
{
  // t->csm_distance1.consign_value = 0;
  // t->csm_distance1.error_value = 0;
  // t->csm_distance1.out_value = 0;
  // t->csm_distance1.filtered_consign_value = 0;

  // t->csm_distance2.consign_value = 0;
  // t->csm_distance2.error_value = 0;
  // t->csm_distance2.out_value = 0;
  // t->csm_distance2.filtered_consign_value = 0;

  t->csm_angle1.consign_value = 0;
  t->csm_angle1.error_value = 0;
  t->csm_angle1.out_value = 0;
  t->csm_angle1.filtered_consign_value = 0;


  t->csm_angle2.consign_value = 0;
  t->csm_angle2.error_value = 0;
  t->csm_angle2.out_value = 0;
  t->csm_angle2.filtered_consign_value = 0;
  

  t->csm_motD.consign_value = 0;
  t->csm_motD.error_value = 0;
  t->csm_motD.out_value = 0;
  t->csm_motD.filtered_consign_value = 0;

  t->csm_motG.consign_value = 0;
  t->csm_motG.error_value = 0;
  t->csm_motG.out_value = 0;
  t->csm_motG.filtered_consign_value = 0;

}

void diff_reset(asserv_manager_t *t)
{

  t->diff_motD_retour.last_in = 0;
  t->diff_motG_retour.last_in = 0;
  t->diff_motD_consigne.last_in = 0;
  t->diff_motG_consigne.last_in = 0;
  t->diff_motD_retour.out = 0;
  t->diff_motG_retour.out = 0;
  t->diff_motD_consigne.out = 0;
  t->diff_motG_consigne.out = 0;

}
/*void antipatinage(void) {
	//printf("...antipatinage...\n");
	static float old_encodeur_gauche = 0.0;
	static float old_encodeur_droit = 0.0;
	static float old_moteur_gauche = 0.0;
	static float old_moteur_droit = 0.0;

	float encodeur_gauche = (float)U_ENC1;
	float encodeur_droit = (float)U_ENC3;
	float moteur_gauche = (float)U_ENC0;
	float moteur_droit = (float)U_ENC2;

	float rapport_roues_gauches = (moteur_gauche - old_moteur_gauche) / (encodeur_gauche - old_encodeur_gauche);
	float rapport_roues_droites = (moteur_droit - old_moteur_droit) / (encodeur_droit - old_encodeur_droit);
        float diff_gauche = abs(rapport_roues_gauches - RAPPORT_STATIC_DERAPAGE);
        float diff_droite = abs(rapport_roues_droites - RAPPORT_STATIC_DERAPAGE);
 
	//printf("***************************************************\n");
	//printf("rapport droit : %f, rapport gauche : %f\n", (double)rapport_roues_droites, (double)rapport_roues_gauches);
	//printf("diff gauche : %f, diff droit : %f\n", (double)diff_gauche, (double)diff_droite);
	
	if ( (diff_gauche > TOLERANCE_DERAPAGE && diff_gauche < 1000000000 ) || (diff_droite > TOLERANCE_DERAPAGE && diff_droite < 1000000000) ) {
		patinage = 1;
		printf("PATIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIINE!!!\n");
	}
	else {
		//printf("Patine po...\n");
		patinage = 0;
	}
	//printf("***************************************************\n");

	/* Les valeurs new deviennent old pour le prochain appel de la fonction 
	old_encodeur_gauche = encodeur_gauche;
	old_encodeur_droit = encodeur_droit;
	old_moteur_gauche = moteur_gauche;
	old_moteur_droit = moteur_droit;
}*/


