/*
 * Principe des algo d'auto-asservissment :
 *
 *
 * tant que |ancienne qualité moyenne - nouvelle qualité moenne| > epsilon
 *
 *    utiliser les nouvelles valeurs de PID

 * 		faire plusieurs fois
 * 			faire effectuer au robot une trajectoire
 * 			calculer une valeur de "qualité" à partir de l'écart entre la position attendue du robot (là où on lui dit d'aller) et sa position réelle (donnée par le positionnement)
 * 			stocker cette valeur dans une "table"
 *
 * 		faire la moyenne des qualités calculées
 * 		calculer une nouvelle valeur de P(ID) en fonction de la qualité moyenne du mouvement
 *
 */


/* *
 * On teste P puis I puis D (I et D à 0 au départ)
 * Test de P : temps de réaction
 * Test de I : erreur statique
 * Test de D : oscillation
 * */

// Codeurs externes UENC0 et UENC2 . Codeurs internes UENC1 et UENC3

#include <aversive.h>
#include <scheduler.h>
#include "position_manager.h"
#include "asserv_manager.h"
#include "trajectory_manager.h"
#include <stdio.h>
#include "asserv_algo.h"

#define R_ENC 3.7  // Rapport de valeur entre les encodeurs internes et externes

#define ABSOLUTE_MAX_P 1000
#define ABSOLUTE_MIN_P 10

#define ABSOLUTE_MAX_I 40
#define ABSOLUTE_MIN_I 0

#define MAX_D 1000
#define MIN_D 0

#define MAX_I_ERROR 9.0// Une erreur de 9 signifie que le robot se trouve, après déplacement, dans un disque de rayon 3 cm autour de la position où on lui a dit d'aller 

#define PAUSE 3000

// #define SE 4000 /* Attention à quoi sert cette macro ? */
#define SKATE_MARGIN 100

#define NB_TEST 10

#define ACC_MAX 1.0 // Accélération max permises lors du mouvement

#define SCHEDULER_10MS 10000/SCHEDULER_UNIT //ca serait cool de mettre la vri valeur de scheduler unit
/* À ce qu'il paraît, cf. scheduler.h, 10001/SCHEDULER_UNIT correspond à 1ms !*/
/* À vérifier */
#define OSC_PERIOD_MS 1*SCHEDULER_10MS

struct pid_error{
	int16_t p;
	int16_t i;
	int16_t d;

	double e;
};

/**
 * 
 **/
int8_t oscillations = 0;

struct pid_error tested_value; 

void pid_e_init(int16_t p, int16_t i, int16_t d, double e)
{
	tested_value.p = p;
	tested_value.i = i;
	tested_value.d = d;
	tested_value.e = e;
}

/**
 * Renvoie 1 si le robot est bloqué (patine)
 * (i.e. les valeurs des encodeurs internes
 * et externes ne correspondent pas) 
 **/
int8_t is_skating(void)  
{
	int8_t flag = 0; 
	static int32_t uenc0old = 0, uenc2old = 0;
	static int32_t uenc1old = 0, uenc3old = 0;

	if ((R_ENC * (double)(ABS(U_ENC0-uenc0old)) < (double)(ABS(U_ENC1-uenc1old))) ||
			(R_ENC * (double)(ABS(U_ENC2-uenc2old)) < (double)(ABS(U_ENC3-uenc3old))))
		flag = 1;

	uenc0old = U_ENC0;
	uenc1old = U_ENC1;
	uenc2old = U_ENC2;
	uenc3old = U_ENC3;

	return flag;
}

/**
 * Renvoi 1 si l'accélération est supérieure à 1.0 cm.s⁻²
 * (hors début et fin de mouvement)
 * 0 sinon.
 * 
 * Une fois le robot lancé, sa vitesse est normalement constante
 * et son accélération nulle, pendant le mouvement.
 * Si ça vitesse varie trop, c'est qu'on a une valeur trop grande de P
 **/
int8_t is_oscillating(void * vtraj)
{
	static int8_t appelcounter = 0;
	static double xvalues[3] = {-1.0,-1.0,-1.0};
	static double yvalues[3] = {-1.0,-1.0,-1.0};
	trajectory_manager_t * traj = vtraj;

	appelcounter++;

	xvalues[0] = xvalues[1];  
	xvalues[1] = xvalues[2];  
	xvalues[2] = ABS(position_get_x_cm(traj->pm));
	yvalues[0] = yvalues[1];  
	yvalues[1] = yvalues[2];
	yvalues[2] = ABS(position_get_y_cm(traj->pm));

	//printf("x : %lf\n", xvalues[2]);
	//printf("y : %lf\n", yvalues[2]);

	if (appelcounter <= 3)
		return 0;

	//printf("%lf\n",xvalues[1]-xvalues[0]);
	//printf("%lf\n",xvalues[2]-xvalues[1]);
	double vold= ABS(xvalues[1] - xvalues[0]) / (OSC_PERIOD_MS / 1000.0);
	double vnew= ABS(xvalues[2] - xvalues[1]) / (OSC_PERIOD_MS / 1000.0);


	//printf("vnew : %lf\n", vnew);  
	//printf("vold : %lf\n\n", vold);


	double acc = (vnew - vold) / (OSC_PERIOD_MS / 1000.0);
	//printf("acc %lf\n", acc);

	if (acc > ACC_MAX)
	{
	  oscillations = 1;
	  //printf("acc %lf\n", acc);
		return 1;
	}

	return 0;
}

/** 
 * Calcul de l'erreur associée à l'action Proportionnelle :
 * cette fonction associe une erreur au trajet effectué par le robot
 * en fonction du temps qu'il a mis pour arriver à l'endroit prévu
 * on cherche à minimiser le temps de trajet, sans patinage
 **/
int8_t error_test_p(trajectory_manager_t * traj)
{
	int8_t i = 0;
	int8_t dist = 100;

	printf("Test : 10 trajets normalement\n");

	for (i = 0 ; i < 6 ; i++)
	{
		printf("tests numero:%d\n",i);
		trajectory_goto_d(traj, END, dist);
		wait_ms(1000);

		while(!trajectory_is_ended(traj))
		{
			wait_ms(100);

			
			if(is_skating()) 
			{
				printf("S\n");
				printf("%lf %lf\n", (double)U_ENC1 / (double)U_ENC0, (double)U_ENC3 / (double)U_ENC2);
				trajectory_pause(traj);

				trajectory_removeStep(traj);
				wait_ms(1000);
				return 1;
			}
		}

		wait_ms(PAUSE);
		dist *= -1;
	}

	return 0; 
}


int16_t pid_p_tester(asserv_manager_t * asserv, trajectory_manager_t * traj, position_manager_t * pos)
{

	int8_t skate = 0;
	int16_t min = ABSOLUTE_MIN_P, max = ABSOLUTE_MAX_P;
	int16_t p = (max-min)/2;  
	int16_t pold = p;

	do{
		pold = p;
		pid_set_gains(&asserv->pid_distance, p, 10/*(ABSOLUTE_MAX_I-ABSOLUTE_MIN_I)/2*/, 100/*(MAX_D-MIN_D)/2*/);
		skate = error_test_p(traj);

		if (skate || oscillations)
			max = p;
		else 
			min = p;

		p = (max + min)/2;
		wait_ms(PAUSE);

		printf("p trouve: %d\n", p);
	}while(ABS(pold - p) >= 10);

	tested_value.p = p;

	return p;                         
}

/** 
 * Calcul de l'erreur associée à l'action Intégrale :
 * cette fonction associe une erreur au trajet effectué par le robot
 * en fonction de la position à laquelle on lui a dit d'aller
 * et de sa position réelle
 **/
double eval_position_error(double x_ordered, double y_ordered, double x_real, double y_real)   // I
{
	double dx = x_ordered - x_real;
	double dy = y_ordered - y_real;

	return dx * dx + dy * dy;
}

/**
 * Cette fonction fait effectuer au robot un trajet prédéterminé
 * et retourne l'erreur associée à ce trajet
 **/
double error_test_i(trajectory_manager_t * traj)
{
	static int8_t a = 1;
	static int8_t b = 0;

	trajectory_goto_d(traj, END, a*100);

	while(!trajectory_is_ended(traj));
	a *=-1;
	b = ((b == 0) ? 1 : 0);

	return eval_position_error(b*100, 0, position_get_x_cm(traj->pm), position_get_y_cm(traj->pm));

}

/**
 * On effectue NB_TEST tests pour avoir une valeur d'erreur moyenne (plus significatif)
 * On évalue l'erreur associée à ces valeurs du PID en faisant un test sur l'asservissement du robot
 *
 * On met à jour, à chaque calcul de erreur, la valeur moyenne des erreurs calculées
 * Moyenne de n termes M(n) : M(n) = ((n-1)*M(n-1)+q)/n
 *
 *
 * On se laisse le temps de prendre le robot et de le remettre en position initiale à la main
 * Si on ne fait pas ça, il va rentrer dans le mur
 **/
double mean_error_tests_i(trajectory_manager_t * traj)
{
	double mean_e = 0;
	double e;
	int16_t k;

	for(k = 0; k < NB_TEST; k++)
	{
		
		e = error_test_i(traj);
		printf("erreur %lf\n",e);
		mean_e = (k * mean_e + e) / (k + 1);  

		wait_ms(PAUSE);
	}
	printf("Moyenne de l'erreur: %lf\n",mean_e);
	return mean_e;
}

/**
 * Avant d'exécuter cette fonction, il faut faire un pid_e_init
 * Et pour avoir une valeur de e à donner à pid_e_init,
 * on peut faire un error_test avant.
 * 
 * On cherche le plus petit i tq la precision soit bonne.
 **/
int8_t work_out_i_best_value(asserv_manager_t* asserv, trajectory_manager_t* traj)
{
	double e ;
	int8_t i = (ABSOLUTE_MAX_I - ABSOLUTE_MIN_I)/2;
	int8_t iold = i;
	int8_t min = ABSOLUTE_MIN_I;
	int8_t max = ABSOLUTE_MAX_I;
	//pid_e_init(tested_value.p,i, 0, MIN_D);

	do{
		iold = i;

		pid_set_gains(&asserv->pid_distance, 900, i, 100);

		//printf("10 tests avec i %d\n",i);
		e = mean_error_tests_i(traj);

		if (e >= MAX_I_ERROR) /*erreur grande*/
			min = i;
		else if (e <= MAX_I_ERROR)
			max = i;

		i = (max + min)/2;

		printf("i trouve: %d\n",i);
	}while(ABS(iold - i ) >= 5);

	return i;
}

/*
	 void useless_test(asserv_manager_t * asserv,
	 trajectory_manager_t * traj,
	 position_manager_t * pos)
	 {
	 double e = 0;
0
	 pid_set_gains(&asserv->pid_distance, 2000, 10, 100);

	 trajectory_goto_d(traj, END, 50);

	 while(!trajectory_is_ended(traj));

	 e = eval_position_error(50, 0, position_get_x_cm(traj->pm), position_get_y_cm(traj->pm));

	 wait_ms((int32_t)(100000000.0 * e));

	 trajectory_goto_d(traj, END, 50);

	 while(!trajectory_is_ended(traj));
	 while(1);
	 }
	 */

void useless_test(asserv_manager_t *	asserv, 
		trajectory_manager_t *	traj, 
		position_manager_t *	pos)
{
	pid_set_gains(&asserv->pid_distance, 1000, 10, 100);

	scheduler_add_periodical_event(&is_oscillating,(void *)traj, OSC_PERIOD_MS);

	trajectory_goto_d(traj,END,100);

	while(1);

	while(!trajectory_is_ended(traj))
	{
		if (oscillations)
		{
			printf("osc\n");
		}
	}
	//int16_t p = pid_p_tester(asserv,traj, pos);
}
