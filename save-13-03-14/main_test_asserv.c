

#include <aversive.h>
#include <wait.h>
#include <time.h>
#include <uart.h>
#include <scheduler.h>
#include <i2cm.h>
#include <adc.h>
#include <math.h>
#include "unioc_config.h"
#include "com.h"
#include "position_manager.h"
#include "asserv_manager.h"
#include "trajectory_manager.h"
#include "asserv_algo.h"


#define LARGEUR_ROBOT 30.0
#define get_fdc 0x16

#define R_ENC 20  // Rapport de valeur entre les encodeurs internes et externes


// Prototypes //////////////////////////////////////////
void init_couleur_depart(void);
void initCom(void);
uint8_t mecaCom(uint8_t ordre);
int8_t get_FdCourse_avant(void);
int8_t get_FdCourse_arriere(void);
void rdsInit(void);
int getDistance(uint16_t code);
void adversary_detection(void);
uint8_t get_adversary_position(void); // attention, l'information "devant", "derriere" etc ne sont pas tout a fait fiable dans le sens ou la fonction renvoie "devant" si elle ne sait pas si c'est devant, derriere etc mais qu'elle sait qu'il y a quelque chose pas très loin (i.e plus de 2 TSOP allumés)
void detection_adversaire(void *);
void goto_d_stop_skating(trajectory_manager_t * traj, double d);
void goto_d_back_detection(trajectory_manager_t * traj, double d);
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//


trajectory_manager_t traj;
int8_t mode_fonctionnement;
extern int8_t patinage;

int main(void)
{
	//Declaration des variables//////////
	position_manager_t pos; 
	asserv_manager_t asserv;
	

	int i = 0;

	/////////////////////////////////////

	//Initialisations////////////////////
	uart_init();
	time_init(128);
	fdevopen(uart0_send,NULL, 0);
	/***** tip -s 38400 -l /dev/ttyS1 *****/

	//adc_init();
	DDRE |= 0xf0;

	//initialisation du fpga
	sbi(XMCRA,SRW11);
	sbi(XMCRA,SRW00);
	sbi(MCUCR,SRE);
	wait_ms(1000);

	//on reset le fpga
	U_RESET = 0x42;
	sbi(PORTB,0);
	wait_ms(100);

	//on relache le reset
	cbi(PORTB,0);
	sbi(PORTE,3);
	wait_ms(100);
 
	//init des interruptions (detection)
	scheduler_init();

	position_init(&pos);
	asserv_init(&asserv,&pos);
	trajectory_init(&traj,&pos,&asserv);
	sei();  // autorise les interruptions

	while(1) {

	  printf("Starting movement\n");
	  trajectory_goto_d(&traj, END, 100);
	  
	  while(!trajectory_is_ended(&traj));
	  
	  int32_t end_pos = position_get_distance(&pos);
	  printf("distance %d", &int32_t end_pos);

	  wait_ms(100);
	  
	  while(1);
	  
	}
}
