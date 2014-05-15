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
#include "task_manager.h"
#include "asserv_algo.h"
#include "astar.h"
#include "avoidance.h"
#include "antipatinage.h"
#include <stdlib.h>

#define LARGEUR_ROBOT 30.0
#define get_fdc 0x16
#define DIST_CENTRE 12.25

// Interrupteur
#define REPOSITIONING (PINE & 0x10) // SORTIE 4
#define TIRETTE (PINE & 0x8) // SORTIE 3
#define SWITCH_TEAM (PINE & 0x40) // SOERTIE 6

// Marge d'erreur sur le positionnement lorsque le robot est  au centre d'un case
#define EPSILON 4.0

//verifier que startCoor a changer
//faire une deuxieme fonction d'evitement pour lorsque le robot est hors A* 


#define ENABLE_MECA
//Equipe
#define RED 1
#define YELLOW 0

//numero des feux a renverser
#define R1 1
#define R2 2
#define R3 3
#define Y1 4
#define Y2 5
#define Y3 6

//Endroit pour se recaller
#define YELLOWSTART 0
#define REDSTART 1
#define REDPAINT 2
#define YELLOWPAINT 3



/////////////////////////////////////////////////////
//YX|0__1__2__3__4__5__6__7__8__9__10_11_12_13_14_15|
//0 |1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1 |
//1 |1  _  1  1  1  1  1  _  _  1  1  1  1  1  _  1 |
//2 |1  _  1  1  1  1  1  _  _  1  1  1  1  1  _  1 |
//3 |1  _  _  _  R1 _  _  _  _  _  _  Y1 _  _  _  1 |
//4 |1  _  _  _  _  _  _  1  1  _  _  _  _  _  _  1 |
//5 |1  _  R2 _  _  _  1  1  1  1  _  _  _  Y2 _  1 |
//6 |1  1  _  _  _  _  1  1  1  1  _  _  _  _  _  1 |
//7 |1  1  _  _  R3 _  _  1  1  _  _  Y3 _  _  1  1 |
//8 |1  1  _  _  _  _  _  _  _  _  _  _  _  _  1  1 |
//9 |1  1  1  1  1  _  _  _  _  _  _  1  1  1  1  1 |
//10|1  1  1  1  1  1  1  1  1  1  1  1  1  1  1  1 |
////////////////////////////////////////////////////




#define MAXIME 0

/////////////////////////////////////
// Prototypes ///////////////////////
/////////////////////////////////////
void initCom(void);
uint8_t mecaCom(uint8_t ordre);
void test_evitement(void);

// Action
//retourne 0 si fail et 1 si reussi
uint8_t findPosition(uint8_t team);
uint8_t putFruit(uint8_t team);
uint8_t takeFruitYellow(uint8_t team);
uint8_t takeFruitRed(uint8_t team);
uint8_t putPaint(uint8_t team);
uint8_t throwSpears(uint8_t team);
uint8_t returnFire(uint8_t fireNb);
void stopAvoid(uint8_t fireNb);
void avoidFire(uint8_t fireNb);

uint8_t calibrateGP2(void);


uint8_t getTeam(void);
/////////////////////////////////////
// Variables globales
/////////////////////////////////////
trajectory_manager_t traj;
position_manager_t pos;
asserv_manager_t asserv;
task_manager_t tkm;


int8_t mode_fonctionnement;
extern int8_t patinage;

int8_t I2C_DISPO = 1;

uint8_t team;

//////////////////////////////////////
// Main
/////////////////////////////////////
int main(void)
{
	/////////////////////////////////////
	///TODO//////////////////////////////
	/////////////////////////////////////
	//(!!!)Debuger l'evitement 
	//			verifier les valeurs des gp2
	//			verifier l'emplacement des obstacles->(printGraphe)
	//Retirer les obstacles apres 5 secs
	//Rajouter des gp2 lateraux (vers l'avant du robot) et le code qui les fait marcher
	//(gp2 Arriere pour autoriser la marche arriere)
	//
	//(!!!)Mettre en place les lanceurs de balles
	//Faire le programme du lanceur de balle
	//			Deux fonctions lancer balles mammouth jaune et mammouth
	//Programmer la nouvelle carte Meca
	//
	//Faire une gestion des taches (sami)
	//
	//Rajouter des fonctions pour gerer les actions assez simples (sami)
	//
	//(!!!)Mettre le filet sur le robot principal (ou faire une PMI qui ne fait que lancer le filet)
	//
	//participer a la coupe
	/////////////////////////////////////




	/////////////////////////////////////
	//Initialisations////////////////////
	/////////////////////////////////////
	uart_init();
	time_init(128);
	printf("fdevopen \n");
	fdevopen(uart0_send,NULL, 0);
	// Affichage uart: tip -s 9600 -l /dev/ttyS0
	// prog usb : 
	// avarice -j /dev/ttyUSB0 --erase --program -f ./main.hex

	adc_init();
	DDRE |= 0x00;
	PORTE |= 0x10;//active le pull up

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
	

	//init des interruptions
	printf("init i2c \n");
	scheduler_init();


	position_init(&pos);
	asserv_init(&asserv,&pos);
	trajectory_init(&traj,&pos,&asserv);
	i2cm_init();
	//autorise les interruptions
	sei(); 

	//init communication
	#ifdef ENABLE_MECA
	printf("init com \n");
	initCom();
	printf("end init com \n");
	#endif

	//init A*
	initObstacle();

	//init méca
	#ifdef ENABLE_MECA
	mecaCom(TIROIR_FERMER);
	mecaCom(PEIGNE_FERMER);
	#endif
	
	team = getTeam();
	while(MAXIME);//boucle anti_maxime

	//	avoidance_init();
	//	antipatinage_init();

	// test_evitement();
	// while(1);




	if(team == RED)
	{
		findPosition(team);
		// attend que la tirette soit retirée
		while (!TIRETTE);

		//position_set_x_cm(&pos,20.0);
		//position_set_y_cm(&pos,20.0);
		enableSpinning();
		//position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(20.0), fxx_from_double(0.0));

		printf("begin match \n");	    

		printf("start pos: x %lf pos y %lf \n",position_get_x_cm(&pos),position_get_y_cm(&pos));

	      	// throwSpears(RED);	
	      	// returnFire(R1);
	      	// returnFire(R3);
	      	// returnFire(R2);	

	      	// takeFruitRed(team);
		putPaint(team);


		findPosition(REDPAINT);


		returnFire(Y1);
		throwSpears(YELLOW);
		putFruit(team);

	      	// disableSpinning();
	      	// findPosition(YELLOW);
	      	// enableSpinning();	      	

		returnFire(Y3);
		takeFruitYellow(team);

		putFruit(team);
		returnFire(Y2);




	}
	else // Team jaune
	{
		findPosition(team);
		// attend que la tirette soit retirée
		while (!TIRETTE);
		//position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(280.0), fxx_from_double(0.0));
		enableSpinning();

		printf("begin match \n");
		printf("start pos: x %lf pos y %lf \n",position_get_x_cm(&pos),position_get_y_cm(&pos));

		
		takeFruitYellow(team);          
		putFruit(team);

		disableSpinning();
		findPosition(RED);
		enableSpinning();
		takeFruitRed(team);
		putFruit(team);
		putPaint(team);

	}

	printf("time: %ld \n",time_get_s());

	while(1);//attention boucle infinie
}

void initCom(void) 
{
	uint8_t ordre;
	do 
	{
		while(i2cm_rcv(mecaADDR,1,&ordre)==0)
		{
			wait_ms(50);
			printf("i2cm_rcv!=0 \n");
		}
	}
	while(ordre != mecaReady);
}

//*** Com avec la Carte Meca***
uint8_t mecaCom(uint8_t ordre) 
{//BA2
	uint8_t result = -1;
   #ifdef ENABLE_MECA	
	I2C_DISPO = 0;

	i2cm_send(mecaADDR,1,&ordre);
	wait_ms(100);

	ordre = mecaBusy;
	uint8_t i = 0;
	while(ordre != mecaReady)
	{
		i2cm_rcv(mecaADDR,1,&ordre);
		if(ordre != mecaReady)
		{
		// C'est ici que l'on recoit effectivement la reponse de la meca

			result = ordre; 
		}
		wait_ms(50);

			// On affiche un message si le temps d'attente est trop long
		i++;
		if (i >= 20)
		{
			printf("Erreur de communication avec la carte méca\n");
		}
	}

	I2C_DISPO = 1;
   #endif
	return -result;

}

void test_evitement(void)
{
	disableSpinning();	

	findPosition(team);

	enableSpinning();	
	// trajectory_goto_a(&traj, END, 90);
	// trajectory_goto_d(&traj, END, -90);	
	// while(!trajectory_is_ended(&traj));
	// trajectory_goto_a(&traj, END, 0);	
	// while(!trajectory_is_ended(&traj));
	
	initTaskManager(&tkm);
	addTask(&tkm, &throwSpears, LOW_PRIORITY, RED);
	addTask(&tkm, &returnFire, LOW_PRIORITY, R1);
	addTask(&tkm, &returnFire, LOW_PRIORITY, R3);
	addTask(&tkm, &returnFire, HIGH_PRIORITY, R2);
	addTask(&tkm, &takeFruitRed, HIGH_PRIORITY, team);
	addTask(&tkm, &putPaint, HIGH_PRIORITY, team);

	addTask(&tkm, &findPosition, HIGH_PRIORITY, REDPAINT);
	addTask(&tkm, &returnFire, HIGH_PRIORITY, Y1);
	addTask(&tkm, &throwSpears, HIGH_PRIORITY, YELLOW);
	addTask(&tkm, &putFruit, HIGH_PRIORITY, team);

	//addTask(&tkm, &findPosition, HIGH_PRIORITY, YELLOW);

	addTask(&tkm, &returnFire, HIGH_PRIORITY, Y3);
	addTask(&tkm, &takeFruitYellow, HIGH_PRIORITY, team);
	addTask(&tkm, &putFruit, HIGH_PRIORITY, team);
	addTask(&tkm, &returnFire, HIGH_PRIORITY, Y2);
	while(doNext(&tkm));




		// trajectory_goto_d(&traj, END, 10);
		// while(!trajectory_is_ended(&traj));
		// printf("astar test \n");	    
		// set_startCoor(G_LENGTH * 2 + 2);
		// set_goalCoor(G_LENGTH * 8 + 13);
		// while(!astarMv());

		// trajectory_goto_d(&traj, END, -100);
		// while(!trajectory_is_ended(&traj));
	while(1);
			// trajectory_goto_arel(&traj, END, 180);
			// while(!trajectory_is_ended(&traj));
			// trajectory_goto_d(&traj, END, 100);
			// while(!trajectory_is_ended(&traj));


		//




	double x = fxx_to_double(position_get_y_cm(&pos));
	double y = fxx_to_double(position_get_x_cm(&pos));
	printf("x : %lf    y : %lf\n", x, y);
	printf("isOut  %d    obs %d\n", isOutOfGraphe(x, y), isObstacle(x, y));
		//go_to_node(x, y,graphe);

}

uint8_t findPosition(uint8_t type)
{
	disableSpinning();

	if(type == REDSTART)
	{
		asserv_set_vitesse_low(&asserv);

		trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);//dist centre = 12.2
		while(!trajectory_is_ended(&traj));
		trajectory_goto_arel(&traj, END, -90.0);
		while(!trajectory_is_ended(&traj));
		trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}


		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
		while(!trajectory_is_ended(&traj));
			//asserv_set_distance()

		cli();
		printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);

		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(20.0), fxx_from_double(0.0));
		wait_ms(100);
		printf("pos x : %ld pos y %ld \n",pos.x,pos.y);
		printf("pos sum x : %lf pos y %lf \n",pos.sum_x,pos.sum_y);
		asserv_stop(&asserv);//attention bug bizare avant
		asserv_set_vitesse_normal(&asserv);
	}
	else if(type == YELLOWSTART)// team == YELLOW
	{			
		asserv_set_vitesse_low(&asserv);

		trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);//dist centre = 12.2
		while(!trajectory_is_ended(&traj));
		trajectory_goto_arel(&traj, END, 90.0);
		while(!trajectory_is_ended(&traj));
		trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
		while(!trajectory_is_ended(&traj));
		cli();

		printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);

		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(280.0), fxx_from_double(0.0));
		asserv_stop(&asserv);

		asserv_set_vitesse_normal(&asserv);
	}
	else if(REDPAINT)
	{

		//asserv_set_vitesse_normal(&asserv);
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos, &eps));
		printf("start pos: %d eps %lf \n",position_get_coor_eps(&pos, &eps),eps);

		set_goalCoor(G_LENGTH * 1 + 7);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    


		while(!astarMv());

		trajectory_goto_a(&traj, END, 90);
		while(!trajectory_is_ended(&traj))
			trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
		while(!trajectory_is_ended(&traj));
		cli();

		printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);

		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(140.0), fxx_from_double(90.0));
		asserv_stop(&asserv);



	}
	else if(YELLOWPAINT)
	{

		//asserv_set_vitesse_normal(&asserv);
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos, &eps));
		printf("start pos: %d eps %lf \n",position_get_coor_eps(&pos, &eps),eps);

		set_goalCoor(G_LENGTH * 1 + 8);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    


		while(!astarMv());

		trajectory_goto_a(&traj, END, -90);
		while(!trajectory_is_ended(&traj))
			trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
		while(!trajectory_is_ended(&traj));
		cli();

		printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);

		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(160.0), fxx_from_double(-90.0));
		asserv_stop(&asserv);
	}
		//printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);

	quadramp_reset(&asserv);
	control_reset(&asserv);
	pid_reset(&asserv);
	diff_reset(&asserv);
	printf("pos x : %ld pos y %ld \n",pos.x,pos.y);
	printf("pos sum x : %lf pos y %lf \n",pos.sum_x,pos.sum_y);
	sei();

	enableSpinning();

	return DONE;
}

uint8_t takeFruitRed(uint8_t type)
{
	double eps = 0;
	set_startCoor(position_get_coor_eps(&pos, &eps));
	printf("start pos: %d eps %lf \n",position_get_coor_eps(&pos, &eps),eps);

	set_goalCoor(G_LENGTH * 5 + 2);

	if (eps > EPSILON)
	{
		go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
	}    


	while(!astarMv());

	printf("take the fruit");
	trajectory_goto_a(&traj, END, -90);
	while(!trajectory_is_ended(&traj));
	wait_ms(200);

	trajectory_goto_d(&traj, END,10);
	while(!trajectory_is_ended(&traj));

	mecaCom(TIROIR_OUVERT);
	mecaCom(PEIGNE_OUVERT);

	trajectory_goto_a(&traj, END, 0);
	while(!trajectory_is_ended(&traj));
	wait_ms(200);
	trajectory_goto_d(&traj, END,60);
	trajectory_goto_a(&traj, END,45);
	trajectory_goto_d(&traj, END,10 * 1.414);
		// trajectory_goto_a(&traj, END, 0);
		// trajectory_goto_d(&traj, END,10);
	trajectory_goto_a(&traj, END, 90);
	trajectory_goto_d(&traj, END,60);
	while(!trajectory_is_ended(&traj));
	mecaCom(PEIGNE_FERMER);

	trajectory_goto_a(&traj, END, 180);
	trajectory_goto_d(&traj, END,10);
	while(!trajectory_is_ended(&traj));
	return DONE;
}

uint8_t takeFruitYellow(uint8_t type)
{

	double eps = 0;
	set_startCoor(position_get_coor_eps(&pos,&eps));
	printf("start pos: %d eps %lf \n",position_get_coor_eps(&pos, &eps),eps);
	set_goalCoor(G_LENGTH * 8 + 9);

	if (eps > EPSILON)
	{
		go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
	}



	while(!astarMv());


	printf("take the fruit");

	mecaCom(PEIGNE_OUVERT);
	mecaCom(TIROIR_OUVERT);

	trajectory_goto_a(&traj, END, 0);
	while(!trajectory_is_ended(&traj));

	trajectory_goto_d(&traj, END, 8);
	while(!trajectory_is_ended(&traj));

	trajectory_goto_a(&traj, END, 90);
	while(!trajectory_is_ended(&traj));

	trajectory_goto_d(&traj, END, 80);
	while(!trajectory_is_ended(&traj));      


	trajectory_goto_a(&traj, END, 135);
	while(!trajectory_is_ended(&traj));

	trajectory_goto_d(&traj, END, 15 * 1.414);
	while(!trajectory_is_ended(&traj));

		// trajectory_goto_a(&traj, END, 90);
		// while(!trajectory_is_ended(&traj));

		// trajectory_goto_d(&traj, END, 12);
		// while(!trajectory_is_ended(&traj));


	trajectory_goto_a(&traj, END, 180);
	while(!trajectory_is_ended(&traj));

	trajectory_goto_d(&traj, END, 55);
	while(!trajectory_is_ended(&traj));

	trajectory_goto_a(&traj, END, -90);
	while(!trajectory_is_ended(&traj));

	trajectory_goto_d(&traj, END, 15);
	while(!trajectory_is_ended(&traj));

	mecaCom(PEIGNE_FERMER);
	return DONE;
}

uint8_t putFruit(uint8_t side)
{
	printf("begin take fruit");
	double eps = 0;
	set_startCoor(position_get_coor_eps(&pos,&eps));
	printf("start pos: %d eps %lf \n", position_get_coor_eps(&pos, &eps),eps);
	static uint8_t k = 0;

	if(team == RED)
	{
		set_goalCoor(G_LENGTH * 3 + 8);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		} 
		while(!astarMv());

		trajectory_goto_a(&traj, END, 180);      	     
		trajectory_goto_d(&traj, END, 15);
		if(k)
		{
			trajectory_goto_a(&traj, END, -90);	
		}
		else
		{
			trajectory_goto_a(&traj, END, -86);
		}

		trajectory_goto_d(&traj, END, -40);
		while(!trajectory_is_ended(&traj));

		mecaCom(TIROIR_DEVERSER);
		wait_ms(2000);

		mecaCom(TIROIR_FERMER);
		trajectory_goto_d(&traj, END, -60);
			//trajectory_goto_a(&traj, END, 0);
			//trajectory_goto_d(&traj, END,15);
		while(!trajectory_is_ended(&traj));
	}
	else
	{

		set_goalCoor(G_LENGTH * 3 + 7);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv());

		trajectory_goto_a(&traj, END, 180);

		if(k)
		{
			trajectory_goto_d(&traj, END, 4);
			trajectory_goto_a(&traj, END, -90);

		}
		else
		{
			trajectory_goto_a(&traj, END, -86);
		}

		trajectory_goto_d(&traj, END, 60);
		while(!trajectory_is_ended(&traj));

		mecaCom(TIROIR_DEVERSER);
		wait_ms(2000);

		mecaCom(TIROIR_FERMER);

		trajectory_goto_d(&traj, END, 60);
		if(k)
		{
			trajectory_goto_a(&traj, END, 0);
			trajectory_goto_d(&traj, END, 12);
		}
		else
		{	
			trajectory_goto_a(&traj, END, 90);

			k = 1;
				//trajectory_goto_d(&traj, END, 0);
		}
		while(!trajectory_is_ended(&traj));


	}
	return DONE;
}
uint8_t putPaint(uint8_t side)
{
	printf("putFruit \n");
	double eps = 0;
	set_startCoor(position_get_coor_eps(&pos,&eps));

	set_goalCoor(G_LENGTH * 1 + 7);

	if (eps > EPSILON)
	{
		go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
	}    
	while(!astarMv());

	printf("angle %lf \n",position_get_angle_deg(&pos));

	trajectory_goto_a(&traj, END, 0);
	while(!trajectory_is_ended(&traj));

	printf("angle %lf \n",position_get_angle_deg(&pos));

	trajectory_goto_d(&traj, END, -100);
	while(!trajectory_is_ended(&traj));			
	return DONE;
}

uint8_t calibrateGP2(void)
{
	////////////////////////////////////////
	//Code de Recuperation des valeurs des gp2 avant
	////////////////////////////////////////
	////////////////////////////////////////
	//Placer le robot face a un mur
	////////////////////////////////////////
	trajectory_goto_d(&traj, END, -15);
	while(!trajectory_is_ended(&traj));
	for (int i = 0; i < 15; ++i)
	{
		wait_ms(500);
		printf("gp2 right %d gp2 left %d \n",adc_get_value(ADC_REF_AVCC | MUX_ADC0),adc_get_value(ADC_REF_AVCC | MUX_ADC1));

		trajectory_goto_d(&traj, END, -10);
		while(!trajectory_is_ended(&traj));
	}
	return DONE;
}

uint8_t getTeam(void)
{
	if(SWITCH_TEAM)//cable noir
	{
		return RED;
	}
	else//cable violet
	{ 
		return YELLOW;
	}
}

	//a faire	
uint8_t throwSpears(uint8_t side)
{
	return DONE;
}

uint8_t returnFire(uint8_t fireNb)
{
	static uint8_t tab[7] = {0};
	for(uint8_t i = 1;i < 7;i++)
	{
		if(!(tab[i] & DONE))
		{
			avoidFire(i);
		}
	}

	if(fireNb == R1)
	{
		printf("return R1 \n");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 3 + 3);
		}
		else
		{
			set_goalCoor(G_LENGTH * 3 + 6);
		}

				//put a wall to avoid the wrong way		

		avoidFire(fireNb);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv());

			//test if it succeed
		eps = 0;

		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 3 + 5);

		}
		else
		{
			set_goalCoor(G_LENGTH * 3 + 4);	
		}
			//remove the wall
		stopAvoid(fireNb);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}
		while(!astarMv());




	}
	else if(fireNb == R2)
	{
		printf("return R2 \n");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 4 + 2);
		}
		else
		{
			set_goalCoor(G_LENGTH * 7 + 2);
		}

				//put a wall to avoid the wrong way		

		avoidFire(fireNb);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv());

			//test if it succeed
		eps = 0;

		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 6 + 2);

		}
		else
		{
			set_goalCoor(G_LENGTH * 5 + 2);	
		}
			//remove the wall
		stopAvoid(fireNb);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}
		while(!astarMv());



	}
	else if(fireNb == R3)
	{
		printf("return R3 \n");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 8 + 6);
		}
		else
		{
			set_goalCoor(G_LENGTH * 8 + 3);
		}

				//put a wall to avoid the wrong way		

		avoidFire(fireNb);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv());

			//test if it succeed
		eps = 0;

		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 8 + 4);

		}
		else
		{
			set_goalCoor(G_LENGTH * 8 + 5);	
		}
			//remove the wall
		stopAvoid(fireNb);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}
		while(!astarMv());



	}
	else if(fireNb == Y1)
	{
		printf("return Y1 \n");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 3 + 9);
		}
		else
		{
			set_goalCoor(G_LENGTH * 3 + 12);
		}

				//put a wall to avoid the wrong way		

		avoidFire(fireNb);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv());

			//test if it succeed
		eps = 0;

		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 3 + 11);

		}
		else
		{
			set_goalCoor(G_LENGTH * 3 + 10);	
		}
			//remove the wall
		stopAvoid(fireNb);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}
		while(!astarMv());




	}
	else if(fireNb == Y2)
	{
		printf("return Y2 \n");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 7 + 13);
		}
		else
		{
			set_goalCoor(G_LENGTH * 4 + 13);
		}

				//put a wall to avoid the wrong way		

		avoidFire(fireNb);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv());

			//test if it succeed
		eps = 0;

		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 5 + 13);

		}
		else
		{
			set_goalCoor(G_LENGTH * 6 + 13);	
		}
			//remove the wall
		stopAvoid(fireNb);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}
		while(!astarMv());




	}
	else if(fireNb == Y3)
	{
		printf("return Y3 \n");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 8 + 12);
		}
		else
		{
			set_goalCoor(G_LENGTH * 8 + 9);
		}

				//put a wall to avoid the wrong way		

		avoidFire(fireNb);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv());

			//test if it succeed
		eps = 0;

		set_startCoor(position_get_coor_eps(&pos,&eps));
		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 8 + 10);

		}
		else
		{
			set_goalCoor(G_LENGTH * 8 + 11);	
		}
			//remove the wall
		stopAvoid(fireNb);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}
		while(!astarMv());
	}
	else
	{

	}

		//if succeed
		//tab[fireNb] += DONE; 

	for(uint8_t i = 1;i < 7;i++)
	{
		if(!(tab[i] & DONE))
		{
			stopAvoid(i);
		}
	}

	return DONE;
}


void avoidFire(uint8_t fireNb)
{

	if(fireNb == R1)
	{
		putObstacle(G_LENGTH * 3 + 5);

		putObstacle(G_LENGTH * 3 + 4);
	}
	if(fireNb == R2)
	{
		putObstacle(G_LENGTH * 5 + 2);
		putObstacle(G_LENGTH * 6 + 2);
	}
	if(fireNb == R3)
	{	
		if(team == RED)
		{
			putObstacle(G_LENGTH * 7 + 5);		
		}
		putObstacle(G_LENGTH * 8 + 5);
		putObstacle(G_LENGTH * 8 + 4);

	}
	if(fireNb == Y1)
	{
		putObstacle(G_LENGTH * 3 + 10);
		putObstacle(G_LENGTH * 3 + 11);
	}
	if(fireNb == Y2)
	{
		putObstacle(G_LENGTH * 5 + 13);
		putObstacle(G_LENGTH * 6 + 13);
	}
	if(fireNb == Y3)
	{
		putObstacle(G_LENGTH * 8 + 10);
		putObstacle(G_LENGTH * 8 + 11);
	}

}
void stopAvoid(uint8_t fireNb)
{
	if(fireNb == R1)
	{
		deleteObstacle(G_LENGTH * 3 + 5);
		deleteObstacle(G_LENGTH * 3 + 4);
	}
	if(fireNb == R2)
	{
		deleteObstacle(G_LENGTH * 5 + 2);
		deleteObstacle(G_LENGTH * 6 + 2);
	}
	if(fireNb == R3)
	{	
		if(team == RED)
		{
			deleteObstacle(G_LENGTH * 7 + 5);		
		}
		deleteObstacle(G_LENGTH * 8 + 5);
		deleteObstacle(G_LENGTH * 8 + 4);

	}
	if(fireNb == Y1)
	{
		deleteObstacle(G_LENGTH * 3 + 10);
		deleteObstacle(G_LENGTH * 3 + 11);
	}
	if(fireNb == Y2)
	{
		deleteObstacle(G_LENGTH * 5 + 13);
		deleteObstacle(G_LENGTH * 6 + 13);
	}
	if(fireNb == Y3)
	{
		deleteObstacle(G_LENGTH * 8 + 10);
		deleteObstacle(G_LENGTH * 8 + 11);
	}
}