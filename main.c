

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

uint16_t stack_begin = 0;


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

void homologation(void);
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
	//int a = 0;
	/////////////////////////////////////
	///TODO//////////////////////////////
	/////////////////////////////////////
	//
	//participer a la coupe
	/////////////////////////////////////




	/////////////////////////////////////
	//Initialisations////////////////////
	/////////////////////////////////////
	uart_init();
	time_init(128);
	printf("fdevopen \n");
	fdevopen(uart0_send,uart0_recv, 1);

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
	
	//printf("stack begin :%d \n", &a);
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
	// while(1)
	// {
	// 	printf("gp2r = %d ,gp2l = %d ,gp2m = %d \n",adc_get_value(ADC_REF_AVCC | MUX_ADC0),adc_get_value(ADC_REF_AVCC | MUX_ADC1),adc_get_value(ADC_REF_AVCC | MUX_ADC2));
	// }

	//avoidance_init();
	//antipatinage_init();

	// homologation();
	// while(1);

	// test_evitement();
	// while(1);	

	if(team == RED)
	{




		disableSpinning();	

		findPosition(team);
		enableSpinning();


		trajectory_goto_d(&traj, END, 20);
		trajectory_goto_a(&traj, END, 55);
		while(!trajectory_is_ended(&traj));
	//
	// trajectory_goto_a(&traj, END, 90);
	// 
	// trajectory_goto_a(&traj, END, 0);	
	// while(!trajectory_is_ended(&traj));


		asserv_set_vitesse_normal(&asserv);
		initTaskManager(&tkm);



		//addTask(&tkm, &returnFire, LOW_PRIORITY, R1);
		addTask(&tkm, &throwSpears, LOW_PRIORITY, RED);
		// addTask(&tkm, &returnFire, LOW_PRIORITY, R3);
		// addTask(&tkm, &returnFire, HIGH_PRIORITY, R2);
		addTask(&tkm, &takeFruitRed, HIGH_PRIORITY, team);
		addTask(&tkm, &putPaint, HIGH_PRIORITY, team);
		addTask(&tkm, &findPosition, HIGH_PRIORITY, REDPAINT);
		addTask(&tkm, &returnFire, HIGH_PRIORITY, Y1);
		//addTask(&tkm, &throwSpears, HIGH_PRIORITY, YELLOW);
		addTask(&tkm, &putFruit, HIGH_PRIORITY, team);
		addTask(&tkm, &returnFire, HIGH_PRIORITY, Y3);
		addTask(&tkm, &takeFruitYellow, HIGH_PRIORITY, team);
		addTask(&tkm, &putFruit, HIGH_PRIORITY, team);
		addTask(&tkm, &returnFire, HIGH_PRIORITY, Y2);


		while (TIRETTE);
		printf("begin match \n");

		avoidance_init();

		while(doNextTask(&tkm));
		printf("finish \n");


	}
	else // Team jaune
	{

		disableSpinning();	

		findPosition(team);
		enableSpinning();


		trajectory_goto_d(&traj, END, 20);
		trajectory_goto_a(&traj, END, -55);
		while(!trajectory_is_ended(&traj));

		// addTask(&tkm, &returnFire, LOW_PRIORITY, R1);
		addTask(&tkm, &returnFire, LOW_PRIORITY, Y2);
		addTask(&tkm, &throwSpears, LOW_PRIORITY, YELLOW);
		// addTask(&tkm, &returnFire, HIGH_PRIORITY, Y3);
		addTask(&tkm, &takeFruitYellow, HIGH_PRIORITY, team);
		addTask(&tkm, &putPaint, HIGH_PRIORITY, team);

		addTask(&tkm, &findPosition, HIGH_PRIORITY, YELLOWPAINT);
		addTask(&tkm, &returnFire, HIGH_PRIORITY, R1);
		//addTask(&tkm, &throwSpears, HIGH_PRIORITY, RED);
		addTask(&tkm, &putFruit, HIGH_PRIORITY, team);

		
		addTask(&tkm, &returnFire, HIGH_PRIORITY, R3);
		
		addTask(&tkm, &takeFruitRed, HIGH_PRIORITY, team);

		addTask(&tkm, &putFruit, HIGH_PRIORITY, team);
		addTask(&tkm, &returnFire, HIGH_PRIORITY, R2);
		

		while (TIRETTE);
		printf("begin match \n");


		avoidance_init();


		while(doNextTask(&tkm));
		printf("finish \n");
		// // attend que la tirette soit retirée

		// printf("begin match \n");
		// printf("start pos: x %lf pos y %lf \n",position_get_x_cm(&pos),position_get_y_cm(&pos));

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


	for(int i = 0;i < 32;i++)
	{
		trajectory_goto_arel(&traj, END, 45);
		while(!trajectory_is_ended(&traj));
	}

		// addTask(&tkm, &returnFire, LOW_PRIORITY, R1);
// 		addTask(&tkm, &throwSpears, LOW_PRIORITY, YELLOW);
// 		addTask(&tkm, &returnFire, LOW_PRIORITY, Y2);
// //	addTask(&tkm, &returnFire, HIGH_PRIORITY, Y3);
// 		addTask(&tkm, &takeFruitYellow, HIGH_PRIORITY, team);
//	addTask(&tkm, &putPaint, HIGH_PRIORITY, team);

//	addTask(&tkm, &findPosition, HIGH_PRIORITY, YELLOWPAINT);
		// addTask(&tkm, &returnFire, HIGH_PRIORITY, R1);
		// addTask(&tkm, &throwSpears, HIGH_PRIORITY, RED);
	// addTask(&tkm, &putFruit, HIGH_PRIORITY, team);


		// addTask(&tkm, &returnFire, HIGH_PRIORITY, R3);

		// addTask(&tkm, &takeFruitRed, HIGH_PRIORITY, team);

		// addTask(&tkm, &putFruit, HIGH_PRIORITY, team);
		// addTask(&tkm, &returnFire, HIGH_PRIORITY, R2);
	// asserv_set_vitesse_normal(&asserv);

	// while (TIRETTE);
	// printf("begin match \n");


	// avoidance_init();


	// while(doNextTask(&tkm));
	// 	printf("finish \n");	// throwSpears(RED);

	// 	printf("succeed");

		// trajectory_goto_d(&traj, END, 10);
		// while(!trajectory_is_ended(&traj));
		// printf("astar test \n");	    
		// set_startCoor(G_LENGTH * 2 + 2);
		// set_goalCoor(G_LENGTH * 8 + 13);
		// while(!astarMv() && !actionIsFailed());

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
		asserv_set_vitesse_fast(&asserv);

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

		asserv_set_vitesse_fast(&asserv);
	}
	else if(type == REDPAINT)
	{

		// asserv_set_vitesse_low(&asserv);
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos, &eps));
		printf("start pos: %d eps %lf \n",position_get_coor_eps(&pos, &eps),eps);

		set_goalCoor(G_LENGTH * 1 + 7);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return DONE;

		asserv_set_vitesse_normal(&asserv);

		trajectory_goto_a(&traj, END, 90);
		trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		if(actionIsFailed())return DONE;

		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
		while(!trajectory_is_ended(&traj));
		if(actionIsFailed())return DONE;
		cli();

		printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);

		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(140.0), fxx_from_double(90.0));
		asserv_stop(&asserv);


		asserv_set_vitesse_normal(&asserv);
	}
	else if(type == YELLOWPAINT)
	{


		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos, &eps));
		printf("start pos: %d eps %lf \n",position_get_coor_eps(&pos, &eps),eps);

		set_goalCoor(G_LENGTH * 1 + 8);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    


		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return DONE;

		asserv_set_vitesse_normal(&asserv);
		disableAvoidance();
		trajectory_goto_a(&traj, END, -90);
		trajectory_goto_d(&traj, END, -200);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		enableAvoidance();
		if(actionIsFailed())return DONE;
		trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
		while(!trajectory_is_ended(&traj));
		if(actionIsFailed())return DONE;
		cli();

		printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);

		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(160.0), fxx_from_double(-90.0));
		asserv_stop(&asserv);
		asserv_set_vitesse_normal(&asserv);
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


	while(!astarMv() && !actionIsFailed());
	if(actionIsFailed())return FAILED;

	printf("take the fruit");
	trajectory_goto_a(&traj, END, -90);
	trajectory_goto_d(&traj, END,10);
	trajectory_goto_a(&traj, END, 0);
	while(!trajectory_is_ended(&traj));
	if(actionIsFailed())return FAILED;

	asserv_set_vitesse_normal(&asserv);
	mecaCom(TIROIR_OUVERT);
	mecaCom(PEIGNE_OUVERT);


	trajectory_goto_d(&traj, END,60);
	trajectory_goto_a(&traj, END,45);
	trajectory_goto_d(&traj, END,9 * 1.414);
	// trajectory_goto_a(&traj, END, 0);
	// trajectory_goto_d(&traj, END,10);
	trajectory_goto_a(&traj, END, 90);
	trajectory_goto_d(&traj, END,60);
	trajectory_goto_a(&traj, END, 180);
	trajectory_goto_d(&traj, END,9);
	while(!trajectory_is_ended(&traj));
	mecaCom(PEIGNE_FERMER);

	if(team == RED)
	{
		asserv_set_vitesse_fast(&asserv);
	}
	if(actionIsFailed())return FAILED;

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


	while(!astarMv() && !actionIsFailed());
	if(actionIsFailed())return FAILED;

	printf("take the fruit");

	mecaCom(PEIGNE_OUVERT);
	mecaCom(TIROIR_OUVERT);

	trajectory_goto_a(&traj, END, 0);
	trajectory_goto_d(&traj, END, 8);
	trajectory_goto_a(&traj, END, 90);
	while(!trajectory_is_ended(&traj));
	if(actionIsFailed())return FAILED;

	asserv_set_vitesse_normal(&asserv);


	trajectory_goto_d(&traj, END, 80);
	trajectory_goto_a(&traj, END, 135);
	if(team == RED)
	{
		trajectory_goto_d(&traj, END, 17 * 1.414);
	}
	else
	{
		trajectory_goto_d(&traj, END, 12 * 1.414);	
	}
	// trajectory_goto_a(&traj, END, 90);
	// while(!trajectory_is_ended(&traj));

	// trajectory_goto_d(&traj, END, 12);
	// while(!trajectory_is_ended(&traj));


	trajectory_goto_a(&traj, END, 180);
	trajectory_goto_d(&traj, END, 55);
	trajectory_goto_a(&traj, END, -90);
	if(team == RED)
	{
		trajectory_goto_d(&traj, END, 17 * 1.414);
	}
	else
	{
		trajectory_goto_d(&traj, END, 12 * 1.414);	
	}
	while(!trajectory_is_ended(&traj));

	mecaCom(PEIGNE_FERMER);
	if(team == YELLOW)
	{
		asserv_set_vitesse_fast(&asserv);
	}
	if(actionIsFailed())return FAILED;

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
		set_goalCoor(G_LENGTH * 3 + 11);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		} 
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;

		trajectory_goto_a(&traj, END, 0);      	     
		while(!trajectory_is_ended(&traj));
		if(actionIsFailed())return FAILED;

		disableSpinning();
		trajectory_goto_d(&traj, END, -40);
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		enableSpinning();
		//se recaller?

		throwSpears(YELLOW);
		trajectory_goto_d(&traj, END, 8);
		if(k)
		{
			trajectory_goto_a(&traj, END, -90);	
		}
		else
		{
			trajectory_goto_a(&traj, END, -90);
		}

		//trajectory_goto_d(&traj, END, -40);

		while(!trajectory_is_ended(&traj));
		//if(actionIsFailed())return FAILED;

		mecaCom(TIROIR_DEVERSER);
		wait_ms(2000);

		mecaCom(TIROIR_FERMER);
		//trajectory_goto_d(&traj, END, -60);
		//trajectory_goto_a(&traj, END, 0);
		//trajectory_goto_d(&traj, END,15);
		while(!trajectory_is_ended(&traj));
	}
	else
	{

		set_goalCoor(G_LENGTH * 3 + 4);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}  
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;

		trajectory_goto_a(&traj, END, 0); 

		while(!trajectory_is_ended(&traj));
		if(actionIsFailed())return FAILED;    	     
		trajectory_goto_d(&traj, END, -40);

		disableSpinning();
		while(!trajectory_is_ended(&traj))
		{
			if(REPOSITIONING)
			{
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
			}
		}
		enableSpinning();
		throwSpears(RED);
		trajectory_goto_d(&traj, END, 8);

		if(k)
		{
			trajectory_goto_a(&traj, END, -90);
		}
		else
		{
			trajectory_goto_a(&traj, END, -90);
		}

		while(!trajectory_is_ended(&traj));

		if(actionIsFailed())return FAILED;

		mecaCom(TIROIR_DEVERSER);
		wait_ms(2000);

		mecaCom(TIROIR_FERMER);

		// if(k)
		// {
		// 	trajectory_goto_a(&traj, END, 0);
		// 	trajectory_goto_d(&traj, END, 12);
		// }
		// else
		// {	
		// 	trajectory_goto_a(&traj, END, 90);
		// 	k = 1;
		// 		//trajectory_goto_d(&traj, END, 0);
		// }
		// while(!trajectory_is_ended(&traj));
	}
	return DONE;
}
uint8_t putPaint(uint8_t side)
{
	printf("putFruit \n");
	double eps = 0;
	set_startCoor(position_get_coor_eps(&pos,&eps));

	if(side == RED)
	{
		set_goalCoor(G_LENGTH * 2 + 7);
	}
	else
	{
		set_goalCoor(G_LENGTH * 2 + 7);
	}
	if (eps > EPSILON)
	{
		go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
	}    
	while(!astarMv() && !actionIsFailed());

	if(actionIsFailed())return FAILED;

	printf("angle %lf \n",position_get_angle_deg(&pos));
	trajectory_goto_a(&traj, END, 180);
	trajectory_goto_d(&traj, END, 10);
	
	while(!trajectory_is_ended(&traj));
	trajectory_goto_a(&traj, END, 0);
	
	asserv_set_vitesse_normal(&asserv);
	if(actionIsFailed())return FAILED;
	disableAvoidance();
	printf("angle %lf \n",position_get_angle_deg(&pos));
	trajectory_goto_d(&traj, END, -100);
	while(!trajectory_is_ended(&traj));			

	enableAvoidance();

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
	//trajectory_goto_d(&traj, END, -10);
	//while(!trajectory_is_ended(&traj));
	for (int i = 0; i < 25; ++i)
	{
		wait_ms(500);
		printf("dist = %d cm",i*5);
		printf("gp2 right %d gp2 left %d gp2 middle %d \n",adc_get_value(ADC_REF_AVCC | MUX_ADC0),adc_get_value(ADC_REF_AVCC | MUX_ADC1),adc_get_value(ADC_REF_AVCC | MUX_ADC2));

		trajectory_goto_d(&traj, END, -5);
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

uint8_t throwSpears(uint8_t side)
{

	if(side == RED)
	{
		if(team == RED)
		{
			double eps = 0;
			set_startCoor(position_get_coor_eps(&pos,&eps));

			set_goalCoor(G_LENGTH * 3 + 3);

			if (eps > EPSILON)
			{
				go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
			}    
			while(!astarMv());

			trajectory_goto_a(&traj, END, 90);      	     
			trajectory_goto_d(&traj, END, 11);

			trajectory_goto_a(&traj, END, -180);      	     
			trajectory_goto_d(&traj, END, 14.8);
			while(!trajectory_is_ended(&traj));
			mecaCom(LANCE_BALLE_AV);
			trajectory_goto_d(&traj, END, -14.8);
			while(!trajectory_is_ended(&traj));
		}
		else
		{
			// double eps = 0;
			// set_startCoor(position_get_coor_eps(&pos,&eps));

			// set_goalCoor(G_LENGTH * 3 + 4);

			// if (eps > EPSILON)
			// {
			// 	go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
			// }    
			// while(!astarMv() && !actionIsFailed());

			// trajectory_goto_a(&traj,END,0);
			// trajectory_goto_d(&traj, END, -16.8);
			// while(!trajectory_is_ended(&traj));
			mecaCom(LANCE_BALLE_AR);
			// trajectory_goto_d(&traj, END, 16.8);
			// while(!trajectory_is_ended(&traj));
		}
	}
	else
	{
		if(team == RED)
		{
			// double eps = 0;
			// set_startCoor(position_get_coor_eps(&pos,&eps));

			// set_goalCoor(G_LENGTH * 3 + 11);

			// if (eps > EPSILON)
			// {
			// 	go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
			// }    
			// while(!astarMv());

			// trajectory_goto_a(&traj, END, 0);      	     
			// trajectory_goto_d(&traj, END, -16.8);
			// while(!trajectory_is_ended(&traj));
			mecaCom(LANCE_BALLE_AR);
			// trajectory_goto_d(&traj, END, 16.8);
			// while(!trajectory_is_ended(&traj));
			


		}
		else
		{
			double eps = 0;
			set_startCoor(position_get_coor_eps(&pos,&eps));

			set_goalCoor(G_LENGTH * 3 + 12);

			if (eps > EPSILON)
			{
				go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
			}    
			while(!astarMv() && !actionIsFailed());

			trajectory_goto_a(&traj,END,-90);
			trajectory_goto_d(&traj, END, 11);

			trajectory_goto_a(&traj,END,180);
			trajectory_goto_d(&traj, END, 14.8);
			while(!trajectory_is_ended(&traj));
			mecaCom(LANCE_BALLE_AV);
			trajectory_goto_d(&traj, END, -14.8);
			while(!trajectory_is_ended(&traj));
		}	
	}
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

	uint16_t stackend = 0;
	printf("stack %d \n",&stackend - &stack_begin);

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
			asserv_set_vitesse_normal(&asserv);
		}

				//put a wall to avoid the wrong way		

		avoidFire(fireNb);


		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv() && !actionIsFailed());

		if(actionIsFailed())return FAILED;

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
		while(!astarMv() && !actionIsFailed());

		if(actionIsFailed())return FAILED;



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
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;

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
		while(!astarMv() && !actionIsFailed());

		if(actionIsFailed())return FAILED;


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
		while(!astarMv() && !actionIsFailed());

		if(actionIsFailed())return FAILED;

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
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;


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
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;
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
		while(!astarMv() && !actionIsFailed());

		if(actionIsFailed())return FAILED;


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
		while(!astarMv() && !actionIsFailed());

		if(actionIsFailed())return FAILED;
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
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;



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
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;

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
		while(!astarMv() && !actionIsFailed());
		if(actionIsFailed())return FAILED;
	}
	else
	{

	}

		//if succeed
	tab[fireNb] += DONE; 

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


void homologation(void)
{
	disableSpinning();
	findPosition(team);
	trajectory_goto_d(&traj, END, 18);
	if(team == RED)
	{
		trajectory_goto_a(&traj, END, 45);      	     
	}
	else
	{
		trajectory_goto_a(&traj, END, -45);	
	}
	while(!trajectory_is_ended(&traj));

	enableSpinning();


	asserv_set_vitesse_normal(&asserv);

	while(TIRETTE);
	avoidance_init();
	if(team == RED)
	{
		returnFire(Y1);
	}
	else
	{
		returnFire(R1);
	}		
}