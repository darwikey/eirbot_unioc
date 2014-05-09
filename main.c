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

#define RED 1
#define YELLOW 0

#define ENABLE_MECA
#define MAXIME 0

/////////////////////////////////////
// Prototypes ///////////////////////
/////////////////////////////////////
void initCom(void);
uint8_t mecaCom(uint8_t ordre);
void test_evitement(void);
void findPosition(uint8_t type);
void putFruit(void);
void takeFruitYellow(void);
void takeFruitRed(void);
void putPaint(void);
void calibrateGP2(void);

/////////////////////////////////////
// Variables globales
/////////////////////////////////////
trajectory_manager_t traj;
position_manager_t pos;
asserv_manager_t asserv;


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
	
	// while(1);
	if(SWITCH_TEAM)//cable noir
	{
		team = RED;
		printf("team rouge\n");
	}
	else//cable violet
	{ 
		team = YELLOW;
		printf("team jaune\n");
	}
	
	while(MAXIME);//boucle anti_maxime

	// while(1)
	// {

	// 	position_set_xya_cm_deg(&pos,fxx_from_double(57),fxx_from_double(57),fxx_from_double(0.0));

	// 	set_goalCoor(84);
	// 	printf("pos: x %lf, y %lf \n",position_get_x_cm(&pos),position_get_y_cm(&pos));


	// 	printf("goal: x%d ,y %d \n",get_goalCoor()%G_LENGTH,get_goalCoor()/G_LENGTH);
	// 	go_to_node(position_get_y_cm(&pos),position_get_x_cm(&pos));
	// 	while(trajectory_is_ended(&traj));
	// 	while(1);
	// }
//	avoidance_init();
//	antipatinage_init();

	// test_evitement();
	// while(1);

	/*wait_ms(1000);
	asserv_set_vitesse_low(&asserv);
        trajectory_goto_d(&traj, END, 20);
	trajectory_goto_arel(&traj, END, 90);

		while(1)
	  {
	    //printf("d: %ld     a: %ld    a_deg: %lf   a_rad %lf    x : %ld   y : %ld \n", (int32_t)(pos.distance / 111), (int32_t)(pos.angle), (double)pos.angle * (180.0 / ROBOT_IMP_PI), (double)pos.angle * 0.000329686, (int32_t)(pos.x / 111), (int32_t)(pos.y / 111));

	    int32_t encodeur_gauche = U_ENC1;
	      int32_t encodeur_droit = U_ENC3;
	      int32_t moteur_gauche = U_ENC0;
	      int32_t moteur_droit = U_ENC2;
	      printf("enc G : %ld     enc D : %ld     motG : %ld     mot D : %ld\n", encodeur_gauche, encodeur_droit, moteur_gauche, moteur_droit);
	      wait_ms(100);
	      }*/

	

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



		takeFruitRed();
		putFruit();

		disableSpinning();
		findPosition(YELLOW);
		enableSpinning();
		printf("coor : x: %d y %d \n",position_get_coor(&pos)%G_LENGTH,position_get_coor(&pos)/G_LENGTH);	
		takeFruitYellow();
		putFruit();
		putPaint();
		
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

		
		takeFruitYellow();          
		putFruit();

		disableSpinning();
		findPosition(RED);
		enableSpinning();
		takeFruitRed();
		putFruit();
		putPaint();

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

		findPosition(YELLOW);

		enableSpinning();	
		takeFruitRed();
		//putPaint();





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

	void findPosition(uint8_t type)
	{
		asserv_set_vitesse_low(&asserv);
		if(type == RED)
		{
			trajectory_goto_d(&traj, END, -200);
			while(!trajectory_is_ended(&traj))
			{
				if(REPOSITIONING)
				{//stop the trajectory
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
			asserv_stop(&asserv);//attention bug bizare avant			
		}
		else // team == YELLOW
		{			
			trajectory_goto_d(&traj, END, -200);
			while(!trajectory_is_ended(&traj))
			{
				if(REPOSITIONING)
				  {//stop the trajectory
				
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


		}
		//printf("pos asserv a : %lu %lu \n",U_ASSERV_ANGLE,U_PM_ANGLE);


		quadramp_reset(&asserv);
		control_reset(&asserv);
		pid_reset(&asserv);
		diff_reset(&asserv);
		sei();
		asserv_set_vitesse_normal(&asserv);

	}

	void takeFruitRed()
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

		trajectory_goto_d(&traj, END,13);
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
	}

	void takeFruitYellow()
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

		trajectory_goto_d(&traj, END, 13 * 1.414);
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

		trajectory_goto_d(&traj, END, 13);
		while(!trajectory_is_ended(&traj));
		
		mecaCom(PEIGNE_FERMER);
	}

	void putFruit(void)
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
				trajectory_goto_d(&traj, END, 12);
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
	}
	void putPaint(void)
	{
		printf("putFruit \n");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		
		set_goalCoor(G_LENGTH * 3 + 7);

		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		while(!astarMv())

		printf("angle %lf \n",position_get_angle_deg(&pos));

		trajectory_goto_a(&traj, END, 0);
		while(!trajectory_is_ended(&traj));

		printf("angle %lf \n",position_get_angle_deg(&pos));
		
		trajectory_goto_d(&traj, END, -100);
		while(!trajectory_is_ended(&traj));			

	}

	void calibrateGP2(void)
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

	}
