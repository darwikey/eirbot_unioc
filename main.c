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
#define DIST_CENTRE 12.2

// Interrupteur
#define REPOSITIONING (PINE & 0x10)
#define TIRETTE (PINE & 0x8) 
#define SWITCH_TEAM (PINE & 0x40)

// Marge d'erreur sur le positionnement lorsque le robot est  au centre d'un case
#define EPSILON 4.0

//verifier que startCoor a changer
//faire une deuxieme fonction d'evitement pour lorsque le robot est hors A* 

#define RED 1
#define YELLOW 0

#define MAXIME 0

/////////////////////////////////////
// Prototypes ///////////////////////
/////////////////////////////////////
void initCom(void);
uint8_t mecaCom(uint8_t ordre);
void test_evitement(void);
void findPosition(uint8_t type);
void putFruit(int8_t team);
void takeFruitYellow(void);
void takeFruitRed(void);

/////////////////////////////////////
// Variables globales
/////////////////////////////////////
trajectory_manager_t traj;
position_manager_t pos;
asserv_manager_t asserv;


int8_t mode_fonctionnement;
extern int8_t patinage;

int8_t I2C_DISPO = 1;



//////////////////////////////////////
// Main
/////////////////////////////////////
int main(void)
{

	/////////////////////////////////////
	//Initialisations////////////////////
	/////////////////////////////////////
	uart_init();
	time_init(128);
	printf("fdevopen \n");
	fdevopen(uart0_send,NULL, 0);
	/***** tip -s 9600 -l /dev/ttyS0 *****/
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

	//avoidance_init();
	antipatinage_init();

	position_init(&pos);
	asserv_init(&asserv,&pos);
	trajectory_init(&traj,&pos,&asserv);
	i2cm_init();
	//autorise les interruptions
	sei(); 

	//init communication
	printf("init com \n");
	//initCom();
	printf("end init com \n");

	//init A*
	initObstacle();

	//init méca
	//mecaCom(TIROIR_FERMER);
	//mecaCom(PEIGNE_FERMER);
	
	uint8_t team;
	if(SWITCH_TEAM)//cable noir
	{
		team = RED;
	}
	else//cable violet
	{
		team = YELLOW;
	}
	
	
	printf("team %d \n ",team);
	
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
	//test_evitement();

	if(team == RED)
	{

		//findPosition(team);
		
			// attend que la tirette soit retirée
		//while (!TIRETTE);
		
		//position_set_x_cm(&pos,20.0);
		//position_set_y_cm(&pos,20.0);
		//enableSpinning();
		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(20.0), fxx_from_double(0.0));

		printf("begin match \n");	    
		
		printf("start pos: x %lf pos y %lf \n",position_get_x_cm(&pos),position_get_y_cm(&pos));

		takeFruitRed();
		putFruit(RED);
		takeFruitYellow();
		putFruit(RED);
		
			while(1);//attention boucle infinie
		}
	else // Team jaune
	{
		//findPosition(team);

			// attend que la tirette soit retirée
		//while (!TIRETTE);
		position_set_xya_cm_deg(&pos,fxx_from_double(20.0),fxx_from_double(280.0), fxx_from_double(0.0));
		//enableSpinning();
		printf("begin match \n");
		printf("start pos: x %lf pos y %lf \n",position_get_x_cm(&pos),position_get_y_cm(&pos));


		takeFruitYellow();          
		putFruit(YELLOW);
		takeFruitRed();
		putFruit(YELLOW);

			while(1);//attention boucle infinie
		}
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
		I2C_DISPO = 0;
		uint8_t result = -1;

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
				printf("Erreur de communication avec la carte méca");
			}
		}

		I2C_DISPO = 1;
		return result;
	}

	void test_evitement(void)
	{
		
		for(int i = 1;i<16;i++)
		{
		trajectory_goto_arel(&traj, END, 180 * i);
		while(!trajectory_is_ended(&traj));
		}	
	//printf("astar test \n");	    
	//set_startCoor(G_LENGTH * 2 + 2);
	//set_goalCoor(G_LENGTH * 8 + 13);

	//while(!astarMv());




		double x = fxx_to_double(position_get_y_cm(&pos));
		double y = fxx_to_double(position_get_x_cm(&pos));
		printf("x : %lf    y : %lf\n", x, y);
		printf("isOut  %d    obs %d\n", isOutOfGraphe(x, y), isObstacle(x, y));
		//go_to_node(x, y,graphe);

		while(1);
	}

	void findPosition(uint8_t type)
	{
		if(RED)
		{
			trajectory_goto_d(&traj, END, -100);
			while(!trajectory_is_ended(&traj))
			{
				if(REPOSITIONING)
			{//stop the trajectory
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
				
				position_set_xya_cm_deg(&pos,position_get_y_cm(&pos),fxx_from_double(DIST_CENTRE), fxx_from_double(DIST_CENTRE));
			}
		}
			trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);//dist centre = 12.2
			while(!trajectory_is_ended(&traj));
			trajectory_goto_a(&traj, END, 0);
			while(!trajectory_is_ended(&traj));
			trajectory_goto_d(&traj, END, -100);
			while(!trajectory_is_ended(&traj))
			{
				if(REPOSITIONING)
				{
					trajectory_reinit(&traj);
					asserv_stop(&asserv);
					
					position_set_xya_cm_deg(&pos,fxx_from_double(DIST_CENTRE),position_get_x_cm(&pos), position_get_angle_deg(&pos));
				}
			}
			trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
			while(!trajectory_is_ended(&traj));
		}
		else
		{
			
			trajectory_goto_d(&traj, END, -100);
			while(!trajectory_is_ended(&traj))
			{
				if(REPOSITIONING)
			{//stop the trajectory
				
				trajectory_reinit(&traj);
				asserv_stop(&asserv);
				

				position_set_xya_cm_deg(&pos,position_get_x_cm(&pos),fxx_from_double(300 - DIST_CENTRE), fxx_from_double(-90.0));
			}
		}
			trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);//dist centre = 12.2
			while(!trajectory_is_ended(&traj));
			trajectory_goto_a(&traj, END, 0);
			while(!trajectory_is_ended(&traj));
			trajectory_goto_d(&traj, END, -100);
			while(!trajectory_is_ended(&traj))
			{
				if(REPOSITIONING)
				{
					
					trajectory_reinit(&traj);
					asserv_stop(&asserv);
					
					position_set_xya_cm_deg(&pos,fxx_from_double(DIST_CENTRE),position_get_x_cm(&pos), position_get_angle_deg(&pos));
				}
			}
			trajectory_goto_d(&traj, END, 20 - DIST_CENTRE);
			while(!trajectory_is_ended(&traj));
		}
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
		
		//mecaCom(TIROIR_OUVERT);
		//mecaCom(PEIGNE_OUVERT);
		
		trajectory_goto_a(&traj, END, 0);
		while(!trajectory_is_ended(&traj));
		wait_ms(200);
		trajectory_goto_d(&traj, END,60);
		while(!trajectory_is_ended(&traj));
		
		trajectory_goto_a(&traj, END, 90);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END,13);
		while(!trajectory_is_ended(&traj));
		
		
		trajectory_goto_a(&traj, END, 0);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END,5);
		while(!trajectory_is_ended(&traj));
		

		trajectory_goto_a(&traj, END, 90);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END,60);
		while(!trajectory_is_ended(&traj));
		//mecaCom(PEIGNE_FERMER);
		
		trajectory_goto_a(&traj, END, 180);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END,5);
		while(!trajectory_is_ended(&traj));
	}

	void takeFruitYellow()
	{

		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
printf("start pos: %d eps %lf \n",position_get_coor_eps(&pos, &eps),eps);
		set_goalCoor(G_LENGTH * 8 + 10);
		
		if (eps > EPSILON)
		{
			go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
		}    
		

		
		while(!astarMv());
		
		
		printf("take the fruit");
		
		//mecaCom(PEIGNE_OUVERT);
		//mecaCom(TIROIR_OUVERT);
		
		trajectory_goto_a(&traj, END, 0);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END, 12);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_a(&traj, END, 90);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END, 60);
		while(!trajectory_is_ended(&traj));      


		trajectory_goto_a(&traj, END, 180);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END, 12);
		while(!trajectory_is_ended(&traj));
		
		trajectory_goto_a(&traj, END, 90);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END, 12);
		while(!trajectory_is_ended(&traj));
		

		trajectory_goto_a(&traj, END, 180);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END, 60);
		while(!trajectory_is_ended(&traj));
		
		trajectory_goto_a(&traj, END, -90);
		while(!trajectory_is_ended(&traj));

		trajectory_goto_d(&traj, END, 12);
		while(!trajectory_is_ended(&traj));
		
		//mecaCom(PEIGNE_FERMER);

	}

	void putFruit(int8_t team)
	{
		printf("begin take fruit");
		double eps = 0;
		set_startCoor(position_get_coor_eps(&pos,&eps));
		printf("start pos: %d eps %lf \n", position_get_coor_eps(&pos, &eps),eps);

		if(team == RED)
		{
			set_goalCoor(G_LENGTH * 3 + 8);
			
			if (eps > EPSILON)
			{
				go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));
			} 
			while(!astarMv());
			
			trajectory_goto_a(&traj, END, 180);      	     
			trajectory_goto_d(&traj, END, 7);
			trajectory_goto_a(&traj, END, -90);      
			trajectory_goto_d(&traj, END, -60);
			while(!trajectory_is_ended(&traj));

			//mecaCom(TIROIR_DEVERSER);
			wait_ms(2000);

			//mecaCom(TIROIR_FERMER);
			trajectory_goto_d(&traj, END, 60);
			trajectory_goto_a(&traj, END, 0);
			trajectory_goto_d(&traj, END, 7);
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
			while(!trajectory_is_ended(&traj));
			
			trajectory_goto_d(&traj, END, 7);
			while(!trajectory_is_ended(&traj));
			
			trajectory_goto_a(&traj, END, -90);
			while(!trajectory_is_ended(&traj));
			
			trajectory_goto_d(&traj, END, 60);
			while(!trajectory_is_ended(&traj));
			
			//mecaCom(TIROIR_DEVERSER);
			wait_ms(4000);

			//mecaCom(TIROIR_FERMER);
			
			trajectory_goto_d(&traj, END, 60);
			while(!trajectory_is_ended(&traj));

			trajectory_goto_a(&traj, END, 0);
			while(!trajectory_is_ended(&traj));

			trajectory_goto_d(&traj, END, 7);
			while(!trajectory_is_ended(&traj));

		}
	}
