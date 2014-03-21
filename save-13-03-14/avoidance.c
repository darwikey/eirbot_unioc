
#include <aversive.h>
#include <wait.h>
#include <time.h>
#include <stdio.h>
#include <scheduler.h>
#include <adc.h>
#include <math.h>
#include "avoidance.h"
#include "astar.h"
#include "gp2.h"

extern trajectory_manager_t traj;
extern position_manager_t pos;
extern asserv_manager_t asserv;
extern node graphe[G_SIZE]; 

// stocke l'id de la fonction appelée par le scheduler
s08 id_scheduler = 0;


void avoidance_init(void)
{

  id_scheduler = scheduler_add_periodical_event(adversary_detection_traj,NULL, 500000/SCHEDULER_UNIT);
}


// fonction appelée par le scheduler
void adversary_detection_Astar(void*p)
{ 
  // désactive le scheduler pour éviter qu'il se déclenche pendant le calcul de l'A* 
  scheduler_del_event(id_scheduler);

  // récupère les tensions des GP2
  long gp2_avant_droite = adc_get_value(ADC_REF_AVCC | MUX_ADC0);
     
  // obstacle detecté à droite
  if(gp2_avant_droite > 200)
    {
      printf("obstacle detecte (droite)\n");
      
      double angle = fxx_to_double(position_get_angle_deg(&pos));
      printf("value of angle %lf \n", angle);
      
      if(new_obstacle(gp2_avant_droite, angle, fxx_to_double(position_get_y_cm(&pos)), fxx_to_double(position_get_x_cm(&pos))))
	{
	  printf("stop the trajectory \n");

	  trajectory_reinit(&traj);
	  asserv_stop(&asserv);
	  
	  stopAstarMovement();

	        
	  //trajectory_resume(&traj);
	  printf("succeed to stop trajectory \n");

	  

	  go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));//attention x et y inverser
	}
    }
  
  // réactive le scheduler
  id_scheduler = scheduler_add_periodical_event(adversary_detection_Astar,NULL, 500000/SCHEDULER_UNIT);
}


// fonction appelée par le scheduler
// 
void adversary_detection_traj(void*p)
{
  // désactive le scheduler pour éviter qu'il se déclenche pendant le calcul
  scheduler_del_event(id_scheduler);

  int coor = gp2_get_coor_obstacle(GP2_RIGHT);
 


  printf("   noeud : %d, %d \t", coor % G_LENGTH, coor / G_LENGTH);

  // check si on a détecté quelque chose
  if (coor != -1)
    {
      uint8_t obstacle = graphe[coor].type == OBSTACLE;

      // Si c'est un obstacle non connu on panique
      if(!obstacle)
	{
	  if(gp2_get_dist(GP2_RIGHT)< 40)
	    {
	      printf("stop the trajectory \n");
	    
	      trajectory_pause(&traj);  
	    }
	  else
	    {
	      printf("too far \n");
	    }
	}
      else
	{
	  printf("OK obstacle doesn't matter\n");
	}
    }
  else // rien détecté
    {
      if(trajectory_is_paused(&traj))
	{
	  printf("resume the trajectory\n");
	  trajectory_resume(&traj);
	}
    }
    
  // réactive le scheduler
  id_scheduler = scheduler_add_periodical_event(adversary_detection_traj,NULL, 500000/SCHEDULER_UNIT);
}


// Planifie une trajectoire pour aller au noeud le plus proche 
//not optimized because le robot retourne sur ses pas

void go_to_node(double position_x, double position_y)
{
  int x = (int) position_x + 20;
  int y = (int) position_y + 20;
  printf("go_to_node ;   actuellement : (x: %d , y: %d);   ",x/UNIT, y/UNIT);

  if(!isOutOfGraphe(x, y) && !isObstacle(x, y))
    {
      trajectory_goto_a(&traj, END, -90);
      trajectory_goto_d(&traj, END, x%UNIT);
      trajectory_goto_a(&traj, END, 180);
      trajectory_goto_d(&traj, END, y%UNIT);

      set_startCoor(x/UNIT + G_LENGTH*(y/UNIT));
    }
  else if(!isOutOfGraphe(x+1, y) && !isObstacle(x+1, y))
    {
      trajectory_goto_a(&traj, END, 90);
      trajectory_goto_d(&traj, END,(UNIT - (x%UNIT)));
      trajectory_goto_a(&traj, END, 180);
      trajectory_goto_d(&traj, END, y%UNIT);
     
      set_startCoor(x/UNIT + 1 + G_LENGTH*(y/UNIT));
    }
  else if(!isOutOfGraphe(x, y+1) && !isObstacle(x, y+1))
    {
      trajectory_goto_a(&traj, END, -90);
      trajectory_goto_d(&traj, END, x%UNIT);
      trajectory_goto_a(&traj, END, 0);
      trajectory_goto_d(&traj, END, UNIT - (y%UNIT));
      
     set_startCoor(x/UNIT + G_LENGTH * (1 + y/UNIT));
    }
  else if(!isOutOfGraphe(x+1, y+1) && !isObstacle(x+1, y+1)) 
    {
     trajectory_goto_a(&traj, END, 90);
     trajectory_goto_d(&traj, END,UNIT - (x%UNIT));
     trajectory_goto_a(&traj, END, 0);
     trajectory_goto_d(&traj, END,UNIT - (y%UNIT));
     
     set_startCoor(x/UNIT+ 1 + G_LENGTH * (1 + y/UNIT));
    }
  else
    {
      printf("echec go_to_node !!!\n");
    }

  printf("startCoor : (%d, %d)\n", get_startCoor() % G_LENGTH, get_startCoor() / G_LENGTH);
}


// place dynamiquement les nouveaux obstacles sur le graphe
int new_obstacle(long gp2Value,double angle,double posx,double posy)
{
  // prend en compte le decallage initiale
  posx += 20.0;
  posy += 20.0;

  int y = (int) (cos(angle * M_PI / 180.0) * 40.0);  
  int x = (int) (sin(angle * M_PI / 180.0) * 40.0);

  printf("dist obstacle : x: %d ,y:%d \n",x,y);
  
  printf("posx : %lf , posy :%lf \n",posx,posy);

  // Calcule la coordonné la plus proche d'un noeud
  int shift_x = (x + (int)posx) % UNIT >= UNIT/2;
  int shift_y = (y + (int)posy) % UNIT >= UNIT/2;
  
  //Now the value of x and y is in UNIT
  x = (x + posx) / UNIT + shift_x;
  y = (y + posy) / UNIT + shift_y;

  printf("new obstacle : x: %d ,y:%d \n",x,y);
  
  int index = x  + G_LENGTH * y;

  // check si on sort pas du tableau
  if (index >= 0 && index < G_SIZE && !isOutOfGraphe(x, y))
    {
      if(graphe[index].type == OBSTACLE)
	{
	  printf("already an obstacle \n");
	  // pas réussi à placer d'obstacle
	  return 0;
	}
    }
  else
    {
      printf("not in the table \n");
      // pas reussi a placer d'obstacle
      return 0;
    }

  // On place dans les cases adjacentes au truc détécté des obstacles
  for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
	{
	  index = x + i + G_LENGTH*(j + y); 
	  if (index >= 0 && index < G_SIZE)
	    {
	      graphe[index].type = OBSTACLE;
	    }
	}
    }
  printf("obstacle in node : x:%d, y:%d \n",x ,y);
  return 1;
}

// renvoie vraie si le point est à l'exterieur de la table
int8_t isOutOfGraphe(int x,int y)
{
  return x < 0 
    || (x / UNIT) >= G_LENGTH
    || y < 0
    || (y / UNIT) >= (G_SIZE / G_LENGTH);
}


// renvoie vraie si le point est dans un obstacle
int8_t isObstacle(int x,int y)
{
  return graphe[x/UNIT + G_LENGTH*(y/UNIT)].type == OBSTACLE;
}


void enable_Astar_scheduler(void)
{
  //scheduler_del_event(id_scheduler);

  //id_scheduler = scheduler_add_periodical_event(adversary_detection_Astar,NULL, 500000/SCHEDULER_UNIT);
}

void disable_Astar_scheduler(void)
{
  //scheduler_del_event(id_scheduler);

  //id_scheduler = scheduler_add_periodical_event(adversary_detection_traj,NULL, 500000/SCHEDULER_UNIT);
}
