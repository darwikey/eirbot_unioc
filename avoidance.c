
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
enum detection_behaviour_t detection_behaviour;


void avoidance_init(void)
{

  id_scheduler = scheduler_add_periodical_event(adversary_detection_traj,NULL, 500000/SCHEDULER_UNIT);

  detection_behaviour = BEHAVIOUR_ASTAR;
}


// fonction appelée par le scheduler
// 
void adversary_detection_traj(void*p)
{
  // désactive le scheduler pour éviter qu'il se déclenche pendant le calcul
  scheduler_del_event(id_scheduler);

  int coorD = gp2_get_coor_obstacle(GP2_RIGHT, 40);

  int coorG = gp2_get_coor_obstacle(GP2_LEFT,40);

  // check si on a détecté quelque chose

  if (coorD != -1 || coorG != -1)
  { 
    uint8_t obstacle = 0;
    int coor;
    if(coorD != -1)
    {
     obstacle = graphe[coorD].type == OBSTACLE;
     coor = coorD;
     printf("coorD%d\n",coorD );
   }
   if(coorG != -1 && obstacle != 0)
   {
     obstacle = graphe[coorD].type == OBSTACLE;
     coor = coorG;
     printf("coorG %d\n",coorG );
   }


   printf("adversary detection :  noeud : %d, %d \n", coor % G_LENGTH, coor / G_LENGTH);

      // Si c'est un obstacle non connu on panique
   if(!obstacle)
   {
     printf("stop the trajectory \n");

     switch (detection_behaviour)
     {
       case BEHAVIOUR_STOP:
       trajectory_pause(&traj);
       break;

       case BEHAVIOUR_ASTAR:
       if(new_obstacle(coor))
       {
		  if(obstacleInTrajectory(get_startCoor(),get_goalCoor()))//need goalcoor
      {
        
        trajectory_reinit(&traj);
        asserv_stop(&asserv);
        
        stopAstarMovement();

		      //trajectory_resume(&traj);
        printf("succeed to stop trajectory \n");
        
  		      go_to_node(fxx_to_double(position_get_y_cm(&pos)),fxx_to_double(position_get_x_cm(&pos)));//attention x et y inverser
          }
        }  
        else
        {
          printf("OK obstacle doesn't matter\n");
        }
        break;
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
   }

  // réactive le scheduler
   id_scheduler = scheduler_add_periodical_event(adversary_detection_traj,NULL, 500000/SCHEDULER_UNIT);
 }

 
// Planifie une trajectoire pour aller au noeud le plus proche 
//not optimized because le robot retourne sur ses pas

 void go_to_node(double position_x, double position_y)
 {
  int x = (int) position_x;
  int y = (int) position_y;
  printf("go_to_node ;   actuellement : (x: %d , y: %d);   ",x/UNIT, y/UNIT);
  
  uint8_t goalCoor = get_goalCoor();
  uint8_t bestCoor = 0;
  uint16_t bestDist = UINT16_MAX;

  if(!isOutOfGraphe(x, y) && !isObstacle(x, y))
  {
    uint16_t dist = ((goalCoor%G_LENGTH) - x/UNIT)*((goalCoor%G_LENGTH) - x/UNIT) 
    + (goalCoor/G_LENGTH - y/UNIT)*(goalCoor/G_LENGTH - y/UNIT);
    if(dist < bestDist)
    {
     bestDist = dist;
     bestCoor = x/UNIT + G_LENGTH*(y/UNIT);
   }
 }
 else if(!isOutOfGraphe(x+1, y) && !isObstacle(x+1, y))
 {
   
  uint16_t dist = ((goalCoor%G_LENGTH) - 1 + x/UNIT)*((goalCoor%G_LENGTH) - 1 + x/UNIT) 
  + (goalCoor/G_LENGTH - y/UNIT)*(goalCoor/G_LENGTH - y/UNIT);
  if(dist < bestDist)
  {
   bestDist = dist;
   bestCoor = (1+x)/UNIT + G_LENGTH*(y/UNIT);
 } 
}
else if(!isOutOfGraphe(x, y+1) && !isObstacle(x, y+1))
{

  uint16_t dist = ((goalCoor%G_LENGTH) - x/UNIT)*((goalCoor%G_LENGTH) - x/UNIT) 
  + (goalCoor/G_LENGTH - 1 + y/UNIT)*(goalCoor/G_LENGTH - 1+ y/UNIT);
  if(dist < bestDist)
  {
   bestDist = dist;
   bestCoor = x/UNIT + G_LENGTH*(1 + y/UNIT);
 }
}
else if(!isOutOfGraphe(x+1, y+1) && !isObstacle(x+1, y+1)) 
{
  
  uint16_t dist = ((goalCoor%G_LENGTH) - 1 + x/UNIT)*((goalCoor%G_LENGTH) - 1 + x/UNIT) 
  + (goalCoor/G_LENGTH - 1 + y/UNIT)*(goalCoor/G_LENGTH - 1 + y/UNIT);
  if(dist < bestDist)
  {
   bestDist = dist;
   bestCoor = x/UNIT + 1 + G_LENGTH*(1 + y/UNIT);
 }
}
else
{
  printf("echec go_to_node !!!\n");
}

if(bestCoor)
{
  
  printf("bestCoor : %d \n",bestCoor);
  double angle = atan2(UNIT * (bestCoor/G_LENGTH) - y,UNIT * (bestCoor%G_LENGTH) - x);
  uint16_t dist =  (UNIT * (bestCoor/G_LENGTH) - y)*(UNIT * (bestCoor/G_LENGTH) - y) + 
  (UNIT * (bestCoor%G_LENGTH) - x)*(UNIT * (bestCoor%G_LENGTH) - x);
  angle = 180.0*angle/M_PI;
  
  
  printf("dist : %d ,angle : %lf \n",dist,angle);
  
  trajectory_goto_a(&traj, END, angle);
  trajectory_goto_d(&traj, END,sqrt((double)dist));
  
  set_startCoor(bestCoor);
  
  printf("startCoor : (%d, %d)\n", get_startCoor() % G_LENGTH, get_startCoor() / G_LENGTH);
}
}


// place dynamiquement les nouveaux obstacles sur le graphe
int new_obstacle(uint16_t coor)
{
  // check si on sort pas du tableau
  if (coor != -1)
  {
    if(graphe[coor].type == OBSTACLE)
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

int x = coor % G_LENGTH;
int y = coor / G_LENGTH;

uint8_t coor_pos = position_get_coor(&pos);


  // On place dans les cases adjacentes au truc détécté des obstacles
for (int i = -1; i <= 1; i++)
{
  for (int j = -1; j <= 1; j++)
  {
   uint8_t index = x + i + G_LENGTH*(j + y); 
   if (index >= 0 
     && index < G_SIZE
     && !isOutOfGraphe(index % G_LENGTH, index / G_LENGTH)
     && index != coor_pos)
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


void set_detection_behaviour(enum detection_behaviour_t behaviour)
{
  detection_behaviour = behaviour;
}


//search if there is an obstacle in the trajectory
uint8_t obstacleInTrajectory(uint8_t startCoor,uint8_t goalCoor)
{
  uint8_t current = goalCoor;
  while(current != startCoor)
  {
    if(graphe[current].type == OBSTACLE)
     {printf("obstacle in the trajectory \n");
   return 1;
 }
 current = graphe[current].parent->coor;
}
return 0;
}

