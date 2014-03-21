#include "gp2.h"
#include <aversive.h>
#include <wait.h>
#include <time.h>
#include <stdio.h>
#include <adc.h>
#include <math.h>
#include <stdint.h>
#include "position_manager.h"
#include "avoidance.h"
#include "astar.h"

extern position_manager_t pos;

// calcule la distance mesurée sur un GP2
// renvoie UINT16_MAX si la distance est supérieur à 90 cm
uint16_t gp2_get_dist(enum gp2_type type)
{
  uint8_t dist = 0;
  
  switch(type)
    {

    case GP2_RIGHT:
      {
	
	long tab[16] = {540, 500, 450, 400, 345, 305, 270, 240, 220, 205, 190, 175, 165, 154, 145, 135};

	// récupère les tensions des GP2
	long gp2_value = adc_get_value(ADC_REF_AVCC | MUX_ADC0);

	while(dist < 16 && gp2_value<tab[dist])
	  {
	    dist++;
	  }
      }
      break;
    }

  
  if(dist<16)
    return dist * 5 + 15;//retourne la distance arrondi au 5 cm supérieur
  else
    return UINT16_MAX;
}


// trouve la position dans le graphe de l'eventuelle point détecté par le gp2
// renvoie -1 s'il n'y a pas d'obstacle en face
int gp2_get_coor_obstacle(enum gp2_type type)
{
  // distance à l'obstacle
  uint16_t distance_gp2 = gp2_get_dist(type);

  // ne détecte pas les obstacle à plus de 80 cm
  if(distance_gp2 < 80)
    {
      
      printf("obstacle detecte (droite)\n");
      
      double angle = fxx_to_double(position_get_angle_deg(&pos));
      printf("value of angle %lf \n", angle);
      
      // prend en compte le decallage initiale
      double posx = fxx_to_double(position_get_y_cm(&pos)) + 20.0;
      double posy = fxx_to_double(position_get_x_cm(&pos)) + 20.0;
      
      int y = (int) (cos(angle * M_PI / 180.0) * (double)distance_gp2);  
      int x = (int) (sin(angle * M_PI / 180.0) * (double)distance_gp2);
      
      printf("dist obstacle : x: %d ,y:%d \n",x,y);
  
      printf("posx : %lf , posy :%lf \n",posx,posy);
      
      // Calcule la coordonné la plus proche d'un noeud
      int shift_x = (x + (int)posx) % UNIT >= UNIT/2;
      int shift_y = (y + (int)posy) % UNIT >= UNIT/2;
      
      //Now the value of x and y is in UNIT
      x = (x + posx) / UNIT + shift_x;
      y = (y + posy) / UNIT + shift_y;
      
      int coor = x  + G_LENGTH * y;

      if (coor >= 0 && coor < G_SIZE && !isOutOfGraphe(x, y))
	{
	  return coor;
	}
    }

  return -1;
}



