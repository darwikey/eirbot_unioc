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
  // distance entre le centre du robot et le gp2
  uint16_t offset = 0;
  
  
  long gp2_value;
  switch(type)
  {

    case GP2_RIGHT:
    {


      long tab[10] = {489,378,285,227,190,156 ,135,124,112,103};
	// récupère les tensions des GP2
      gp2_value = adc_get_value(ADC_REF_AVCC | MUX_ADC0);
      printf("gp2right: %ld  \n",gp2_value);
      while(dist < 10 && gp2_value<tab[dist])
      {
       dist++;
     }

     offset = 10;
   }
   break;
   case GP2_LEFT:
   {
    long tab[10] = {527, 409, 312, 247, 205, 174, 152, 133, 122, 109};

    gp2_value  = adc_get_value(ADC_REF_AVCC | MUX_ADC1);
    printf("gp2left: %ld \n ",gp2_value);

    while(dist < 10 && gp2_value<tab[dist])
    {
      dist++;
    }
  //on commence à 5 cm
  //  dist -= 2;
    offset = 10;
  }
  break;  
}
printf("dist = %d \n", dist);

if((type == GP2_RIGHT && dist<10) || (type == GP2_LEFT && dist < 10))
    return dist * 5 + 15 + offset;//retourne la distance arrondi au 5 cm supérieur
  else
    return UINT16_MAX;
}


// trouve la position dans le graphe de l'eventuelle point détecté par le gp2, ne détecte pas à plus de distance_max
// renvoie -1 s'il n'y a pas d'obstacle en face
int gp2_get_coor_obstacle(enum gp2_type type, uint16_t distance_max)
{
  // distance à l'obstacle
  uint16_t distance_gp2 = gp2_get_dist(type);

  // ne détecte pas les obstacle à plus de distance_max
  if(distance_gp2 < distance_max)
  {

    if(type == GP2_LEFT)
    {
        printf("obstacle detecte (gauche)\n");
    }
    if(type == GP2_RIGHT) 
      {printf("obstacle detecte (droite)\n");
  }
  double angle = fxx_to_double(position_get_angle_deg(&pos));
      //printf("value of angle %lf \n", angle);

      // prend en compte le decallage initiale
  double posx = fxx_to_double(position_get_y_cm(&pos));
  double posy = fxx_to_double(position_get_x_cm(&pos));

  int y = (int) (cos(angle * M_PI / 180.0) * (double)distance_gp2);  
  int x = (int) (sin(angle * M_PI / 180.0) * (double)distance_gp2);

  // if(type == GP2_LEFT)
  // {
  //   y += (int) (sin(M_PI - angle * M_PI / 180.0) * 10.0);
  //   x += (int) (cos(M_PI - angle * M_PI / 180.0) * 10.0);
  // }
  // else if(type == GP2_RIGHT)
  // {
  //   y += (int) (sin(M_PI - angle * M_PI / 180.0) * -10.0);
  //   x += (int) (cos(M_PI - angle * M_PI / 180.0) * -10.0);
  // }
      printf("dist obstacle : x: %d ,y:%d \n",x,y);
  
      //printf("posx : %lf , posy :%lf \n",posx,posy);

      // Calcule la coordonnée la plus proche d'un noeud
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



