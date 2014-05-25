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
  uint8_t offset = 0;
  
  
  long gp2_value = 0;
  

  if(type == GP2_RIGHT)
  {

    long tab[11] = {517 ,486 ,428 ,372 ,315 ,276 ,248 ,217 ,200 ,177 ,164};
	// récupère les tensions des GP2
    gp2_value = adc_get_value(ADC_REF_AVCC | MUX_ADC0);
    printf("gp2right: %ld  \n",gp2_value);
    // for(dist=0; dist<sizeof(tab)/sizeof(*tab); dist++)
    // {
    //   if(gp2_value<tab[dist])
    //     break;
    // }
    for(uint8_t i = 0;i < 11;i++)
    {
      if(gp2_value<tab[i])
        dist++;
    }

    offset = 5;
  }
  else if(type == GP2_LEFT)
  {
    long tab[11] = {552, 514, 444, 385, 328, 290, 259, 234, 212, 191, 178};

    gp2_value  = adc_get_value(ADC_REF_AVCC | MUX_ADC1);
    printf("gp2left: %ld \n ",gp2_value);

    for(uint8_t i = 0;i < 11;i++)
    {
      if(gp2_value<tab[i])
        dist++;
    }
  //on commence à 5 cm
  //  dist -= 2;
    offset = 5;
  }
  else if(type == GP2_MIDDLE)
  {
    long tab[3] = {616, 470, 320};

    gp2_value  = adc_get_value(ADC_REF_AVCC | MUX_ADC2);
    printf("gp2middle: %ld \n ",gp2_value);

    for(uint8_t i = 0;i < 3;i++)
    {
      if(gp2_value<tab[i])
        dist++;
    }
  //on commence à 5 cm
  //  dist -= 2;
    offset = 0;
  }
  printf("dist = %d \n", dist);

  if((type == GP2_RIGHT && dist<11) || (type == GP2_LEFT && dist < 11) || (type == GP2_MIDDLE && dist < 3))
    return dist * 5 + 10 + offset;//retourne la distance arrondi au 5 cm supérieur
  else
    return UINT16_MAX;
}


// trouve la position dans le graphe de l'eventuelle point détecté par le gp2, ne détecte pas à plus de distance_max
// renvoie -1 s'il n'y a pas d'obstacle en face
uint8_t gp2_get_coor_obstacle(enum gp2_type type, uint16_t distance_max)
{
  // distance à l'obstacle
  uint16_t distance_gp2 = gp2_get_dist(type);

  // ne détecte pas les obstacle à plus de distance_max
  if(distance_gp2 < distance_max)
  {

    if(type == GP2_LEFT)
    {
     // printf("obstacle detecte (gauche)\n");
    }
    if(type == GP2_RIGHT) 
    {
     // printf("obstacle detecte (droite)\n");
    }
    if(type == GP2_MIDDLE) 
    {
     // printf("obstacle detecte (milieu)\n");
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

    uint8_t coor = x  + G_LENGTH * y;

    if (coor >= 0 && coor < G_SIZE && !isOutOfGraphe(x, y))
    {
     return coor;
   }
 }

 return UINT8_MAX;
}