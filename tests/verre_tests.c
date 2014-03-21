#include <stdio.h>
#include <stdlib.h>
#include "../verre.h"

int main(int argc, char**argv)
{
  int16_t x=140;
  int16_t y=100;
  int16_t i;
  glass_manager gm;
  gm.glass[12];
  glassManagerInit(&gm);
  for(i=0;i<12;i++)
    printf("%d x:%d y:%d\n",i,gm.glass[i].x,gm.glass[i].y);
  glass_t* closest=findClosest(&gm,x,y); 
  printf("position : x:%d y:%d\n", x, y);
  printf("closest : x:%d y:%d\n",closest->x,closest->y); 
  closest=findClosest(&gm,1,0);
  printf("position : x:%d y:%d\n", 1, 0);
  printf("closest : x:%d y:%d\n",closest->x,closest->y); 
  closest=findClosest(&gm,-25,120);
  printf("position : x:%d y:%d\n", -25, 120);
  printf("closest : x:%d y:%d\n",closest->x,closest->y); 
  closest=findClosest(&gm,-50,110);
  printf("position : x:%d y:%d\n", -50, 110);
  printf("closest : x:%d y:%d\n",closest->x,closest->y); 
  
  uint8_t nb_glass=takeGlass(&gm,&gm.glass[8]);
  for(i=0;i<gm.nb_glass_table;i++)
    printf("%d x:%d y:%d\n",i,gm.glass[i].x,gm.glass[i].y);
  printf("nb_glass : %d\n",nb_glass);

  return 0;
}
