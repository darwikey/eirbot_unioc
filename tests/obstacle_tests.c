#include <stdio.h>
#include <stdlib.h>

#include "../obstacle.h"

int main(int args, char* argv[]) {
  
  init_obstacle();
  printf("position %d %d -> %d\n",0,0,known_obstacle(0,0));
  printf("position %d %d -> %d\n",0,100,known_obstacle(0,100));
  printf("position %d %d -> %d\n",-80,100,known_obstacle(-80,100));
  printf("position %d %d -> %d\n",200,30,known_obstacle(200,30));
  printf("position %d %d -> %d\n",60,-20,known_obstacle(60,-20));
  printf("position %d %d -> %d\n",20,20,known_obstacle(20,20));


  return 0;
}
