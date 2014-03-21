#include "../evitement.h"
#include "../obstacle.h"

// CRAAADDDDEEEE !
#include "../obstacle.c"

#include <stdio.h>

void print_pos(struct pos* pts) {
  printf("(%d, %d)[%d]", pts->x, pts->y, pts->a);
}

void print_chemin(chemin* path) {
  if(path != NULL) {
    print_pos(&path->target);
    printf("->");
    print_chemin(path->next);
  }
  else {
    printf("\n");
  }
}

int main(int argc, char* argv) {
  init_obstacle();

  chemin* path = NULL;
  struct pos cur, target;
  cur.x = 0;
  cur.y = 0;
  target.x = 0;
  target.y = 200;
  
  // Contournement du gateau
  path = evitement_get_chemin(NULL, 0, &cur, &target);
  print_chemin(path);

  // Ajout d'un obstacle
  obstacle obs;
  MK_CIRCLE(obs, 40, 70, 30);
 

  path = evitement_get_chemin(&obs, 1, &cur, &target);
  print_chemin(path);

  return 0;
}