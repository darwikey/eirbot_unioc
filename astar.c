#include "astar.h"
#include <stdio.h>

extern trajectory_manager_t traj;

//graphe où sera charge la table et ses obstacles
node graphe[G_SIZE] = { 0 };
uint8_t startCoor;
uint8_t goalCoor;

int stopMovement = 0;

uint8_t aStarLoop()
{
  coordinate coor;
  uint8_t i;
  uint8_t neighbors[8];
  graphe[startCoor].type = OPENLIST; //OPENLIST is list of node we have to analysed//list is a type 
  graphe[startCoor].remainDist = findDist(graphe[startCoor], graphe[goalCoor]);
  uint8_t current = startCoor;
  
  while (graphe[goalCoor].type != CLOSEDLIST)//closed list is nodes which are already analysed
  {
      //init neighbors
    initNeighbors(current, neighbors);
    for (i = 0; i < 8; i++)
    {
     if (graphe[neighbors[i]].type != OBSTACLE && graphe[neighbors[i]].type != CLOSEDLIST && neighbors[i] != OUT)
     {
       if (graphe[neighbors[i]].type == OPENLIST)
       {
		  //update the node if it's already in the openlist
        if (graphe[neighbors[i]].crossedDist > findDist(graphe[neighbors[i]], graphe[current]) + graphe[current].crossedDist)
        {
          graphe[neighbors[i]].parent = &graphe[graphe[current].coor];
          graphe[neighbors[i]].crossedDist = findDist(graphe[neighbors[i]], graphe[current]) + graphe[current].crossedDist;
        }
      }
	      else//add it
        {
          graphe[neighbors[i]].parent = &graphe[graphe[current].coor];
          graphe[neighbors[i]].remainDist = findDist(graphe[neighbors[i]], graphe[goalCoor]);
          graphe[neighbors[i]].crossedDist = findDist(graphe[neighbors[i]], graphe[current]) + graphe[current].crossedDist;
          graphe[neighbors[i]].type = OPENLIST;
        }
      }
    }
    if(isVoid(OPENLIST))
    {

     break;
   }
   current = findBest(OPENLIST);
   coor = getCoor(graphe[current]);
      //printf("(%d,%d) \n", coor.x, coor.y);
      //put the analysed node in the closedList
   graphe[current].type = CLOSEDLIST;
 }
 if (graphe[goalCoor].parent == NULL)
 {
  printf("path not found");
  return 0;
}
else
{
      //reconstruct the path
  printf("path: \n");
  while (current != startCoor)
  {
   coor = getCoor(graphe[current]);
   printf("(%d,%d), ", coor.x, coor.y);
   current = graphe[current].parent->coor;
 }
 coor = getCoor(graphe[startCoor]);
 printf("(%d,%d), ", coor.x, coor.y);
 return 1;
}
}

uint8_t findDist(node node1, node node2)
{
  coordinate c1 = getCoor(node1);
  coordinate c2 = getCoor(node2);
  //trial with manhattan dist
  //upgrade
  uint8_t dist = abs(c1.x - c2.x) + abs(c1.y - c2.y);
  if (abs(c1.x - c2.x) == 2 && abs(c1.y - c2.y) == 2)
  {
    dist = 3;
  }
  //we can change with defined dist for the neighbors dist
  return dist;
}

void addElement(nodeList *list, node n)
{
  nodeListElement *new = malloc(sizeof(*new));

  nodeListElement   *current = list->firstElement;
  if (list->firstElement == NULL)
  {
    list->firstElement = new;
    new->next = NULL;
    new->n = n;
  }
  else
  {
    while (current->next != NULL)
    {
     current = current->next;
   }
   current->next = new;
   new->next = NULL;
   new->n = n;
 }
}

void rmElement(nodeList *list, node nd)
{
  nodeListElement *current = list->firstElement;
  nodeListElement *deleted = NULL;
  if (nd.coor == current->n.coor)
  {
    deleted = current;
    list->firstElement = current->next;
  }
  else
  {
    while (current->next != NULL)
    {
     if (nd.coor == current->next->n.coor)
     {
       deleted = current->next;
       current->next = current->next->next;
       break;
     }
     current = current->next;
   }
 }
 if (deleted != NULL)
 {
  free(deleted);
}
}


int8_t isVoid(int8_t list)
{
  for(uint8_t i = 1; i < G_SIZE; i++)
  {
    if (graphe[i].type == list)
    {
     return 0;
   }
 }
 return 1;
}

void initNeighbors(uint8_t current, uint8_t *neighbors)
{

  neighbors[0] = current - 1*G_LENGTH - 1;
  neighbors[1] = current - 1*G_LENGTH;
  neighbors[2] = current - 1*G_LENGTH + 1;
  neighbors[3] = current - 1;
  neighbors[4] = current + 1;
  neighbors[5] = current + 1*G_LENGTH - 1;
  neighbors[6] = current + 1*G_LENGTH;
  neighbors[7] = current + 1*G_LENGTH + 1;

  if (current - 1*G_LENGTH - 1 > G_SIZE || current - 1*G_LENGTH - 1 < 0 || current%G_LENGTH < 1)
  {
    neighbors[0] = OUT;
  }
  if (current - 1*G_LENGTH > G_SIZE || current - 1*G_LENGTH < 0)
  {
    neighbors[1] = OUT;
  }
  if (current - 1*G_LENGTH + 1 > G_SIZE || current - 1*G_LENGTH + 1 < 0 || current%G_LENGTH >= G_LENGTH - 1)
  {
    neighbors[2] = OUT;
  }
  if (current - 1 > G_SIZE || current - 1 < 0 || current%G_LENGTH < 1)
  {
    neighbors[3] = OUT;
  }
  if (current + 1 > G_SIZE || current + 1 < 0 || current%G_LENGTH >= G_LENGTH - 1)
  {
    neighbors[4] = OUT;
  }
  if (current + 1*G_LENGTH - 1 > G_SIZE || current + 1*G_LENGTH - 1 < 0 || current%G_LENGTH < 1)
  {
    neighbors[5] = OUT;
  }
  if (current + 1*G_LENGTH > G_SIZE || current + 1*G_LENGTH < 0)
  {
    neighbors[6] = OUT;
  }
  if (current + 1*G_LENGTH + 1 > G_SIZE || current + 1*G_LENGTH + 1 < 0 || current%G_LENGTH >= G_LENGTH - 1)
  {
    neighbors[7] = OUT;
  }
}

uint8_t findBest(uint8_t openlist)
{
  uint8_t best = 0;
  uint8_t max = 0xff;//changer si trop couteux
  uint8_t i = 0;
  while (i<=G_SIZE)
  {
    if (graphe[i].type == openlist)
    {
     if (graphe[i].crossedDist + graphe[i].remainDist < max)
     {
       max = graphe[i].crossedDist + graphe[i].remainDist;
       best = graphe[i].coor;
     }

   }
   i++;
 }

 return best;
}

coordinate getCoor(node n)
{
  coordinate c;
  c.x = n.coor % G_LENGTH;
  c.y = n.coor / G_LENGTH;
  return c;
}


void initObstacle()
{
  int i;
  for(i = 0; i < G_LENGTH;i++)
  {
    graphe[i].type = OBSTACLE; 
    graphe[(G_WIDTH-1)*G_LENGTH + i].type = OBSTACLE;  
  }
  for(i=0;i<G_WIDTH;i++) 
  { 
    graphe[i*G_LENGTH].type = OBSTACLE;
    graphe[i*G_LENGTH + G_LENGTH-1].type = OBSTACLE;
  } 
  for(i=2;i<7;i++)
  {
    graphe[G_LENGTH +i ].type=OBSTACLE;
    graphe[2*G_LENGTH +i ].type=OBSTACLE; 
    graphe[G_LENGTH +i+7 ].type=OBSTACLE; 
    graphe[2*G_LENGTH +i+7 ].type=OBSTACLE; 
  }
  for(i=6;i<10;i++) 
  { 
    graphe[5*G_LENGTH +i ].type=OBSTACLE;
    graphe[6*G_LENGTH +i ].type=OBSTACLE;
    graphe[4*G_LENGTH +i ].type=OBSTACLE;
    graphe[7*G_LENGTH +i ].type=OBSTACLE;
  }
  //test
  //graphe[3*G_LENGTH +6 ].type=OBSTACLE;
  //
  //arbre
  graphe[4*G_LENGTH +1 ].type=OBSTACLE;
  graphe[4*G_LENGTH +14 ].type=OBSTACLE;
  for(i = 0; i <4;i++)
  { 
    graphe[(i+6)*G_LENGTH + 1 ].type=OBSTACLE;
    graphe[(i+6)*G_LENGTH + 14 ].type=OBSTACLE;
    graphe[9*G_LENGTH + 11 + i ].type=OBSTACLE;
    graphe[9*G_LENGTH + 1 + i ].type=OBSTACLE;
  } 
}


// diminue le nombre de point dans la trajectoire
void polishing(mvStack *s) 
{
  node current = graphe[goalCoor];
  uint8_t type =10;
  mvStackElement e = {0,0};
  mvStackElement r = {0,0};
  while (current.coor != startCoor)
  {
      //printf("polishing %d %d \n",current.coor,type);
      if(current.parent->coor == current.coor - 1*G_LENGTH - 1)//0
	//012
	//3x4
	//567
      {
       if (type == 0)
       {
         e.val += 1.41421356f;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);
       push(s,r);	    
       r.val=1;
	      r.type = ROTATE; //2= ROTATE


	      type = 0;
	      e.type = DRIVE;
	      e.val = 1.41421356f;
	    }
   }
      else if(current.parent->coor == current.coor - 1*G_LENGTH)//1
      {	  
       if (type == 1)
       {
         e.val++;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);
       push(s,r);	    
       r.val=0;
	      r.type = ROTATE; //2= ROTATE

	      
	      type = 1;
	      e.type = DRIVE;
	      e.val = 1;
	    }
   }
      else if(current.parent->coor == current.coor - 1*G_LENGTH + 1)//2
      {
       if (type == 2)
       {
         e.val += 1.41421356f;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);
       push(s,r);      	    
       r.val=-1;
	      r.type = ROTATE; //2= ROTATE


	      type = 2;
	      e.type = DRIVE;
	      e.val = 1.41421356f;
	    }
   }
      else if(current.parent->coor == current.coor - 1)//3
      {	 
       if (type == 3)
       {
         e.val ++;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);
       push(s,r);	    
       r.val=2;
	      r.type = ROTATE; //2= ROTATE


	      type = 3;
	      e.type = DRIVE;
	      e.val = 1;
	    }
   }
      else if(current.parent->coor == current.coor + 1)//4
      {	 
       if (type == 4)
       {
         e.val++;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);
       push(s,r);     	
       r.val = -2;
	      r.type = ROTATE; //2= ROTATE


	      type = 4;
	      e.type = DRIVE;
	      e.val = 1;
	    }
   }
      else if(current.parent->coor == current.coor + 1*G_LENGTH - 1)//5
      {
       if (type == 5)
       {
         e.val+= 1.41421356f;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);
       push(s,r);
       r.val=3;
	      r.type = ROTATE; //2= ROTATE
	      

	      type = 5;
	      e.type = DRIVE;
	      e.val = 1.41421356f;
	    }	  
   }
      else if(current.parent->coor == current.coor + 1*G_LENGTH)//6
      {
       if (type == 6)
       {
         e.val++;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);	   
       push(s,r);
       r.val=4;
	      r.type = ROTATE; //2= ROTATE
	      

	      type = 6;
	      e.type = DRIVE;
	      e.val = 1;
	    }
   }
      else if(current.parent->coor == current.coor + 1*G_LENGTH + 1)//7
      {
       if (type == 7)	
       {
         e.val+=1.41421356f;
	      e.type = DRIVE;//1 = DRIVE
	    }
     else
     {
       push(s,e);
       push(s,r);
       r.val=-3;
	      r.type = ROTATE; //2= ROTATE

	      type = 7;
	      e.type = DRIVE;
	      e.val = 1.41421356f;
	    }
   }
   else
   {
     printf("Error path not found\n");
   }
   current = *(current.parent);
 }
 push(s,e);
 push(s,r);
}

//stack primitive
mvStackElement pop(mvStack *s)
{
  if(stack_is_empty(s))
    exit(1);
  s->top--;
  return s->items[s->top];
}

void push(mvStack *s,mvStackElement item)
{
  if(stack_full(s))
    exit(1);
  s->items[s->top++]=item;
}

uint8_t stack_size(mvStack *s)
{
  return s->top;
}

uint8_t stack_full(mvStack *s)
{
  return s->top  == STACK_SIZE;
}

uint8_t stack_is_empty(mvStack *s)
{
  return !s->top;
}
void stack_clear(mvStack *s)
{
  s->top = 0;
  for(int i = 0; i<STACK_SIZE;i++)
  {
    s->items[i].val = 0.f;
    s->items[i].type = 0;
  }
}

mvStack initStack(void)
{
  mvStack s;
  s.top = 0;
  return s;
}



mvStack stack;//pile d'instructions

int8_t astarMv()
{
  stopMovement = 0;
  
  set_detection_behaviour(BEHAVIOUR_ASTAR); 
  //init graphe
  uint8_t i = 0;
  for (i = 0; i < G_SIZE; i++)
  {
    graphe[i].coor = i;
    graphe[i].parent = 0;
    graphe[i].remainDist = 0;
    graphe[i].crossedDist = 0;
    if(graphe[i].type != OBSTACLE)
    {
     graphe[i].type = 0;
   }
 }
 printf("astar test \n");	

 int8_t bool = aStarLoop();

  //si pas de chemin trouve 
 if(bool == 0)
 {
  printf("no path found \n");
  return 0;
}


  stack_clear(&stack);//pile d'instructions
  
  polishing(&stack);
  mvStackElement currentElement;
  printf(" begin stack \n");
  currentElement.type=1;
  currentElement.val=0;
  while(currentElement.type != 0 && !stopMovement)
  {
    currentElement = pop(&stack);
    printf ("stackelement:%f,%d \n",currentElement.val,currentElement.type);
    printf("Starting movement\n");
    if(currentElement.type == DRIVE)
    {
     trajectory_goto_d(&traj, END, currentElement.val*UNIT);

   }
   if(currentElement.type == ROTATE)
   {
     trajectory_goto_a(&traj, END, currentElement.val*45);
   }

 }
 while(!trajectory_is_ended(&traj));
 printf("end stack \n");

 if(stopMovement)
 {
  return 0;
}
set_detection_behaviour(BEHAVIOUR_STOP);
return 1;
}


uint8_t get_startCoor()
{
  return startCoor;
}

uint8_t get_goalCoor()
{
  return goalCoor;
}

void set_startCoor(uint8_t coor)
{
  startCoor = coor;
}

void set_goalCoor(uint8_t coor)
{
  goalCoor = coor;
}

void stopAstarMovement()
{
  stopMovement = 1;
  stack_clear(&stack);
}

void printGraphe(void)
{
  for (int i = 0; i < G_SIZE; ++i)
  {
    if(i%G_LENGTH == 0)
    {
      printf("\n");
    }
    //if(graphe[i].parent == 0)
    {
      printf("%d ",graphe[i].type);
    }
    //else
    {
      //printf("%d\t",graphe[i].parent->coor);
    }
  }
}
