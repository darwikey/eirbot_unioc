#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <SDL2\SDL.h>
#include "algo.h"
#define W_LENGTH 903
#define W_WIDTH 603

int main(int argc, char *argv[])
{
	//probleme de mvt inutile minizigzag inutile verifier les calculs de distances distances
	//if (argc == 5)
	{
		
		//node graphe[LENGTH*WIDTH] = { 0 };
		node graphe[G_SIZE] = { 0 };
		int i = 0;
		for (i = 0; i < G_SIZE; i++)
		{
			graphe[i].coor = i;
		}
		//graphe[G_LENGTH * 5 + 5].type = OBSTACLE;
		int startCoor = G_LENGTH*1 + 1;//atoi(argv[1]) + LENGTH *atoi(argv[2]);
		int trueGoal;//put the true goal and do a function who give a near node
		int goalCoor = G_LENGTH * 10 + 10;//atoi(argv[3]) + LENGTH *atoi(argv[4]);
			//put the obstacle in the graphe			
		
		aStarLoop(graphe, startCoor, goalCoor);

	}
	return EXIT_SUCCESS;
}