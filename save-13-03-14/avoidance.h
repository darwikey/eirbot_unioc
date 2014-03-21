#ifndef AVOIDANCE_H
#define AVOIDANCE_H


void avoidance_init(void);

void go_to_node(double x,double y);

void adversary_detection_Astar(void* p);
void adversary_detection_traj(void* p);

int new_obstacle(long gp2Value,double angle,double posx,double  posy);

int8_t isObstacle(int x,int y);
int8_t isOutOfGraphe(int x,int y);






#endif
