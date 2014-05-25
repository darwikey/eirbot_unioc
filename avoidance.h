#ifndef AVOIDANCE_H
#define AVOIDANCE_H


enum detection_behaviour_t
  {
    // arrête le robot en cas de détection adverse
    BEHAVIOUR_STOP,
    // relance un A* en cas de détection adverse
    BEHAVIOUR_ASTAR,
  };

void avoidance_init(void);

void go_to_node(double x,double y);

void adversary_detection_traj(void* p);

int new_obstacle(uint16_t);

int8_t isObstacle(int x,int y);
int8_t isOutOfGraphe(int x,int y);

uint8_t obstacleInTrajectory(uint8_t startCoor,uint8_t goalCoor);

void set_detection_behaviour(enum detection_behaviour_t);

void disableAvoidance(void);

void clearGraphe(void);

void enableAvoidance(void);
#endif
