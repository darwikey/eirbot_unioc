#ifndef ANTIPATINAGE_H
#define ANTIPATINAGE_H

#include "aversive.h"
#include <stdint.h>
#include "position_manager.h"

void antipatinage_init(void);

void antipatinage_scheduler(void);

uint8_t roue_patine(void); 

#define RATIO 20
#define DIST_MIN ROBOT_IMP_CM/2

#endif // ANTIPATINAGE_H
