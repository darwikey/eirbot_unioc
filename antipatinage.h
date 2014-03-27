#ifndef ANTIPATINAGE_H
#define ANTIPATINAGE_H

#include "aversive.h"
#include <stdint.h>

void antipatinage_init(void);

void antipatinage_scheduler();

uint8_t roue_patine(void); 



#endif // ANTIPATINAGE_H
