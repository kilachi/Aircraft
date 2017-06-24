#ifndef _AC_THROTTLE_H
#define _AC_THROTTLE_H

#include "Aircraft_Config.h"

void throttle_channel(void);
void throttle_setmode(u8 mode);
void throttle_update(float T);
void stabilize_run(float T);
uint8_t throttle_check(void);
void throttle_idling(float thr);

#endif

