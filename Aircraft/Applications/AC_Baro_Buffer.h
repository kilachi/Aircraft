#ifndef __AC_BARO_BUFFER_H
#define __AC_BARO_BUFFER_H

#include "Aircraft_Config.h"

void _climb_rate_filter_update(float sample, uint32_t timestamp);
float _baro_get_climb_rate(void);

void baro_buffer_clear(void);
uint8_t _hist_position_estimate_z_is_full(void);
float baro_get_peek(u8 position);
void baro_buffer_push_back(float item );


#endif
