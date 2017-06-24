/******************** (C) COPYRIGHT 2015 Cyrus ***************************
 * ×÷Õß		 £ºCyrus
 * ÃèÊö    £ºBmp085
**********************************************************************************/
#include "AC_Baro_Buffer.h"

#define FILTER_SIZE 7

static u8 _new_data;
static u8 sample_index;
static uint32_t _timestamps[FILTER_SIZE];
static float samples[FILTER_SIZE];
static float _last_slope;
void _climb_rate_filter_update(float sample, uint32_t timestamp)
{
	uint8_t i = sample_index;
	uint8_t i1;
	if (i == 0) {
			i1 = FILTER_SIZE-1;
	} else {
			i1 = i-1;
	}
	if (_timestamps[i1] == timestamp) {
			// this is not a new timestamp - ignore
			return;
	}
	// add timestamp before we apply to FilterWithBuffer
	_timestamps[i] = timestamp;

	// call parent's apply function to get the sample into the array
	samples[sample_index++] = sample;

	// wrap index if necessary
	if( sample_index >= FILTER_SIZE )
			sample_index = 0;
	
	_new_data = 1;
}

#define f(i) samples[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]
#define x(i) _timestamps[(((sample_index-1)+i+1)+3*FILTER_SIZE/2) % FILTER_SIZE]
float _baro_get_climb_rate(void)
{
	if (!_new_data) {
			return _last_slope;
	}
	
	float result = 0;

	if (_timestamps[FILTER_SIZE-1] == _timestamps[FILTER_SIZE-2]) {
			// we haven't filled the buffer yet - assume zero derivative
			return 0;
    }
	result = 2*5*(f(1) - f(-1)) / (x(1) - x(-1))
					 + 4*4*(f(2) - f(-2)) / (x(2) - x(-2))
					 + 6*1*(f(3) - f(-3)) / (x(3) - x(-3));
	result /= 32;	
		
	// cope with numerical errors
	if (isnan(result) || isinf(result)) {
			result = 0;
	}

	_new_data = 0;
	_last_slope = result;

	return result;		
}

/////////////////////////////////////////////////////////////////////
#define SIZE 10
uint8_t _num_items;             // number of items in the buffer
uint8_t _head;                  // first item in the buffer (will be returned with the next pop_front call)
float _buff[SIZE];            // x values of each point on the curve

void baro_buffer_clear(void) 
{
	// clear the curve
	_num_items = 0;
  _head = 0;
}
uint8_t _hist_position_estimate_z_is_full(void)
{
	return _num_items >= SIZE;
}
float baro_get_peek(u8 position)
{
	uint8_t j = _head + position;

	// return zero if position is out of range
	if( position >= _num_items ) {
		const static float r = 0;
		return r;
	}

	// wrap around if necessary
	if( j >= SIZE )
			j -= SIZE;

	// return desired value
	return _buff[j];
}
void baro_buffer_push_back(float item )
{
	// determine position of new item
	uint8_t tail = _head + _num_items;
	if( tail >= SIZE ) {
			tail -= SIZE;
	}

	// add item to buffer
	_buff[tail] = item;

	// increment number of items
	if( _num_items < SIZE ) {
			_num_items++;
	}else{
			// no room for new items so drop oldest item
			_head++;
			if( _head >= SIZE ) {
					_head = 0;
			}
	}
}
/******************* (C) COPYRIGHT 2015 Cyrus *****END OF FILE************/
