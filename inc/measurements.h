#ifndef _MEASUREMENTS_H_
#define _MEASUREMENTS_H_

#include <stdint.h>

void measurements_init(void);

void update_measurements(void);

int64_t get_encoder_value(void);

int64_t get_current_value(void);

void set_current_n_aver(uint32_t N);

#endif //_MEASUREMENTS_H_
