#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

void motor_init(void); 

void motor_set_pwm(int16_t input);

#endif // _MOTOR_H_
