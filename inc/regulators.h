#ifndef _REGULATORS_H_
#define _REGULATORS_H_

#include <stdint.h>

/** Struct for decoding settings-changing message from CAN */
struct pid_settings_msg 
{
    /** PID Type according to :c:type:`pid_types`. */
    uint8_t pid_type;
    
    /** PID Field according to :c:type:`pid_fields` */
    uint8_t pid_field;
    
    /** Value to write */
    int32_t value;
} __attribute__((packed));

/** Struct for decoding desire-value-changing message from CAN */
struct pid_des_value_msg
{
    /** PID Type according to :c:type:`pid_types`. */
    uint8_t pid_type;
    
    /** Value to write */
    int32_t value;
} __attribute__((packed));

void regulators_init(void);

void regulators_update(void);

void change_pid_settings(uint8_t data[], uint8_t N);

void set_desired_value(uint8_t data[], uint8_t N);

int32_t regulator_get_fd_speed(void);

int32_t regulator_get_motor_v(void);

#endif //_REGULATORS_H_
