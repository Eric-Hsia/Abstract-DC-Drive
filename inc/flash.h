#ifndef _FLASH_H_
#define _FLASH_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Struct for storing settings of PID regulator
 */
struct pid_settings
{
    /** Proportional coefficient */
    int32_t P;
    /** Integral coefficient */
    int32_t I;
    /** Diferential coefficient */
    int32_t D;
    /** Coefficient on which will be divided all coefficients. Should be greater than 0 */
    int32_t div;
};

/**
 * Type of PID regulators
 */
enum pid_types 
{
    SPEED_PID = 0,
    CURRENT_PID
};

bool read_settings_flash(struct pid_settings *settings, enum pid_types type);

bool write_settings_flash(struct pid_settings *settings, enum pid_types type);

#endif // _FLASH_H_
