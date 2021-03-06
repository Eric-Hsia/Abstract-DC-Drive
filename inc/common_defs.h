#ifndef _COMMON_DEFS_H_
#define _COMMON_DEFS_H_

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
    /** Coefficient on which will be divided all coefficients. Must not be 0 */
    int32_t div;
    /** Find average in N last measurment before using as feedback signal */
    int32_t aver_N;
};

/**
 * Type of PID regulators
 */
enum pid_types 
{
    /** None, disable */
    NONE = 0,
    /** Speed control */
    SPEED_PID = 1,
    /** Current control */
    CURRENT_PID,
    /** Position control */
    POSITION_PID,
    /** Direct voltage control */
    DIRECT_CONT,
};

/** Fields of pid settings */
enum pid_fields
{
    /** P field */
    PID_FIELD_P = 0,
    /** I field */
    PID_FIELD_I,
    /** D field */
    PID_FIELD_D,
    /** div field */
    PID_FIELD_DIV,
    /** aver_N field */
    PID_FIELD_AVER_N,
};

/** Info data to send throught CAN */
enum log_types
{
    /** Position */
    LOG_POSITION = 0,
    /** Feedback speed */
    LOG_SPEED,
    /** Feedback current */
    LOG_CURRENT,
    /** Temperature */
    LOG_TEMPERATURE,
    /** Voltage */
    LOG_VOLTAGE,
};

#endif // _COMMON_DEFS_H_
