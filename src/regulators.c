#include "regulators.h"
#include "flash.h"
#include "motor.h"
#include "measurements.h"

#include <abstractSTM32.h>
#include <abstractLOG.h>
#include <int_PID.h>

static volatile int64_t des_speed = 0;
static volatile int64_t fd_speed = 0;
static volatile int64_t motor_v = 0;
static volatile int64_t time = 0;

struct int_pid PID_speed = {
    .P = 0,
    .D = 0,
    .I = 0,
    .div = 0,
    .desired = &des_speed,
    .in = &fd_speed,
    .out = &motor_v,
    .time = &time
};

static volatile int64_t des_current = 0;
static volatile int64_t fd_current = 0;

struct int_pid PID_current = {
    .P = 0,
    .D = 0,
    .I = 0,
    .div = 0,
    .desired = &des_current,
    .in = &fd_current,
    .out = &motor_v,
    .time = &time
};

static volatile int64_t des_position = 0;
static volatile int64_t fd_position = 0;

struct int_pid PID_position = {
    .P = 0,
    .D = 0,
    .I = 0,
    .div = 0,
    .desired = &des_position,
    .in = &fd_position,
    .out = &motor_v,
    .time = &time
};

static volatile enum pid_types pid_mode = NONE;

static void speed_pid_update(void);
static void current_pid_update(void);
static void position_pid_update(void);
static void save_settings_to_flash(uint8_t pid_type);

/**
 * Initialize regulators
 */
void regulators_init(void)
{
    struct pid_settings buff = {0};
    if (read_settings_flash(&buff, SPEED_PID)) {
        PID_speed.P = buff.P;
        PID_speed.I = buff.I;
        PID_speed.D = buff.D;
        PID_speed.div = buff.div;
        
        abst_logf("Loaded Speed PID\n");
        abst_logf("P: %i, I: %i, D: %i, div: %i\n",
                  (int)PID_speed.P, 
                  (int)PID_speed.I, 
                  (int)PID_speed.D, 
                  (int)PID_speed.div);
    }
    if (read_settings_flash(&buff, CURRENT_PID)) {
        PID_current.P = buff.P;
        PID_current.I = buff.I;
        PID_current.D = buff.D;
        PID_current.div = buff.div;
        set_current_n_aver(buff.aver_N); // In measurements.c
        
        abst_logf("Loaded Current PID\n");
        abst_logf("P: %i, I: %i, D: %i, div: %i\n",
                  (int)PID_current.P, 
                  (int)PID_current.I, 
                  (int)PID_current.D, 
                  (int)PID_current.div);
    }
    
    if (read_settings_flash(&buff, POSITION_PID)) {
        PID_position.P = buff.P;
        PID_position.I = buff.I;
        PID_position.D = buff.D;
        PID_position.div = buff.div;
        
        abst_logf("Loaded Speed PID\n");
        abst_logf("P: %i, I: %i, D: %i, div: %i\n",
                  (int)PID_position.P, 
                  (int)PID_position.I, 
                  (int)PID_position.D, 
                  (int)PID_position.div);
    }
    
    pid_init(&PID_speed);
    pid_init(&PID_current);
    pid_init(&PID_position);
}

/**
 * Update regulators according to current mode
 */
void regulators_update(void)
{
    abst_logf("Upd regulators: %i\n", (int)pid_mode);
    abst_logf("des_speed: %i\n", (int)des_speed);
    abst_logf("des_current: %i\n", (int)des_current);
    abst_logf("des_position: %i\n", (int)des_position);
    abst_logf("motor_v: %i\n", (int)motor_v);
    
    switch (pid_mode) {
        case SPEED_PID:
            speed_pid_update();
            break;
        case CURRENT_PID:
            current_pid_update();
            break;
        case POSITION_PID:
            position_pid_update();
            break;
        case DIRECT_CONT:
            motor_set_pwm(motor_v);
            break;
        default:
            motor_set_pwm(0);
    }
}

/**
 * Change pid settings according to the data given from CAN.
 * If N == 1 just save settings to a flash.
 * 
 * :param data: An array of information received
 * :param N: Length of the array
 */
void change_pid_settings(uint8_t data[], uint8_t N)
{
    abst_log("Change PID set\n");
    uint8_t pid_type = 0;
    
    if (N == 1) { // Save to flash
        save_settings_to_flash(data[0]);
    }
    
    if (N != sizeof(struct pid_settings_msg)) // Unknown data format
        return;
    
    struct pid_settings_msg *settings = (typeof(settings)) data;
    
    abst_logf("Type: %i, Field: %i, Value: %i\n", 
              (int)settings->pid_type, 
              (int)settings->pid_field,
              (int)settings->value);
    
    struct int_pid *pid;
    switch (settings->pid_type) {
        case SPEED_PID:
            pid = &PID_speed;
            break;
        case CURRENT_PID:
            pid = &PID_current;
            break;
        case POSITION_PID:
            pid = &PID_position;
            break;
        default:
            return; // Unknown type
    }
    
    switch (settings->pid_field) {
        case PID_FIELD_P:
            pid->P = settings->value;
            break;
        case PID_FIELD_I:
            pid->I = settings->value;
            break;
        case PID_FIELD_D:
            pid->D = settings->value;
            break;
        case PID_FIELD_DIV:
            pid->div = settings->value;
            break;
        case PID_FIELD_AVER_N:
            if (settings->pid_type == CURRENT_PID)
                set_current_n_aver(settings->value);
            break;
        default:
            return; // Unknown field
    }
    
    pid_init(pid);
}

/**
 * Set desired value of pid regulator and switch mode to this regulator.
 * Based on message from CAN.
 * 
 * :param data: An array of information received
 * :param N: Length of the array
 */
void set_desired_value(uint8_t data[], uint8_t N)
{
    abst_log("Change PID des\n");
    if (N != sizeof(struct pid_des_value_msg))
        return; // Unknown message format
    
    struct pid_des_value_msg *message = (typeof(message)) data;
    
    abst_logf("Type: %i, Value: %i\n", 
              (int)message->pid_type, 
              (int)message->value);
    
    switch (message->pid_type) {
        case SPEED_PID:
            des_speed = message->value;
            break;
        case CURRENT_PID:
            des_current = message->value;
            break;
        case POSITION_PID:
            des_position = message->value;
            break;
        case DIRECT_CONT:
            motor_v = message->value;
            break;
        default:
            return; // Unknown pid type
    }
    pid_mode = message->pid_type;
}

int32_t regulator_get_fd_speed(void)
{
    return fd_speed;
}

static void save_settings_to_flash(uint8_t pid_type)
{
    struct int_pid *pid = 0;
    switch (pid_type) {
        case SPEED_PID:
            pid = &PID_speed;
            break;
        case CURRENT_PID:
            pid = &PID_current;
            break;
        case POSITION_PID:
            pid = &PID_position;
            break;
        default:
            return;
    }
    
    struct pid_settings buff = {
        .P = (int32_t)pid->P,
        .I = (int32_t)pid->I,
        .D = (int32_t)pid->D,
        .div = (int32_t)pid->div,
    };
    
    if (pid_type == CURRENT_PID)
        buff.aver_N = get_current_n_aver();
    
    write_settings_flash(&buff, pid_type);
}

/*
 * Update speed regulator and set out voltage
 */
static void speed_pid_update(void)
{
    static int64_t enc_val_prev = 0;
    static uint32_t time_prev;
    
    int64_t enc_val = get_encoder_value();
    time = abst_time_ms();

    fd_speed = (enc_val - enc_val_prev) * 1000 / (time - time_prev);
    abst_logf("Update fd_speed: %i\n", (int)fd_speed);
    
    pid_update(&PID_speed);
    motor_set_pwm(motor_v);
    
    enc_val_prev = enc_val;
    time_prev = time;
}

/*
 * Update current regulator and set out voltage
 */
static void current_pid_update(void)
{
    fd_current = get_current_value();
    
    pid_update(&PID_current);

    motor_set_pwm(motor_v);
}

/*
 * Update position regulator and set out voltage
 */
static void position_pid_update(void)
{
    fd_position = get_encoder_value();
    time = abst_time_ms();

    pid_update(&PID_position);
    motor_set_pwm(motor_v);
}
