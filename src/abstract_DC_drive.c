/**
 * Absrtact DC Drive Project
 *
 * make PROFILE=release LOG=1 tidy all
 */

#include "flash.h"
#include "regulators.h"
#include "measurements.h"
#include "motor.h"
#include "can.h"

#include <abstractSTM32.h>
#include <abstractLOG.h>
#include <libopencm3/stm32/rcc.h>

void log_usart(void);

void lib_init(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    abst_init(72e6, 700);
    abst_log_init(9600);
    
    regulators_init();
    motor_init();
    measurements_init();
    can_bus_init();
}


int main(void)
{
    lib_init();
    
    while (1) {
        update_measurements();
        regulators_update();
        log_usart();
        abst_delay_ms(2);
    }
}

void log_usart(void)
{
//     static uint32_t log = 0;
//     
//     if (log % 50 == 0) {
//         log = 0;
//         switch (pid_mode) {
//             case SPEED_PID:
//                 abst_logf("Des_speed: %i, Speed: %i,  Motor_V: %i\n", (int)des_speed, (int)fd_speed, (int)motor_v);
//                 break;
//             case CURRENT_PID:
//                 abst_logf("Des_current: %i, Current: %i, Motor_V: %i\n", (int)des_current, (int)fd_current, (int)motor_v);
//                 break;
//             case POSITION_PID:
//                 abst_logf("Des_position: %ld, Position: %ld, Motor_V: %ld\n", (long)des_position, (long)fd_position, (long)(motor_v << 16) >> 16);
//                 break;
//         }
//     }
//     log++;
}
