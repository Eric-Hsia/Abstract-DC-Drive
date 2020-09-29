/**
 * Absrtact DC Drive Project
 *
 * make PROFILE=release LOG=1 tidy all
 * make PROFILE=debug LOG=1 tidy all
 */
#define ADC_NO_CALIBR

#include "flash.h"
#include "regulators.h"
#include "measurements.h"
#include "motor.h"
#include "can.h"

#include <abstractSTM32.h>
#include <abstractLOG.h>
#include <libopencm3/stm32/rcc.h>
#include <abstractCAN.h>

#include "libopencm3/cm3/nvic.h"
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/can.h>


void log_usart(void);

void lib_init(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV16);
    rcc_apb1_frequency = 72e6 / 16;
    abst_init(72e6, 100);
    abst_log_init(9600);
    
    abst_log("Log i\n");
    regulators_init();
    abst_log("Regulators i\n");
    motor_init();
    abst_log("Motor i\n");
    measurements_init();
    abst_log("Mesurments i\n");
    while(!can_bus_init());
    abst_log("CAN i\n");
}

struct abst_pin can_RX_main = {
    .port = ABST_GPIOA,
    .num = 11,
    .mode = ABST_MODE_AF,
    .af_dir = ABST_AF_INPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_50MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

int main(void)
{
    lib_init();
    abst_log("Start program\n");
    abst_delay_ms(1e3);
    while (1) {
        update_measurements();
        regulators_update();
        log_usart();
        abst_log("--------------"); // Delimeter
        abst_delay_ms(200);
    }
}

void log_usart(void)
{
    static uint32_t log = 0;
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
