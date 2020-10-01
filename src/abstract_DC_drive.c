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
#include <abstractCAN.h>

#include <libopencm3/stm32/rcc.h>

void log_usart(void);

/* Initializing all modules */
void lib_init(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_DIV16);
    rcc_apb1_frequency = 72e6 / 16;
    abst_init(72e6, 100);
    abst_log_init(115200, 500);
    
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

int main(void)
{
    lib_init();
    abst_log("Start program\n");
    while (1) {
        update_measurements();
        regulators_update();
        abst_delay_ms(20);
    }
}
