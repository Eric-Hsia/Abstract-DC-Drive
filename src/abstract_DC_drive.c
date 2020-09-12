/**
 * Absrtact DC Drive Project
 *
 * make PROFILE=release LOG=1 tidy all
 */

#include "abstractSTM32.h"
#include "abstractADC.h"
#include "abstractLOG.h"

struct abst_pin led = {
    .port = ABST_GPIOA,
    .num = 8,
    .mode = ABST_MODE_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin DC_IN1 = {
    .port = ABST_GPIOA,
    .num = 11,
    .mode = ABST_MODE_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin DC_IN2 = {
    .port = ABST_GPIOA,
    .num = 12,
    .mode = ABST_MODE_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin pot = {
    .port = ABST_GPIOA,
    .num = 0,
    .mode = ABST_MODE_ANALOG,
    .adc_num = 1,
    .adc_channel = 0,
    .adc_sample_time = ABST_ADC_SMPR_SMP_55DOT5CYC,
    .otype = ABST_OTYPE_OD,
    .speed = ABST_OSPEED_50MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

int main(void)
{
    abst_init(16e6, 0);
    abst_log_init();

    abst_gpio_init(&led);
    abst_gpio_init(&DC_IN1);
    abst_gpio_init(&DC_IN2);

    volatile uint16_t adc_vals[] = {0};
    struct abst_pin *pins_arr[] = {&pot};
    volatile uint16_t *input = adc_vals;

    enum abst_errors err = abst_adc_read_cont(pins_arr, // Array of pins
                                              adc_vals, // Array of values
                                              sizeof(adc_vals) / sizeof(adc_vals[0]), // Length of array
                                              2,  // Prescale
                                              1); // Prior

    while (1) {
        abst_pwm_soft(&DC_IN1, *input / 16);
        abst_pwm_soft(&DC_IN2, 0);

        abst_log(*input / 16);

        abst_delay_ms(20);
    }
}