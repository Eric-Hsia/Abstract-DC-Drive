/**
 * Absrtact DC Drive Project
 *
 * make PROFILE=release LOG=1 tidy all
 */

#include "flash.h"

#include <abstractSTM32.h>
#include <abstractADC.h>
#include <abstractLOG.h>
#include <abstractENCODER.h>

#include <int_PID.h>

#include <libopencm3/stm32/rcc.h>

// Place settings in FLASH at address (0x08000000 + 30 * 1024) (see linker script) 
// volatile const int32_t settings[256] __attribute__((optimize("O0"), used, section(".SETTINGS")));


struct abst_pin DC_OUT1 = {
    .port = ABST_GPIOA,
    .num = 12,
    .mode = ABST_MODE_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin DC_OUT2 = {
    .port = ABST_GPIOA,
    .num = 11,
    .mode = ABST_MODE_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin DC_EN = {
    .port = ABST_GPIOA,
    .num = 7,
    .mode = ABST_MODE_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin pot_ch = {
    .port = ABST_GPIOA,
    .num = 0,
    .mode = ABST_MODE_ANALOG,
    .adc_num = 1,
    .adc_channel = 0,
    .adc_sample_time = ABST_ADC_SMPR_SMP_55DOT5CYC,
    .otype = ABST_OTYPE_OD,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin current_ch = {
    .port = ABST_GPIOA,
    .num = 4,
    .mode = ABST_MODE_ANALOG,
    .adc_num = 1,
    .adc_channel = 4,
    .adc_sample_time = ABST_ADC_SMPR_SMP_55DOT5CYC,
    .otype = ABST_OTYPE_OD,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin_group ENC = {
    .port = ABST_GPIOA,
    .num = 1 << 8 | 1 << 9,
    .mode = ABST_MODE_AF,
    .af_dir = ABST_AF_INPUT,
    .otype = ABST_OTYPE_OD,
    .speed = ABST_OSPEED_50MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_encoder encoder;

volatile uint32_t des_speed = 0;
volatile uint32_t fd_speed = 0;
volatile uint32_t motor_v = 0;
volatile uint32_t time = 0;

struct int_pid PID = {
    .P = 10e6,
    .D = 0,
    .I = 1e4,
    .div = 1e6,
    .desired = &des_speed,
    .in = &fd_speed,
    .out = &motor_v,
    .time = &time
};

void motor_set_pwm(int16_t input);

int main(void)
{
    abst_init(8e6, 0);
    abst_log_init(9600);

    struct pid_settings speed_pid_settings = {
        .P = 10e6,
        .D = 123e5,
        .I = 1e4,
        .div = 1e6
    };
    
//     bool w_status = write_settings_flash(&speed_pid_settings, SPEED_PID);
//     if (!w_status) {
//         abst_log("Cannot write settings into FLASH\n");
//         return 1;
//     }
    
    speed_pid_settings.P = 0;
    speed_pid_settings.I = 0;
    speed_pid_settings.D = 0;
    speed_pid_settings.div = 0;
    
    if (!read_settings_flash(&speed_pid_settings, SPEED_PID)) {
        abst_log("Could not read the settings of Speed PID\n");
        return 1;
    }
    
    abst_logf("P: %i, I: %i, D: %i, div: %i\n", (int)speed_pid_settings.P, 
                                                (int)speed_pid_settings.I, 
                                                (int)speed_pid_settings.D,
                                                (int)speed_pid_settings.div);
    
    
    abst_gpio_init(&DC_OUT1);
    abst_gpio_init(&DC_OUT2);
    abst_gpio_init(&DC_EN);

    abst_group_gpio_init(&ENC);

    abst_encoder_init(&encoder,
                      1, // Timer
                      ABST_TIM_DIV_2); // Divider

    pid_init(&PID);

    struct abst_pin *pins_arr[] = {&pot_ch, &current_ch};
    uint8_t N = sizeof(pins_arr) / sizeof(pins_arr[0]);
    volatile uint16_t adc_vals[N];

    volatile uint16_t *input = adc_vals;
    volatile uint16_t *current = adc_vals + 1;

    enum abst_errors err = abst_adc_read_cont(pins_arr, // Array of pins
                                              adc_vals, // Array of values
                                              N, // Length of array
                                              2,  // Prescale
                                              1); // Prior of DMA requests

    int64_t enc_val = 0;
    int64_t enc_val_prev = 0;

    uint32_t time_prev = 0;

    uint32_t log = 0;

    abst_digital_write(&DC_EN, 0); // Run the motor

    while (1) {
        enc_val_prev = enc_val;
        enc_val = abst_encoder_read(&encoder);
        
        time_prev = time;
        time = abst_time_ms();

        fd_speed = (enc_val - enc_val_prev) * 1000 / 4 / (time - time_prev);

        des_speed = *input / 8 - 255;

        pid_update(&PID);

        motor_set_pwm(motor_v);

        if (log % 1 == 0) {
            log = 0;
            // abst_logf("Input: %i, Current: %i, ", (int)des_speed, (int)*current);
            // abst_logf("Enc: %i, Speed: %i, ", (int)enc_val, (int)fd_speed);
            // abst_logf("PWM: %i, ", (int)motor_v);
            // abst_logf("P: %i, I: %i, D: %i\n", (int)pid_dbg.P, (int)pid_dbg.I, (int)pid_dbg.D);
//             abst_logf("Input: %i, Speed: %i\n", (int)des_speed, (int)fd_speed);
        }
        log++;
        abst_delay_ms(50);
    }
}

void tim1_up_isr(void)
{
    abst_encoder_interrupt_handler(&encoder);
}

// Motor control using driver
void motor_set_pwm(int16_t input)
{
    const uint16_t max = 255;
    if (input > max)
        input = max;
    else if (input < -max)
        input = -max;

    uint8_t ch1;
    uint8_t ch2;

    if (input >= 0) {
        ch1 = input;
        ch2 = 0;
    } 
    else {
        ch1 = 0;
        ch2 = -input;
    }

    abst_pwm_soft(&DC_OUT1, ch1);
    abst_pwm_soft(&DC_OUT2, ch2);
}
