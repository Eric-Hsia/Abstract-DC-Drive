#include "measurements.h"

#include <abstractENCODER.h>
#include <abstractADC.h>

#include <stdlib.h>

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

struct abst_pin temp_ch = {
    .port = ABST_GPIOA,
    .num = 3,
    .mode = ABST_MODE_ANALOG,
    .adc_num = 1,
    .adc_channel = 3,
    .adc_sample_time = ABST_ADC_SMPR_SMP_55DOT5CYC,
    .otype = ABST_OTYPE_OD,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

static uint16_t current_N = 1;
static uint16_t *current_arr;

static uint16_t temp_N = 1;
static uint16_t *temp_arr;

static volatile uint16_t *adc_vals;
static volatile uint16_t *input;
static volatile uint16_t *current;
static volatile uint16_t *temp;

static uint16_t avrg(uint16_t array[], uint16_t N);

/**
 * Initialize ADC and ENCODER
 */
void measurements_init(void)
{
    // ENCODER
    abst_group_gpio_init(&ENC);
    
    abst_encoder_init(&encoder,
                      1, // Timer
                      ABST_TIM_DIV_2); // Divider
    
    // ADC
    struct abst_pin *pins_arr[] = {&current_ch, &temp_ch ,&pot_ch};
    uint8_t N = sizeof(pins_arr) / sizeof(pins_arr[0]);
    adc_vals = malloc(sizeof(adc_vals[0]) * N);
    
    current = adc_vals;
    temp = adc_vals + 1;
    input = adc_vals + 2;
    
    abst_adc_read_cont( pins_arr, // Array of pins
                        adc_vals, // Array of values
                        N, // Length of array
                        8,  // Prescale
                        1); // Prior of DMA requests
    
    
    current_arr = malloc(sizeof(current_arr[0]) * current_N);
    for (uint16_t i = 0; i < current_N; i++)
        current_arr[i] = 0;
    
    temp_arr = malloc(sizeof(temp_arr[0]) * temp_N);
    for (uint16_t i = 0; i < temp_N; i++)
        temp_arr[i] = 0;
}

void update_measurements(void)
{
    static uint16_t current_i = 0;
    current_arr[current_i++] = (*current) >> 2;
    if (current_i % (current_N + 1) == 0)
        current_i = 0;
    
    static uint16_t temp_i = 0;
    temp_arr[temp_i++] = (*temp) >> 2;
    if (temp_i % (temp_N + 1) == 0)
        temp_i = 0;
}

int64_t get_encoder_value(void)
{
    abst_encoder_read(&encoder);
}

int64_t get_current_value(void)
{
    avrg(current_arr, current_N);
}

void tim1_up_isr(void)
{
    abst_encoder_interrupt_handler(&encoder);
}

void set_current_n_aver(uint32_t N)
{
    current_N = N;
}

uint32_t get_current_n_aver(void)
{
    return current_N;
}

// Average of an array
static uint16_t avrg(uint16_t array[], uint16_t N)
{
    uint32_t summ = 0;
    for (uint16_t i = 0; i < N; i++)
        summ += array[i];
    return summ / N;
}
