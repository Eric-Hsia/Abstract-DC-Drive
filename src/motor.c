#include "motor.h"
#include "abstractSTM32.h"

struct abst_pin DC_OUT1 = {
    .port = ABST_GPIOA,
    .num = 5,
    .mode = ABST_MODE_OUTPUT,
    .otype = ABST_OTYPE_PP,
    .speed = ABST_OSPEED_2MHZ,
    .pull_up_down = ABST_PUPD_NONE,
    .is_inverse = false
};

struct abst_pin DC_OUT2 = {
    .port = ABST_GPIOA,
    .num = 6,
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

/*
 * Setup PWM according to input
 */
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

/** 
 * Initialize the pins for motor
 */
void motor_init(void)
{
    abst_gpio_init(&DC_OUT1);
    abst_gpio_init(&DC_OUT2);
    abst_gpio_init(&DC_EN); 
    
    motor_set_pwm(0);
    abst_digital_write(&DC_EN, 0);
}
