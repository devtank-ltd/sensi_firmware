
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

#include "config.h"
#include "pinmap.h"
#include "pwm.h"

static unsigned pwm_freq = 1000;
static unsigned pwm_duty = 0;


void pwm_init()
{
    rcc_periph_clock_enable(RCC_TIM1);

    timer_disable_counter(RCC_TIM1);

    timer_set_mode(RCC_TIM1,
        TIM_CR1_CKD_CK_INT,
        TIM_CR1_CMS_EDGE,
        TIM_CR1_DIR_UP);
    timer_set_prescaler(RCC_TIM1, 48);

    timer_enable_preload(RCC_TIM1);
    timer_continuous_mode(RCC_TIM1);

    timer_set_period(RCC_TIM1,48000000/48);

    timer_disable_oc_output(RCC_TIM1, TIM_OC1);
    timer_set_oc_mode(RCC_TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(RCC_TIM1, TIM_OC1);

    timer_enable_counter(RCC_TIM1);

    timer_set_oc_value(RCC_TIM1, TIM_OC1, 0);
    
    const port_n_pins_t port_n_pins[] = PWM_PORT_N_PINS;

    for(unsigned n = 0; n < ARRAY_SIZE(port_n_pins); n++)
    {
        uint32_t gpioport = port_n_pins[n].port;
        uint32_t gpios = port_n_pins[n].pins;

        gpio_mode_setup( gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpios);
        gpio_set_af( gpioport, ((uint32_t[])PWM_GPIO_FUNC)[n], gpios );
        gpio_set_output_options( gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, gpios );
    }

    pwm_set(pwm_freq, pwm_duty);
}

void pwm_set(unsigned freq, unsigned duty)
{
    timer_set_period(TIM14, 48000000/48/freq);

    duty = 100 - duty;

    timer_set_oc_value(TIM14, TIM_OC1, 48000000/48/freq * duty / 2 / 100);

    timer_set_counter(TIM14, 0);

    pwm_freq = freq;
    pwm_duty = duty;
}

void pwm_get(unsigned *freq, unsigned *duty)
{
    if (freq)
        *freq = pwm_freq;
    if (duty)
        *duty = pwm_duty;
}
