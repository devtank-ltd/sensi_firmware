
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

#include "config.h"
#include "pinmap.h"
#include "log.h"
#include "pwm.h"

static unsigned pwm_freq = 100;
static unsigned pwm_duty = 0;


#define CPU_MHZ (rcc_ahb_frequency / 1000000)

void pwm_init()
{
    rcc_periph_clock_enable(RCC_PWM_TIMER);

    timer_disable_counter(PWM_TIMER);

    timer_set_mode(PWM_TIMER,
        TIM_CR1_CKD_CK_INT,
        TIM_CR1_CMS_EDGE,
        TIM_CR1_DIR_UP);

    timer_set_prescaler(PWM_TIMER, CPU_MHZ);

    timer_enable_preload(PWM_TIMER);
    timer_continuous_mode(PWM_TIMER);

    timer_set_period(PWM_TIMER, 1000000);

    timer_disable_oc_output(PWM_TIMER, PWM_TIMER_CH);
    timer_set_oc_mode(PWM_TIMER, PWM_TIMER_CH, TIM_OCM_PWM1);
    timer_enable_oc_output(PWM_TIMER, PWM_TIMER_CH);

    timer_enable_counter(PWM_TIMER);

    timer_set_oc_value(PWM_TIMER, PWM_TIMER_CH, 0);
    
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
    timer_set_period(PWM_TIMER, 1000000/freq);

    timer_set_oc_value(PWM_TIMER, PWM_TIMER_CH, 1000000/freq * duty / PWM_MAX);

    timer_set_counter(PWM_TIMER, 0);

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
