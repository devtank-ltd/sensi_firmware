#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "config.h"
#include "adcs.h"
#include "adc_ex.h"
#include "pulsecount.h"
#include "inputs.h"
#include "timers.h"
#include "uart_rings.h"

static volatile unsigned fast_rate_sps     = DEFAULT_SPS;
static volatile unsigned fast_sample_count = DEFAULT_SPS-1;


void tim2_isr(void)
{
    fast_sample_count++;

    if (fast_sample_count == DEFAULT_SPS)
    {
        adcs_second_boardary();
        fast_sample_count = 0;
    }

    adcs_do_samples();
    timer_clear_flag(TIM2, TIM_SR_CC1IF);
}


void     timers_init()
{
    rcc_periph_clock_enable(RCC_TIM2);

    timer_disable_counter(TIM2);

    timer_set_mode(TIM2,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    //-1 because it starts at zero, and interrupts on the overflow
    timer_set_prescaler(TIM2, rcc_ahb_frequency / 1000000-1);
    timer_set_period(TIM2, 1000 * 1000 / DEFAULT_SPS - 1);
    timer_enable_preload(TIM2);
    timer_continuous_mode(TIM2);

    timer_enable_counter(TIM2);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    timer_set_counter(TIM2, 0);
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, TIMER2_PRIORITY);
}
