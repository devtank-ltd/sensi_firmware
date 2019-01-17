#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include "adcs.h"
#include "pulsecount.h"


void tim3_isr(void)
{
    static unsigned count = 999;

    count++;

    if (count == 1000)
    {
        pulsecount_second_boardary();
        adcs_second_boardary();
        count = 0;
    }

    pulsecount_do_samples();
    adcs_do_samples();

    timer_clear_flag(TIM3, TIM_SR_CC1IF);
}


void timers_init()
{
    rcc_periph_clock_enable(RCC_TIM3);

    timer_disable_counter(TIM3);

    timer_set_mode(TIM3,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    //-1 because it starts at zero, and interrupts on the overflow
    timer_set_prescaler(TIM3, rcc_ahb_frequency / 1000000-1);
    timer_set_period(TIM3, 1000-1);

    timer_enable_preload(TIM3);
    timer_continuous_mode(TIM3);

    timer_enable_counter(TIM3);
    timer_enable_irq(TIM3, TIM_DIER_CC1IE);

    timer_set_counter(TIM3, 0);
    nvic_enable_irq(NVIC_TIM3_IRQ);
    nvic_set_priority(NVIC_TIM3_IRQ, 1);
}