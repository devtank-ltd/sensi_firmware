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
static volatile unsigned adc_ex_sample_count = DEFAULT_ADC_EX_SPS-1;


void tim3_isr(void)
{
    pulsecount_second_boardary();

    timer_clear_flag(TIM3, TIM_SR_CC1IF);
}


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


void tim7_isr(void)
{
    adc_ex_sample_count++;

    if (adc_ex_sample_count == DEFAULT_ADC_EX_SPS)
    {
        adcs_ex_second_boardary();
        adc_ex_sample_count = 0;
    }

    adcs_ex_do_samples();
    timer_clear_flag(TIM7, TIM_SR_CC1IF);
}



void     timers_init()
{
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM7);

    timer_disable_counter(TIM3);

    timer_set_mode(TIM3,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    //-1 because it starts at zero, and interrupts on the overflow
    timer_set_prescaler(TIM3, rcc_ahb_frequency / 1000-1);
    timer_set_period(TIM3, 1000-1);

    timer_enable_preload(TIM3);
    timer_continuous_mode(TIM3);

    timer_enable_counter(TIM3);
    timer_enable_irq(TIM3, TIM_DIER_CC1IE);

    timer_set_counter(TIM3, 0);
    nvic_enable_irq(NVIC_TIM3_IRQ);
    nvic_set_priority(NVIC_TIM3_IRQ, TIMER1_PRIORITY);



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

/*
    timer_disable_counter(TIM7);

    timer_set_mode(TIM7,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);
    //-1 because it starts at zero, and interrupts on the overflow
    timer_set_prescaler(TIM7, rcc_ahb_frequency / 1000000-1);
    timer_set_period(TIM7, 1000 * 1000 / DEFAULT_ADC_EX_SPS - 1);
    timer_enable_preload(TIM7);
    timer_continuous_mode(TIM7);

    timer_enable_counter(TIM7);
    timer_enable_irq(TIM7, TIM_DIER_CC1IE);

    timer_set_counter(TIM7, 0);
    nvic_enable_irq(NVIC_TIM7_IRQ);
    nvic_set_priority(NVIC_TIM7_IRQ, TIMER3_PRIORITY);
*/
}
