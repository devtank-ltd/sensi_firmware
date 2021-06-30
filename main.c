#include <stdint.h>
#include <inttypes.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include "pinmap.h"
#include "cmd.h"
#include "log.h"
#include "usb_uarts.h"
#include "uarts.h"
#include "adcs.h"
#include "adc_ex.h"
#include "pulsecount.h"
#include "timers.h"
#include "inputs.h"
#include "outputs.h"
#include "uart_rings.h"
#include "pwm.h"

volatile unsigned uptime = 0;

void hard_fault_handler(void)
{
    platform_raw_msg("----big fat libopen3 crash -----");
    while(true)
        uart_rings_out_drain();
}


void sys_tick_handler(void)
{
    gpio_toggle(LED_PORT, LED_PIN);
    uptime += 1;
}


int main(void) {
    rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_48MHZ]);
    uarts_setup();

    platform_raw_msg("----start----");
    log_out("Frequency : %lu", rcc_ahb_frequency);
    log_out("Version : %s", GIT_VERSION);

    log_init();
    cmds_init();
    usb_init();
    adcs_init();
//    adcs_ex_init();
    pwm_init();
    timers_init();
    inputs_init();
    outputs_init();

    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_clear(LED_PORT, LED_PIN);

    uptime = 0;

    systick_set_reload(rcc_ahb_frequency / 8 / 1000 * TICK_MS);
    STK_CVR = 0;
    systick_counter_enable();
    systick_interrupt_enable();

    log_out("Press 'D' for debug.");
    log_debug_mask = 0;
    log_async_log = true;

    while(true)
    {
        uart_rings_in_drain();
        uart_rings_out_drain();
    }

    return 0;
}
