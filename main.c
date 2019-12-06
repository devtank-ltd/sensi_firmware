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


void hard_fault_handler(void)
{
    platform_raw_msg("----big fat libopen3 crash -----");
    while(true);
}


void sys_tick_handler(void)
{
	gpio_toggle(LED_PORT, LED_PIN);
}


int main(void) {
    rcc_clock_setup_in_hsi48_out_48mhz();
    uarts_setup();

    platform_raw_msg("----start----");
    log_out("Frequency : %lu", rcc_ahb_frequency);
    log_out("Version : %s", GIT_VERSION);

    log_init();
    cmds_init();
    usb_init();
    adcs_init();
    adcs_ex_init();
    pwm_init();
    timers_init();
    inputs_init();
    outputs_init();

    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_clear(LED_PORT, LED_PIN);

    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    systick_set_reload(rcc_ahb_frequency / 8 / 1000 * TICK_MS);
    STK_CVR = 0;
    systick_counter_enable();
    systick_interrupt_enable();

    log_out("Press 'D' for debug.");
    log_async_log = true;

    while(true)
    {
        uart_rings_in_drain();
        uart_rings_out_drain();
    }

    return 0;
}
