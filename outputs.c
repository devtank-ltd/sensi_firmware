#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "config.h"
#include "pinmap.h"
#include "inputs.h"
#include "log.h"


static const port_n_pins_t    outputs[] = OUTPUTS_PORT_N_PINS;

void     outputs_init()
{
    for(unsigned n = 0; n < ARRAY_SIZE(outputs); n++)
    {
        const port_n_pins_t * output = &outputs[n];
        rcc_periph_clock_enable(PORT_TO_RCC(output->port));
        gpio_mode_setup(output->port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, output->pins);
    }
}


void     outputs_set(unsigned index, bool on_off)
{
    if (index >= ARRAY_SIZE(outputs))
        return;

    const port_n_pins_t * output = &outputs[index];
    if (on_off)
        gpio_set(output->port, output->pins);
    else
        gpio_clear(output->port, output->pins);
}


void     outputs_log()
{
    for(unsigned n = 0; n < ARRAY_SIZE(outputs); n++)
    {
        const port_n_pins_t * output = &outputs[n];
        uint16_t on_off = gpio_get(output->port, output->pins);
        log_out("Output %02u : %s", n, (on_off)?"on":"off");
    }
}
