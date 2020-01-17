#include <inttypes.h>
#include <stdio.h>
#include <inttypes.h>

#include "max31865_adc.h"
#include "log.h"
#include <libopencm3/stm32/spi.h>



static void _output_adcex(unsigned adc, uint16_t temp)
{
    log_out("ADCEX: %u", adc + 1);
    log_out("RAW: %"PRIu16, temp);
}


void adcs_ex_init()
{
    max31865_init();
    spi_enable(MAX31865_SPI);
    for(unsigned n = 0; n < RTD_COUNT; n++)
        max31865_config(n);
}


unsigned adcs_ex_get_count()
{
    return RTD_COUNT;
}


uint16_t adc_ex_get(unsigned adc)
{
    if (adc >= RTD_COUNT)
        return 0;
    max31865_wait_for_data_ready(adc);
    return max31865_read_temperature(adc);
}


void adcs_ex_adc_log(unsigned adc)
{
    _output_adcex(adc, adc_ex_get(adc));
}


void adcs_ex_log()
{
    for(unsigned n = 0; n < RTD_COUNT; n++)
        adcs_ex_adc_log(n);
}
