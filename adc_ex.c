#include <inttypes.h>
#include <stdio.h>

#include "max31865_adc.h"
#include "log.h"
#include <libopencm3/stm32/spi.h>



static void _output_adcex(unsigned adc, uint16_t temp)
{
    // Algorithm and magic numbers courtesy of MikroElektonica...
    /*
    coefficient = (referentResistance/400.0);

    inputData >>= 1;
    floatValue = (float)inputData * coefficient;
    floatValue /= 32;
    floatValue -= 256;
    */
    log_out("ADCEX: %u", adc + 1);
    log_out("RAW: %04X", temp);
    log_out("REAL: (((%u / 2) * (%u / 400.0)) / 32) - 256", temp, MAX31865_REF_RESISTANCE_470);
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


void adcs_ex_adc_log(unsigned adc)
{
    uint16_t temp;

    max31865_wait_for_data_ready(adc);
    temp = max31865_read_temperature(adc);
    _output_adcex(adc, temp);
}


void adcs_ex_log()
{
    for(unsigned n = 0; n < RTD_COUNT; n++)
        adcs_ex_adc_log(n);
}
