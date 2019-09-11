#include <inttypes.h>
#include <stdio.h>

#include "max31865_adc.h"
#include "log.h"
#include <libopencm3/stm32/spi.h>



static void _output_adcex(unsigned adc, int16_t itemp)
{
    // Algorithm and magic numbers courtesy of MikroElektonica...
    /*
    coefficient = (referentResistance/400.0);

    inputData >>= 1;
    floatValue = (float)inputData * coefficient;
    floatValue /= 32;
    floatValue -= 256;
    */
    log_out("ADCEX: %u", adc);
    log_out("RAW: %04X", itemp);
    log_out("REAL: (((%i / 2) * (%u / 400.0)) / 32) - 256", itemp, MAX31865_REF_RESISTANCE_470);
}


void adcs_ex_init()
{
    max31865_init();
    spi_enable(MAX31865_SPI);
    max31865_config();

#if TEST_DIAGNOSTICS
    uint32_t i = 0;
    int16_t itemp;

    for (i=0; i<100000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    _output_adcex(0, itemp);

    for (i=0; i<1000000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    _output_adcex(0, itemp);

    for (i=0; i<1000000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    _output_adcex(0, itemp);
#endif
}


unsigned adcs_ex_get_count()
{
    return 2;
}


void adcs_ex_adc_log(unsigned adc)
{
    int16_t itemp;

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_INTERNAL);
    _output_adcex(adc, itemp);
}


void adcs_ex_log()
{
    for(unsigned n = 0; n < 2; n++)
        adcs_ex_adc_log(n);
}
