#include <inttypes.h>
#include <stdio.h>

#include "max31865_adc.h"
#include "log.h"
#include <libopencm3/stm32/spi.h>



static void _output_convertion(int16_t itemp)
{
    // Algorithm and magic numbers courtesy of MikroElektonica...
    /*
    coefficient = (referentResistance/400.0);

    inputData >>= 1;
    floatValue = (float)inputData * coefficient;
    floatValue /= 32;
    floatValue -= 256;
    */
    log_out("DTEMP: (((%i >> 1) * (%u / 400.0)) / 32) - 256", itemp, MAX31865_REF_RESISTANCE_470);
}


void adcs_ex_init()
{
    platform_raw_msg("adcs_ex_init: start");

    max31865_init();
    spi_enable(MAX31865_SPI);
    max31865_config();

    platform_raw_msg("adcs_ex_init: complete");

#if TEST_DIAGNOSTICS
    uint32_t i = 0;
    char buffer[32];
    int16_t itemp;

    for (i=0; i<100000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    snprintf(buffer, 32, "ITEMP: %04X", itemp);
    platform_raw_msg(buffer);

    _output_convertion(itemp);

    for (i=0; i<1000000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    snprintf(buffer, 32, "TEMP: %04X", itemp);
    platform_raw_msg(buffer);

    _output_convertion(itemp);

    for (i=0; i<1000000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    snprintf(buffer, 32, "TEMP: %04X", itemp);
    platform_raw_msg(buffer);

    _output_convertion(itemp);
#endif
}


unsigned adcs_ex_get_count()
{
    return 2;
}


void adcs_ex_adc_log(unsigned adc)
{
    adc = adc;

    int16_t itemp;

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_INTERNAL);
    log_out("TEMP: %04X", itemp);
    _output_convertion(itemp);
}


void adcs_ex_log()
{
    for(unsigned n = 0; n < 2; n++)
        adcs_ex_adc_log(n);
}
