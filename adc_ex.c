#include <inttypes.h>
#include <stdio.h>

#include "max31865_adc.h"
#include "log.h"
#include <libopencm3/stm32/spi.h>

typedef struct
{
    long int max_value;
    long int min_value;
    int64_t  total_value;
    unsigned count;
} adc_channel_info_t;

static volatile adc_channel_info_t adc_channel_info[2] = {{0}};

static volatile adc_channel_info_t adc_channel_info_cur[2] = {{0}};


static unsigned channel_regs[1][4] = {{ 0x01, 0x00, 0x20, 0x22}};
static unsigned call_count = 0;

void adcs_ex_do_samples();

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
    double dtemp;

    for (i=0; i<100000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    snprintf(buffer, 32, "ITEMP: %04X", itemp);
    platform_raw_msg(buffer);

    dtemp = max31865_convert_temperature(itemp, MAX31865_REF_RESISTANCE_470);
    snprintf(buffer, 32, "DTEMP: %5.2f", dtemp);
    platform_raw_msg(buffer);

    for (i=0; i<1000000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    snprintf(buffer, 32, "TEMP: %04X", itemp);
    platform_raw_msg(buffer);

    dtemp = max31865_convert_temperature(itemp, MAX31865_REF_RESISTANCE_470);
    snprintf(buffer, 32, "DTEMP: %5.2f", dtemp);
    platform_raw_msg(buffer);

    for (i=0; i<1000000; i++) { asm("nop"); }

    max31865_wait_for_data_ready();
    itemp = max31865_read_temperature(MAX31865_RTD_EXTERNAL);
    snprintf(buffer, 32, "TEMP: %04X", itemp);
    platform_raw_msg(buffer);

    dtemp = max31865_convert_temperature(itemp, MAX31865_REF_RESISTANCE_470);
    snprintf(buffer, 32, "DTEMP: %5.2f", dtemp);
    platform_raw_msg(buffer);
#endif
}

static void _adcs_ex_do_samples_cs(unsigned adc_offset)
{
platform_raw_msg("_adcs_ex_do_samples_cs");
for(unsigned n = 0; n < ARRAY_SIZE(channel_regs); n++)
    {
        max31865_start_reading(adc_offset);
        max31865_wait_for_data_ready();
        int16_t raw_value = max31865_read_temperature(adc_offset);
        double adc = max31865_convert_temperature(raw_value, MAX31865_REF_RESISTANCE_470);

        volatile adc_channel_info_t* channel_info = &adc_channel_info[adc_offset + n];

        if (adc > channel_info->max_value)
            channel_info->max_value = adc;

        if (adc < channel_info->min_value)
            channel_info->min_value = adc;

        channel_info->total_value += adc;
        channel_info->count ++;
    }
platform_raw_msg("_adcs_ex_do_samples_cs - complete");
}

void adcs_ex_do_samples()
{
    call_count++;

    if (!(call_count % 2))
        return;

    _adcs_ex_do_samples_cs(MAX31865_RTD_INTERNAL);
    // TODO: sample for chip 1
}


void adcs_ex_second_boardary()
{
    unsigned sample_count = call_count / 2 / ARRAY_SIZE(adc_channel_info);

    log_debug(DEBUG_ADC_EX, "ADCEX SPS %u", sample_count);

    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_info); n++)
    {
        volatile adc_channel_info_t * channel_info = &adc_channel_info[n];

        adc_channel_info_cur[n] = *channel_info;

        channel_info->max_value = 0;
        channel_info->min_value = (long int)-1;
        channel_info->total_value  = 0;
        channel_info->count = 0;

        if (adc_channel_info_cur[n].count != sample_count)
            log_debug(DEBUG_ADC_EX, "ADCEX %u %u != %u", n, adc_channel_info_cur[n].count, sample_count);
    }

    call_count = 0;
}


unsigned adcs_ex_get_count()
{
    return ARRAY_SIZE(channel_regs);
}


void adcs_ex_adc_log(unsigned adc)
{
    if (adc >= ARRAY_SIZE(adc_channel_info))
        return;

    volatile adc_channel_info_t * channel_info = &adc_channel_info_cur[adc];

    log_out("ADCEX : %u", adc);
    log_out("Min : %li", channel_info->min_value);
    log_out("Max : %li", channel_info->max_value);
    log_out("Avg : %"PRIi64" / %u", channel_info->total_value, channel_info->count);
}


void adcs_ex_log()
{
    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_info_cur); n++)
        adcs_ex_adc_log(n);
}

uint64_t adcs_ex_read_value(int32_t adc_offset)
{
    platform_raw_msg("adcs_ex_read_value");

    max31865_start_reading(adc_offset);
    int16_t raw_value = max31865_read_temperature(adc_offset);
    max31865_wait_for_data_ready();
    double adc_val = max31865_convert_temperature(raw_value,
                                                  MAX31865_REF_RESISTANCE_470);

    platform_raw_msg("_adcs_ex_read_value - complete");

    log_out("ADCEX : %ld", adc_offset);
        log_out("Val   : %5.2f", adc_val);

return adc_val;
}

