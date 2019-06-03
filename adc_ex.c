#include <inttypes.h>

#include "ads1248.h"
#include "log.h"


typedef struct
{
    long int max_value;
    long int min_value;
    int64_t  total_value;
    unsigned count;
} adc_channel_info_t;

static volatile adc_channel_info_t adc_channel_info[1] = {{0}};

static volatile adc_channel_info_t adc_channel_info_cur[1] = {{0}};


static unsigned channel_regs[1][4] = {{ 0x01, 0x00, 0x20, 0x22}};
static unsigned call_count = 0;


void     adcs_ex_init()
{
    InitSPI();
    InitDevice();

    ADS1248SetStart(1);

    ADS1248SendResetCommand();
    ADS1248SendSDATAC();
    ADS1248WaitForDataReady(0);

    unsigned regs[2] = {0};
    ADS1248ReadRegister(ADS1248_10_IDAC0, 2, regs);

    regs[0] = (regs[0] & 0xE0) | 0x6; // 1000 μA (1mA)
    regs[1] = 0x89; // Use IEXC1 and IEXC2

    ADS1248WriteRegister(ADS1248_10_IDAC0, 2, regs);

}


void     adcs_ex_do_samples()
{
    call_count++;

    if (!(call_count % 2))
        return;

    for(unsigned n; n < ARRAY_SIZE(channel_regs); n++)
    {
        ADS1248SendSDATAC();
        ADS1248WaitForDataReady(0);

        ADS1248WriteSequence(ADS1248_0_MUX0, 4, channel_regs[n]);

        ADS1248SendSync();
        ADS1248WaitForDataReady(0);

        long int adc = ADS1248ReadData();

        volatile adc_channel_info_t * channel_info = &adc_channel_info[n];

        if (adc > channel_info->max_value)
            channel_info->max_value = adc;

        if (adc < channel_info->min_value)
            channel_info->min_value = adc;

        channel_info->total_value += adc;
        channel_info->count ++;
    }
}


void adcs_ex_second_boardary()
{
    unsigned sample_count = call_count / 2 / ARRAY_SIZE(channel_regs);

    log_debug(DEBUG_ADC, "ADCEX SPS %u", sample_count);

    for(unsigned n = 0; n < ARRAY_SIZE(channel_regs); n++)
    {
        volatile adc_channel_info_t * channel_info = &adc_channel_info[n];

        adc_channel_info_cur[n] = *channel_info;

        channel_info->max_value = 0;
        channel_info->min_value = (long int)-1;
        channel_info->total_value  = 0;
        channel_info->count = 0;

        if (adc_channel_info_cur[n].count != sample_count)
            log_debug(DEBUG_ADC, "ADCEX %u %u != %u", n, adc_channel_info_cur[n].count, sample_count);
    }

    call_count = 0;
}


unsigned adcs_ex_get_count()
{
    return ARRAY_SIZE(channel_regs);
}


void adcs_ex_adc_log(unsigned adc)
{
    if (adc >= ARRAY_SIZE(channel_regs))
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
