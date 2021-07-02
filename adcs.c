#include <inttypes.h>

/* On some versions of gcc this header isn't defining it. Quick fix. */
#ifndef PRIu64
#define PRIu64 "llu"
#endif

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rcc.h>

#include "log.h"
#include "adcs.h"
#include "pinmap.h"

typedef struct
{
    uint8_t adc;
    uint8_t channel;
} adc_channel_pair_t;

static const adc_channel_pair_t adc_channels[] = ADCS_CHANNEL;

#define ADC_CHIP(_index_) ((uint32_t[]){0, ADC1, ADC2})[_index_];

typedef struct
{
    unsigned max_value;
    unsigned min_value;
    uint64_t total_value;
    unsigned count;
} adc_channel_info_t;


const port_n_pins_t port_n_pins[] = ADCS_PORT_N_PINS;

static volatile adc_channel_info_t adc_channel_info[ARRAY_SIZE(port_n_pins)] = {{0}};

static volatile adc_channel_info_t adc_channel_info_cur[ARRAY_SIZE(port_n_pins)] = {{0}};

static volatile uint16_t last_value[ARRAY_SIZE(port_n_pins)] = {0};

static unsigned adc_index = 0;
static unsigned loop_count = 0;


static void adc_init(int adc)
{
    adc_calibrate(adc);
    adc_set_single_conversion_mode(adc);
    adc_disable_external_trigger_regular(adc);
    adc_set_right_aligned(adc);
    adc_set_sample_time_on_all_channels(adc, ADC_SMPR_SMP_4DOT5CYC);
    adc_set_resolution(adc, ADC_CFGR1_RES_12_BIT);
    adc_power_on(adc);
}


void adcs_init()
{
    for(unsigned n = 0; n < ARRAY_SIZE(port_n_pins); n++)
    {
        rcc_periph_clock_enable(PORT_TO_RCC(port_n_pins[n].port));
        gpio_mode_setup(port_n_pins[n].port,
                        GPIO_MODE_ANALOG,
                        GPIO_PUPD_NONE,
                        port_n_pins[n].pins);
    }

    rcc_periph_clock_enable(RCC_ADC12);

    adc_power_off(ADC1);
    adc_power_off(ADC2);

    adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2);
    adc_init(ADC1);
    adc_init(ADC2);
}


void adcs_do_samples()
{
    const adc_channel_pair_t * pair = &adc_channels[adc_index];

    uint32_t adc_chip = ADC_CHIP(pair->adc);

    if (adc_eoc(adc_chip))
    {
        uint32_t adc = adc_read_regular(adc_chip);

        volatile adc_channel_info_t * channel_info = &adc_channel_info[adc_index];

        if (adc > channel_info->max_value)
            channel_info->max_value = adc;

        if (adc < channel_info->min_value)
            channel_info->min_value = adc;

        channel_info->total_value += adc;
        channel_info->count ++;
        last_value[adc_index]   = adc;

        /* Start next ADC sample */
        adc_index++;
        if (adc_index == ARRAY_SIZE(adc_channels))
        {
            adc_index = 0;
            loop_count++;
        }
        pair = &adc_channels[adc_index];
        adc_chip = ADC_CHIP(pair->adc);
    }
    else log_debug(DEBUG_ADC, "ADC:%u ch:%u sampling not done", pair->adc, pair->channel);
    adc_set_regular_sequence(adc_chip, 1, (uint8_t*)&pair->channel);
    adc_start_conversion_regular(adc_chip);
}


void adcs_second_boardary()
{
    unsigned sample_count = loop_count;

    log_debug(DEBUG_ADC, "ADCS SPS %u", loop_count);

    for(unsigned n = 0; n < ARRAY_SIZE(port_n_pins); n++)
    {
        volatile adc_channel_info_t * channel_info = &adc_channel_info[n];

        adc_channel_info_cur[n] = *channel_info;

        channel_info->max_value = 0;
        channel_info->min_value = 0xFFFFFFFF;
        channel_info->total_value  = 0;
        channel_info->count = 0;

        if (adc_channel_info_cur[n].count != sample_count)
            log_debug(DEBUG_ADC, "ADC %u %u != %u", n, adc_channel_info_cur[n].count, sample_count);
    }

    loop_count = 0;
}


unsigned adcs_get_count()
{
    return ARRAY_SIZE(port_n_pins);
}


unsigned  adcs_get_last(unsigned adc)
{
    if (adc >= ARRAY_SIZE(port_n_pins))
        return 0;

    return last_value[adc];
}


unsigned  adcs_get_tick(unsigned adc)
{
    if (adc >= ARRAY_SIZE(port_n_pins))
        return 0;

    return adc_channel_info[adc].count;
}


void adcs_adc_log(unsigned adc)
{
    if (adc >= ARRAY_SIZE(port_n_pins))
        return;

    volatile adc_channel_info_t * channel_info = &adc_channel_info_cur[adc];
    const adc_channel_pair_t * pair = &adc_channels[adc];

    log_out("ADC : %u (ADC:%u Ch:%u)", adc + 1, pair->adc, pair->channel);
    log_out("Min : %u", channel_info->min_value);
    log_out("Max : %u", channel_info->max_value);
    log_out("Avg : %"PRIu64" / %u", channel_info->total_value, channel_info->count);
}


void adcs_log()
{
    for(unsigned n = 0; n < ARRAY_SIZE(adc_channel_info_cur); n++)
        adcs_adc_log(n);
}
