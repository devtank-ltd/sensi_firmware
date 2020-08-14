#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "pinmap.h"
#include "log.h"
#include "cmd.h"
#include "uarts.h"
#include "uart_rings.h"
#include "usb_uarts.h"
#include "adcs.h"
#include "adc_ex.h"
#include "inputs.h"
#include "outputs.h"
#include "timers.h"
#include "pwm.h"

static char   * rx_buffer;
static unsigned rx_buffer_len = 0;
static unsigned rx_pos        = 0;


typedef struct
{
    const char * key;
    const char * desc;
    void (*cb)(void);
} cmd_t;


static void adc_cb();
static void adcex_cb();
static void adcs_ex_loop();
static void input_cb();
static void output_cb();
static void count_cb();
static void all_cb();
static void pwm_cb();
static void version_cb();


static cmd_t cmds[] = {
    { "adc",      "Print ADC.",              adc_cb},
    { "adcs",     "Print all ADCs.",         adcs_log},
    { "adcex",    "Print ADC EX.",           adcex_cb},
    { "adcexs",   "Print all ADC EX.",       adcs_ex_log},
    { "loop_ex",  "Loop printing ADC EX.",   adcs_ex_loop},
    { "inputs",   "Print all inputs.",       inputs_log},
    { "outputs",  "Print all outputs.",      outputs_log},
    { "input",    "Print input.",            input_cb},
    { "output",   "Get/set output on/off.",  output_cb},
    { "count",    "Counts of controls.",     count_cb},
    { "all",      "Print everything.",       all_cb},
    { "pwm",      "Get/set PWM",             pwm_cb},
    { "version",  "Print version.",          version_cb},
    { NULL },
};



static unsigned _read_index(char ** pos)
{
    unsigned r = strtoul(rx_buffer + rx_pos, pos, 10);
    if (r)
        r--;
    return r;
}

void adc_cb()
{
    adcs_adc_log(_read_index(NULL));
}


void adcex_cb()
{
    adcs_ex_adc_log(_read_index(NULL));
}

void adcs_ex_loop()
{
    unsigned adc = _read_index(NULL);
    unsigned input_count = uart_ring_in_length(0);
    unsigned cur_tick = uptime;
    int prev_value = 0;
    while(true)
    {
        if (cur_tick != uptime)
        {
            log_out(LOG_START_SPACER);
            unsigned temp = adc_ex_get(adc);
            log_out("ADCEX: %u", adc + 1);
            log_out("RAW: %u (delta:%i)", temp, ((int)temp)-prev_value);
            log_out(LOG_END_SPACER);
            prev_value = temp;
            cur_tick = uptime;
        }

        if (input_count != uart_ring_in_length(0))
            break;
        uart_rings_in_drain();
        uart_rings_out_drain();
    }
}


void input_cb()
{
    inputs_input_log(_read_index(NULL));
}


void output_cb()
{
    char * pos = NULL;
    unsigned output = _read_index(&pos);

    if (pos)
        while(*pos == ' ')
            pos++;

    if (pos && *pos)
    {
        bool on_off = (strtoul(pos, NULL, 10))?true:false;

        log_out("Set output %02u to %s", output + 1, (on_off)?"ON":"OFF");
        outputs_set(output, on_off);
    }
    else output_output_log(output);
}


void count_cb()
{
    log_out("Inputs  : %u", inputs_get_count());
    log_out("Outputs : %u", outputs_get_count());
    log_out("ADCs    : %u", adcs_get_count());
    log_out("ADCEXs  : %u", adcs_ex_get_count());
    log_out("PWMs    : 1");
}


void all_cb()
{
    inputs_log();
    outputs_log();
    adcs_log();
}


void pwm_cb()
{
    char * pos = NULL;
    unsigned freq = strtoul(rx_buffer + rx_pos, &pos, 10);
    unsigned duty;

    if (pos)
        while(*pos == ' ')
            pos++;

    if (pos && *pos)
    {
        duty = strtoul(pos, &pos, 10);
        duty *= 100;
        if (*pos == '.')
        {
            pos++;
            if (*pos)
            {
                duty += ((*pos++)-'0') * 10;
                if (*pos)
                    duty += ((*pos++)-'0') * 1;
            }
        }
        if (duty > PWM_MAX)
            duty = PWM_MAX;
        pwm_set(freq, duty);
    }

    pwm_get(&freq, &duty);
    log_out("PWM Freq : %u", freq);
    log_out("PWM Duty : %u.%02u", duty/100, duty%100);
}


void version_cb()
{
    log_out("Version : %s", GIT_VERSION);
}


void cmds_process(char * command, unsigned len)
{
    if (!len)
        return;

    log_debug(DEBUG_SYS, "Command \"%s\"", command);

    rx_buffer = command;
    rx_buffer_len = len;

    bool found = false;
    log_out(LOG_START_SPACER);
    for(cmd_t * cmd = cmds; cmd->key; cmd++)
    {
        unsigned keylen = strlen(cmd->key);
        if(rx_buffer_len >= keylen &&
           !strncmp(cmd->key, rx_buffer, keylen) &&
           (rx_buffer[keylen] == '\0' || rx_buffer[keylen] == ' '))
        {
            rx_pos = keylen;
            found = true;
            cmd->cb();
            break;
        }
    }
    if (!found)
    {
        log_out("Unknown command \"%s\"", rx_buffer);
        log_out(LOG_SPACER);
        for(cmd_t * cmd = cmds; cmd->key; cmd++)
            log_out("%10s : %s", cmd->key, cmd->desc);
    }
    log_out(LOG_END_SPACER);
}



void cmds_init()
{
}
