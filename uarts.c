#include <stdlib.h>

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/cm3/nvic.h>

#include "cmd.h"
#include "log.h"
#include "usb_uarts.h"
#include "pinmap.h"
#include "ring.h"
#include "uart_rings.h"
#include "uarts.h"


typedef struct
{
    uint32_t              usart;
    enum rcc_periph_clken uart_clk;
    uint32_t              baud;
    uint32_t              gpioport;
    uint16_t              pins;
    uint8_t               alt_func_num;
    uint8_t               irqn;
    uint8_t               priority;
} uart_channel_t;

static const uart_channel_t uart_channels[] = UART_CHANNELS;


static void uart_setup(const uart_channel_t * channel)
{
    rcc_periph_clock_enable(PORT_TO_RCC(channel->gpioport));
    rcc_periph_clock_enable(channel->uart_clk);

    gpio_mode_setup( channel->gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, channel->pins );
    gpio_set_af( channel->gpioport, channel->alt_func_num, channel->pins );

    usart_set_baudrate( channel->usart, channel->baud );
    usart_set_databits( channel->usart, 8 );
    usart_set_stopbits( channel->usart, USART_CR2_STOPBITS_1 );
    usart_set_mode( channel->usart, USART_MODE_TX_RX );
    usart_set_parity( channel->usart, USART_PARITY_NONE );
    usart_set_flow_control( channel->usart, USART_FLOWCONTROL_NONE );

    nvic_set_priority(channel->irqn, channel->priority);
    nvic_enable_irq(channel->irqn);
    usart_enable(channel->usart);
    usart_enable_rx_interrupt(channel->usart);
}



static bool uart_getc(uint32_t uart, char* c)
{
    uint32_t flags = USART_ISR(uart);

    if (!(flags & USART_ISR_RXNE))
    {
        USART_ICR(uart) = flags;
        return false;
    }

    *c = usart_recv(uart);

    return ((*c) != 0);
}


void process_debug(void)
{
    char c;

    if (!uart_getc(uart_channels[0].usart, &c))
    {
        return;
    }

    switch(c)
    {
        case 'D':
            log_debug_mask = DEBUG_SYS;
            log_debug(DEBUG_SYS, "Enabling Debug via debug comms");
            log_debug(DEBUG_SYS, "U = enable UART debug");
            log_debug(DEBUG_SYS, "A = enable ADC debug");
            log_debug(DEBUG_SYS, "X = enable ADCEX debug");
            log_debug(DEBUG_SYS, "R = show UART ring buffers");
            break;
        case 'A':
            if (log_debug_mask)
            {
                log_debug(DEBUG_SYS, "Enabled ADC debug");
                log_debug_mask |= DEBUG_ADC;
            }
            break;
        case 'X':
            if (log_debug_mask)
            {
                log_debug(DEBUG_SYS, "Enabled ADCEX debug");
                log_debug_mask |= DEBUG_ADC_EX;
            }
            break;

        case 'U':
            if (log_debug_mask)
            {
                log_debug(DEBUG_SYS, "Enabled UART debug");
                log_debug_mask |= DEBUG_UART;
            }
            break;
        case 'R':
            if (log_debug_mask)
            {
                if (!(log_debug_mask & DEBUG_UART))
                    log_debug(DEBUG_SYS, "Enabled UART debug");
                log_debug_mask |= DEBUG_UART;
                uart_rings_check();
            }
            break;
        default:
            log_debug(DEBUG_SYS, "Disabling Debug via debug comms");
            log_debug_mask = 0;
    }
}


void usart3_exti28_isr(void)
{
    process_debug();
}


void uarts_setup(void)
{
    for(unsigned n = 0; n < UART_CHANNELS_COUNT; n++)
        uart_setup(&uart_channels[n]);
}


bool uart_is_tx_empty(unsigned uart)
{
    if (uart >= UART_CHANNELS_COUNT)
        return false;

    uart = uart_channels[uart].usart;

    return ((USART_ISR(uart) & USART_ISR_TXE));
}


bool uart_out(unsigned uart, char c)
{
    if (uart >= UART_CHANNELS_COUNT)
        return false;

    const uart_channel_t * channel = &uart_channels[uart];

    if (!(USART_ISR(channel->usart) & USART_ISR_TXE))
        return false;

    if (uart)
        log_debug(DEBUG_UART, "UART %u single out.", uart);

    usart_send(channel->usart, c);
    return true;
}
