#ifndef __PINMAPS__
#define __PINMAPS__

#include <stdint.h>
#include <libopencm3/stm32/gpio.h>

typedef struct
{
    uint32_t port;
    uint32_t pins;
} port_n_pins_t;

#define PORT_TO_RCC(_port_)   (RCC_GPIOA + ((_port_ - GPIO_PORT_A_BASE) / 0x400))

#define GPIO(_array_, _index_) ((port_n_pins_t[])_array_)[_index_]

#define LED_PORT_N_PINS           \
{                                 \
    {GPIOC, GPIO10},  /* LED 0 */ \
}


#define LED_PORT   GPIO(LED_PORT_N_PINS, 0).port
#define LED_PIN    GPIO(LED_PORT_N_PINS, 0).pins

#define USB_N_PINS                    \
{                                     \
    {GPIOA, GPIO12},    /* USB DP */  \
    {GPIOA, GPIO11},    /* USB DM */  \
}

#define ADCS_PORT_N_PINS                            \
{                                                   \
    {GPIOA, GPIO4},      /* ADC 1  = Channel 4  */  \
    {GPIOB, GPIO0},      /* ADC 2  = Channel 8  */  \
    {GPIOB, GPIO1},      /* ADC 3  = Channel 9  */  \
    {GPIOC, GPIO0},      /* ADC 4  = Channel 10 */  \
    {GPIOC, GPIO1},      /* ADC 5  = Channel 11 */  \
    {GPIOC, GPIO3},      /* ADC 6  = Channel 13 */  \
    {GPIOC, GPIO4},      /* ADC 7  = Channel 14 */  \
    {GPIOC, GPIO5},      /* ADC 8  = Channel 15 */  \
}

#define ADC_CHANNELS  {4,6,7,8,9,10,11,13,14,15}

#define ADC_COUNT 10


#define UART_CHANNELS                                                                                                     \
{                                                                                                                         \
    { USART2, RCC_USART2, UART_2_SPEED, GPIOA, GPIO2  | GPIO3,  GPIO_AF1, NVIC_USART2_IRQ, UART2_PRIORITY }, /* UART 0 */ \
}

#define UART_CHANNELS_COUNT 1


#define INPUTS_PORT_N_PINS              \
{                                       \
    {GPIOC, GPIO12},   /* INPUT 1 */    \
    {GPIOA, GPIO15},   /* INPUT 2 */    \
    {GPIOB, GPIO7},    /* INPUT 3 */    \
    {GPIOA, GPIO8},    /* INPUT 4 */    \
    {GPIOB, GPIO10},   /* INPUT 5 */    \
    {GPIOB, GPIO4},    /* INPUT 6 */    \
    {GPIOD, GPIO2},    /* INPUT 7 */    \
    {GPIOC, GPIO2},    /* INPUT 8 */    \
}

#define INPUT_PULL                \
{                                 \
    GPIO_PUPD_PULLUP,    /* 1 */  \
    GPIO_PUPD_PULLUP,    /* 2 */  \
    GPIO_PUPD_PULLUP,    /* 3 */  \
    GPIO_PUPD_PULLUP,    /* 4 */  \
    GPIO_PUPD_PULLUP,    /* 5 */  \
    GPIO_PUPD_PULLUP,    /* 6 */  \
    GPIO_PUPD_PULLUP,    /* 7 */  \
    GPIO_PUPD_PULLUP,    /* 8 */  \
}
/*
 * NOTE: A13 and A14 will cause SWD problems.
 */

#define OUTPUTS_PORT_N_PINS              \
{                                        \
    {GPIOC, GPIO8},     /* OUTPUT 1 */   \
    {GPIOC, GPIO6},     /* OUTPUT 2 */   \
    {GPIOB, GPIO12},    /* OUTPUT 3 */   \
    {GPIOB, GPIO11},    /* OUTPUT 4 */   \
    {GPIOB, GPIO2},     /* OUTPUT 5 */   \
    {GPIOB, GPIO6},     /* OUTPUT 6 */   \
    {GPIOB, GPIO14},    /* OUTPUT 7 */   \
    {GPIOB, GPIO15},    /* OUTPUT 8 */   \
    {GPIOA, GPIO9},     /* OUTPUT 9 */   \
    {GPIOC, GPIO13},    /* OUTPUT 10 */   \
    {GPIOB, GPIO5},     /* OUTPUT 11 */  \
    {GPIOB, GPIO8},     /* OUTPUT 12 */  \
    {GPIOB, GPIO9},     /* OUTPUT 13 */  \
}

#define OUTPUT_PULL                      \
{                                        \
    GPIO_PUPD_PULLDOWN,      /* 1  */    \
    GPIO_PUPD_PULLDOWN,      /* 2  */    \
    GPIO_PUPD_PULLDOWN,      /* 3  */    \
    GPIO_PUPD_PULLDOWN,      /* 4  */    \
    GPIO_PUPD_PULLDOWN,      /* 5  */    \
    GPIO_PUPD_PULLUP,        /* 6  */    \
    GPIO_PUPD_PULLUP,        /* 7  */    \
    GPIO_PUPD_PULLUP,        /* 8  */    \
    GPIO_PUPD_PULLDOWN,      /* 9  */    \
    GPIO_PUPD_PULLDOWN,      /* 10  */    \
    GPIO_PUPD_PULLDOWN,      /* 11 */    \
    GPIO_PUPD_PULLDOWN,      /* 12 */    \
    GPIO_PUPD_PULLDOWN,      /* 13 */    \
}


#define MAX31865_SPI_SCLK       GPIO5
#define MAX31865_SPI_MISO       GPIO6
#define MAX31865_SPI_MOSI       GPIO7
#define MAX31865_SPI_CS_PIN0    GPIO0
#define MAX31865_DRDY_PORT      GPIOA
#define MAX31865_DRDY_PIN       GPIO1

#define SPI_PORT_N_PINS                             \
{                                                   \
    {GPIOA, GPIO5 | GPIO6 | GPIO7}, /* SPI 1*/      \
    {GPIOA, GPIO0},                 /* SPI 1 CS 0*/ \
    {GPIOA, GPIO1},                 /* SPI 1 DRDY*/ \
}

#define MAX31865_RRC_SPI_CLK RCC_SPI1
#define MAX31865_PORT           GPIO(SPI_PORT_N_PINS, 0).port

#define MAX31865_SPI_AF_GPIOs   GPIO(SPI_PORT_N_PINS, 0).pins
#define MAX31865_SPI_AF_GPIOS_F GPIO_AF0

#define MAX31865_RTD_INTERNAL   MAX31865_SPI_CS_PIN0

// TODO: Need a chip select pin for external RTD probe controller chip
// TODO: #define MAX31865_RTD_EXTERNAL     MAX31865_SPI_CS_PIN1

#define MAX31865_SPI            SPI1
#define MAX31865_SPI_DIVIDER    SPI_CR1_BAUDRATE_FPCLK_DIV_256

#define I2C_PORT_N_PINS                             \
{                                                   \
    {GPIOB, GPIO13},               /* I2C SCL */    \
    {GPIOC, GPIO14},               /* I2C SDA */    \
}


#define PWM_PORT_N_PINS           \
{                                 \
    {GPIOA, GPIO10},    /* PWM */ \
}

#define PWM_GPIO_FUNC      \
{                          \
    GPIO_AF1,              \
}

#endif //__PINMAPS__
