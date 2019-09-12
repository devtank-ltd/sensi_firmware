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

#define ADCS_PORT_N_PINS                             \
{                                                    \
    {GPIOA, GPIO0},      /* ADC 1  = Channel 0  */  \
    {GPIOA, GPIO1},      /* ADC 2  = Channel 1  */  \
    {GPIOA, GPIO4},      /* ADC 3  = Channel 4  */  \
    {GPIOA, GPIO6},      /* ADC 4  = Channel 6  */  \
    {GPIOA, GPIO7},      /* ADC 5  = Channel 7  */  \
    {GPIOB, GPIO0},      /* ADC 6  = Channel 8  */  \
    {GPIOB, GPIO1},      /* ADC 7  = Channel 9  */  \
    {GPIOC, GPIO0},      /* ADC 8  = Channel 10 */  \
    {GPIOC, GPIO1},      /* ADC 9  = Channel 11 */  \
    {GPIOC, GPIO2},      /* ADC 10 = Channel 12 */  \
    {GPIOC, GPIO3},      /* ADC 11 = Channel 13 */  \
    {GPIOC, GPIO4},      /* ADC 12 = Channel 14 */  \
    {GPIOC, GPIO5},      /* ADC 13 = Channel 15 */  \
}

#define ADC_CHANNELS  {0,1,4,6,7,8,9,10,11,12,13,14,15}


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
    {GPIOC, GPIO13},   /* INPUT 6 */    \
    {GPIOD, GPIO2},    /* INPUT 7 */    \
    {GPIOB, GPIO2},    /* INPUT 8 */    \
    {GPIOB, GPIO8},    /* INPUT 9 */    \
    {GPIOB, GPIO6},    /* INPUT 10 */   \
    {GPIOB, GPIO9},    /* INPUT 11 */   \
    {GPIOC, GPIO7},    /* INPUT 12 */   \
    {GPIOC, GPIO9},    /* INPUT 13 */   \
    {GPIOB, GPIO11},   /* INPUT 14 */   \
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
    GPIO_PUPD_PULLUP,    /* 9 */  \
    GPIO_PUPD_PULLUP,    /* 10 */ \
    GPIO_PUPD_PULLUP,    /* 11 */ \
    GPIO_PUPD_PULLUP,    /* 12 */ \
    GPIO_PUPD_PULLUP,    /* 13 */ \
    GPIO_PUPD_PULLUP,    /* 14 */ \
}
/*
 * NOTE: A13 and A14 will cause SWD problems.
 */

#define OUTPUTS_PORT_N_PINS              \
{                                        \
    {GPIOA, GPIO9},     /* OUTPUT 1 */   \
    {GPIOA, GPIO10},    /* OUTPUT 2 */   \
    {GPIOA, GPIO13},    /* OUTPUT 3 */   \
    {GPIOA, GPIO14},    /* OUTPUT 4 */   \
    {GPIOC, GPIO8},     /* OUTPUT 5 */   \
}

#define OUTPUT_PULL                      \
{                                        \
    GPIO_PUPD_PULLDOWN,      /* 1  */    \
    GPIO_PUPD_PULLDOWN,      /* 2  */    \
    GPIO_PUPD_PULLDOWN,      /* 3  */    \
    GPIO_PUPD_PULLDOWN,      /* 4  */    \
    GPIO_PUPD_PULLDOWN,      /* 5  */    \
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

#define PWM_TIMER      TIM3
#define PWM_TIMER_CH   TIM_OC1
#define RCC_PWM_TIMER  RCC_TIM3

#define PWM_PORT_N_PINS           \
{                                 \
    {GPIOC, GPIO6},    /* PWM */ \
}

#define PWM_GPIO_FUNC      \
{                          \
    GPIO_AF0,              \
}

#endif //__PINMAPS__
