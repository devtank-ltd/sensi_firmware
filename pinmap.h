#ifndef __PINMAPS__
#define __PINMAPS__

#include <stdint.h>

typedef struct
{
    uint32_t port;
    uint32_t pins;
} port_n_pins_t;

#define PORT_TO_RCC(_port_)   (RCC_GPIOA + ((_port_ - GPIO_PORT_A_BASE) / 0x400))

#define LED_PORT   GPIOA
#define LED_PIN    GPIO5

#define ADCS_PORT_N_PINS                            \
{                                                   \
    {GPIOA, GPIO4},      /* ADC 0  = Channel 4  */  \
    {GPIOA, GPIO6},      /* ADC 1  = Channel 6  */  \
    {GPIOA, GPIO7},      /* ADC 2  = Channel 7  */  \
    {GPIOB, GPIO0},      /* ADC 3  = Channel 8  */  \
    {GPIOB, GPIO1},      /* ADC 4  = Channel 9  */  \
    {GPIOC, GPIO0},      /* ADC 5  = Channel 10 */  \
    {GPIOC, GPIO1},      /* ADC 6  = Channel 11 */  \
    {GPIOC, GPIO3},      /* ADC 7  = Channel 13 */  \
    {GPIOC, GPIO4},      /* ADC 8  = Channel 14 */  \
    {GPIOC, GPIO5},      /* ADC 9  = Channel 15 */  \
}

#define ADC_CHANNELS  {4,6,7,8,9,10,11,13,14,15}

#define ADC_COUNT 10

#define PPS_PORT_N_PINS             \
{                                   \
    {GPIOB, GPIO3},     /* PPS 0 */ \
    {GPIOC, GPIO7},     /* PPS 1 */ \
}


#define PPS_EXTI      \
{                     \
    {TIM1, EXTI3},    \
    {TIM14, EXTI7},   \
}

#define PPS_INIT                     \
{                                    \
    {RCC_TIM1, NVIC_EXTI2_3_IRQ },   \
    {RCC_TIM14, NVIC_EXTI4_15_IRQ},  \
}

#define PPS0_EXTI_ISR        exti2_3_isr
#define PPS1_EXTI_ISR        exti4_15_isr


#define UART_CHANNELS                                                                                                     \
{                                                                                                                         \
    { USART2, RCC_USART2, UART_2_SPEED, GPIOA, GPIO2  | GPIO3,  GPIO_AF1, NVIC_USART2_IRQ, UART2_PRIORITY }, /* UART 0 */ \
}

#define UART_CHANNELS_COUNT 1


#define ADS1248_RRC_SPI_CLK     RCC_SPI1
#define ADS1248_PORT            GPIOA

#define ADS1248_SPI_AF_GPIOs    (GPIO5 | GPIO6 | GPIO7)
#define ADS1248_SPI_AF_GPIOS_F  GPIO_AF0

#define ADS1248_SPI_CS_PIN      GPIO9

#define ADS1248_SPI             SPI1
#define ADS1248_SPI_DIVIDER     SPI_CR1_BAUDRATE_FPCLK_DIV_8

#define ADS1248_DRDY_PORT       GPIOA
#define ADS1248_DRDY_PIN        GPIO8

#define ADS1248_START_PORT      GPIOA
#define ADS1248_START_PIN       GPIO10

#define ADS1248_RESET_PORT      GPIOB
#define ADS1248_RESET_PIN       GPIO10



#define INPUTS_PORT_N_PINS              \
{                                       \
    {GPIOC, GPIO12},   /* Input 0 */    \
    {GPIOA, GPIO15},   /* Input 1 */    \
    {GPIOB, GPIO7},    /* Input 2 */    \
    {GPIOA, GPIO8},    /* Input 3 */    \
    {GPIOB, GPIO10},   /* Input 4 */    \
    {GPIOB, GPIO4},    /* Input 5 */    \
    {GPIOD, GPIO2},    /* Input 6 */    \
    {GPIOC, GPIO2},    /* Input 7 */    \
    {GPIOB, GPIO13},   /* Input 8 */    \
}

#define INPUT_PULL                \
{                                 \
    GPIO_PUPD_PULLUP,    /* 0 */  \
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
    {GPIOC, GPIO8},     /* Output 0 */   \
    {GPIOC, GPIO6},     /* Output 1 */   \
    {GPIOB, GPIO12},    /* Output 2 */   \
    {GPIOB, GPIO11},    /* Output 3 */   \
    {GPIOB, GPIO2},     /* Output 4 */   \
    {GPIOB, GPIO6},     /* Output 5 */   \
    {GPIOB, GPIO14},    /* Output 6 */   \
    {GPIOB, GPIO15},    /* Output 7 */   \
    {GPIOA, GPIO9},     /* Output 8 */   \
    {GPIOA, GPIO10},    /* Output 9 */   \
    {GPIOA, GPIO13},    /* Output 10 */  \
    {GPIOA, GPIO14},    /* Output 11 */  \
    {GPIOC, GPIO13},    /* Output 12 */  \
    {GPIOB, GPIO5},     /* Output 13 */  \
    {GPIOB, GPIO8},     /* Output 14 */  \
    {GPIOB, GPIO9},     /* Output 15 */  \
}

#define OUTPUT_PULL                      \
{                                        \
    GPIO_PUPD_PULLDOWN,      /* 0  */    \
    GPIO_PUPD_PULLDOWN,      /* 1  */    \
    GPIO_PUPD_PULLDOWN,      /* 2  */    \
    GPIO_PUPD_PULLDOWN,      /* 3  */    \
    GPIO_PUPD_PULLDOWN,      /* 4  */    \
    GPIO_PUPD_PULLUP,        /* 5  */    \
    GPIO_PUPD_PULLUP,        /* 6  */    \
    GPIO_PUPD_PULLUP,        /* 7  */    \
    GPIO_PUPD_PULLDOWN,      /* 8  */    \
    GPIO_PUPD_PULLDOWN,      /* 9  */    \
    GPIO_PUPD_PULLDOWN,      /* 10 */    \
    GPIO_PUPD_PULLDOWN,      /* 11 */    \
    GPIO_PUPD_PULLDOWN,      /* 12 */    \
    GPIO_PUPD_PULLDOWN,      /* 13 */    \
    GPIO_PUPD_PULLDOWN,      /* 14 */    \
    GPIO_PUPD_PULLDOWN,      /* 15 */    \
}

#endif //__PINMAPS__
