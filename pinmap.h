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
    {GPIOA, GPIO0},      /* ADC 1  = Channel 0  */  \
    {GPIOA, GPIO2},      /* ADC 2  = Channel 2  */  \
    {GPIOA, GPIO3},      /* ADC 3  = Channel 3  */  \
    {GPIOA, GPIO4},      /* ADC 4  = Channel 4  */  \
    {GPIOA, GPIO6},      /* ADC 5  = Channel 6  */  \
    {GPIOA, GPIO7},      /* ADC 6  = Channel 7  */  \
    {GPIOB, GPIO1},      /* ADC 7  = Channel 9  */  \
    {GPIOC, GPIO0},      /* ADC 8  = Channel 10 */  \
    {GPIOC, GPIO1},      /* ADC 9  = Channel 11 */  \
    {GPIOC, GPIO2},      /* ADC 10 = Channel 12 */  \
    {GPIOC, GPIO3},      /* ADC 11 = Channel 13 */  \
    {GPIOC, GPIO4},      /* ADC 12 = Channel 14 */  \
    {GPIOC, GPIO5},      /* ADC 13 = Channel 15 */  \
}

/* schematics -> Connected to
 *     ADC1  -> F4_OUT
 *     ADC2  -> F3_OUT
 *     ADC3  -> F2_OUT
 *     ADC4  -> F1_OUT
 *     ADC5  -> TH2_OUT
 *     ADC6  -> AIN_BUF_CH3
 *     ADC7  -> AIN_BUF_CH1
 *     ADC8  -> I_MON
 *     ADC9  -> TH4_OUT
 *     ADC10 -> TH3_OUT
 *     ADC11 -> TH1_OUT
 *     ADC12 -> AIN_BUF_CH4
 *     ADC13 -> AIN_BUF_CH2
 */


#define ADC_CHANNELS  {0,2,3,4,6,7,9,10,11,12,13,14,15}


#define UART_CHANNELS                                                                                                       \
{                                                                                                                           \
    { USART3, RCC_USART3, UART_3_SPEED, GPIOB, GPIO10 | GPIO11, GPIO_AF4, NVIC_USART3_4_IRQ, UART3_PRIORITY }, /* UART 0 */ \
}

#define UART_CHANNELS_COUNT 1


#define INPUTS_PORT_N_PINS              \
{                                       \
    {GPIOA, GPIO15},   /* INPUT 1 */    \
    {GPIOC, GPIO10},   /* INPUT 2 */    \
    {GPIOC, GPIO11},   /* INPUT 3 */    \
    {GPIOC, GPIO12},   /* INPUT 4 */    \
    {GPIOB, GPIO12},   /* INPUT 5 */    \
    {GPIOC, GPIO13},   /* INPUT 6 */    \
    {GPIOA, GPIO1},    /* INPUT 7 */    \
}

/* schematics -> Connected to
 *     GPIO1 -> GPI01
 *     GPIO2 -> GPIO2
 *     GPIO3 -> GPIO3
 *     GPIO4 -> GPIO4
 *     GPIO5 -> Not Used
 *     SB1   -> SB1
 *     SB2   -> SB2
*/


#define INPUT_PULL                \
{                                 \
    GPIO_PUPD_PULLUP,    /* 1 */  \
    GPIO_PUPD_PULLUP,    /* 2 */  \
    GPIO_PUPD_PULLUP,    /* 3 */  \
    GPIO_PUPD_PULLUP,    /* 4 */  \
    GPIO_PUPD_PULLUP,    /* 5 */  \
    GPIO_PUPD_PULLUP,    /* 6 */  \
    GPIO_PUPD_PULLUP,    /* 7 */  \
}

#define OUTPUTS_PORT_N_PINS              \
{                                        \
    {GPIOA, GPIO10},    /* OUTPUT 1 */   \
    {GPIOD, GPIO2},     /* OUTPUT 2 */   \
    {GPIOB, GPIO8},     /* OUTPUT 3 */   \
    {GPIOB, GPIO9},     /* OUTPUT 4 */   \
}

/* schematics -> Connected to
 *   HS_OUT1 -> HS_CTRL_SW1
 *   HS_OUT3 -> HS_CTRL_SW3
 *   HS_OUT4 -> HS_CTRL_SW4
 *   RL1     -> RL1_OUT
*/

#define OUTPUT_PULL                      \
{                                        \
    GPIO_PUPD_PULLDOWN,      /* 1  */    \
    GPIO_PUPD_PULLDOWN,      /* 2  */    \
    GPIO_PUPD_PULLDOWN,      /* 3  */    \
    GPIO_PUPD_PULLDOWN,      /* 4  */    \
}


#define SPI_PORT_N_PINS                             \
{                                                   \
    {GPIOB, GPIO3 | GPIO4 | GPIO5}, /* SPI 1*/      \
}

#define RTD_CS_PORT_N_PINS               \
{                                        \
    {GPIOA, GPIO8},   /* RTN_CS 1 */     \
    {GPIOC, GPIO9},   /* RTN_CS 2 */     \
    {GPIOB, GPIO14},  /* RTN_CS 3 */     \
    {GPIOC, GPIO7},   /* RTN_CS 4 */     \
}

#define RTD_INT_PORT_N_PINS              \
{                                        \
    {GPIOC, GPIO8},   /* RTN_INT 1 */    \
    {GPIOA, GPIO9},   /* RTN_INT 2 */    \
    {GPIOB, GPIO13},  /* RTN_INT 3 */    \
    {GPIOB, GPIO15},  /* RTN_INT 4 */    \
}

#define RTD_COUNT 4

#define MAX31865_RRC_SPI_CLK RCC_SPI1
#define MAX31865_PORT           GPIO(SPI_PORT_N_PINS, 0).port

#define MAX31865_SPI_AF_GPIOs   GPIO(SPI_PORT_N_PINS, 0).pins
#define MAX31865_SPI_AF_GPIOS_F GPIO_AF0


#define MAX31865_SPI            SPI1
#define MAX31865_SPI_DIVIDER    SPI_CR1_BAUDRATE_FPCLK_DIV_256

#define I2C_PORT_N_PINS                             \
{                                                   \
    {GPIOB, GPIO6},               /* I2C SCL */    \
    {GPIOB, GPIO7},               /* I2C SDA */    \
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
