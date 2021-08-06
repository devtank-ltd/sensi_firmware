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
    {GPIOB, GPIO1},  /* LED 0 */ \
}


#define LED_PORT   GPIO(LED_PORT_N_PINS, 0).port
#define LED_PIN    GPIO(LED_PORT_N_PINS, 0).pins

#define USB_N_PINS                    \
{                                     \
    {GPIOA, GPIO12},    /* USB DP */  \
    {GPIOA, GPIO11},    /* USB DM */  \
}

#define USB_GPIO_PORT   GPIO(USB_N_PINS, 0).port
#define USB_DP_PIN      GPIO(USB_N_PINS, 0).pins
#define USB_DM_PIN      GPIO(USB_N_PINS, 1).pins
#define USB_GPIO_PINS   (USB_DP_PIN | USB_DM_PIN)
#define USB_ALT_FUNC    GPIO_AF14


#define ADCS_PORT_N_PINS                           \
{                                                  \
    {GPIOA, GPIO0},      /* ADC 0  = ADC1_IN1  */  \
    {GPIOA, GPIO2},      /* ADC 1  = ADC1_IN3  */  \
    {GPIOA, GPIO3},      /* ADC 2  = ADC1_IN4  */  \
    {GPIOA, GPIO4},      /* ADC 3  = ADC1_IN5  */  \
    {GPIOA, GPIO6},      /* ADC 4  = ADC2_IN3  */  \
    {GPIOA, GPIO7},      /* ADC 5  = ADC2_IN4  */  \
    {GPIOA, GPIO5},      /* ADC 6  = ADC2_IN2  */  \
    {GPIOC, GPIO0},      /* ADC 7  = ADC12_IN6 */  \
    {GPIOC, GPIO1},      /* ADC 8  = ADC12_IN7 */  \
    {GPIOC, GPIO2},      /* ADC 9  = ADC12_IN8 */  \
    {GPIOC, GPIO3},      /* ADC 10 = ADC12_IN9 */  \
    {GPIOC, GPIO4},      /* ADC 11 = ADC2_IN5  */  \
    {GPIOC, GPIO5},      /* ADC 12 = ADC2_IN11 */  \
}


/* schematics -> Connected to
 * PA0 ADC1_IN1  -> F4_OUT
 * PA2 ADC1_IN3  -> F3_OUT
 * PA3 ADC1_IN4  -> F2_OUT
 * PA4 ADC1_IN5  -> F1_OUT
 * PA6 ADC2_IN3  -> TH2_OUT
 * PA7 ADC2_IN4  -> AIN_BUF_CH3
 * PA5 ADC2_IN2  -> AIN_BUF_CH1
 * PC0 ADC12_IN6 -> I_MON
 * PC1 ADC12_IN7 -> TH4_OUT
 * PC2 ADC12_IN8 -> TH3_OUT
 * PC3 ADC12_IN9 -> TH1_OUT
 * PC4 ADC2_IN5  -> AIN_BUF_CH4
 * PC5 ADC2_IN11 -> AIN_BUF_CH2
 */

#define ADCS_CHANNEL \
{                    \
    {1, 1 },         \
    {1, 3 },         \
    {1, 4 },         \
    {1, 5 },         \
    {2, 3 },         \
    {2, 4 },         \
    {2, 2 },         \
    {1, 6 },         \
    {1, 7 },         \
    {1, 8 },         \
    {1, 9 },         \
    {2, 5 },         \
    {2, 11},         \
}


#define UART_CHANNELS                                                                                                            \
{                                                                                                                                \
    { USART3, RCC_USART3, UART_3_SPEED, GPIOB, GPIO10 | GPIO11, GPIO_AF7, NVIC_USART3_EXTI28_IRQ, UART3_PRIORITY }, /* UART 0 */ \
}

#define UART_DEBUG USART3

#define UART_CHANNELS_COUNT 1


#define INPUTS_PORT_N_PINS              \
{                                       \
    {GPIOA, GPIO15},   /* INPUT 1 */    \
    {GPIOC, GPIO10},   /* INPUT 2 */    \
    {GPIOC, GPIO11},   /* INPUT 3 */    \
    {GPIOC, GPIO12},   /* INPUT 4 */    \
    {GPIOB, GPIO12},   /* INPUT 5 */    \
    {GPIOA, GPIO1},    /* INPUT 6 */    \
}

/* schematics -> Connected to
 *     PA15 GPIO1 -> GPI01_EXT
 *     PC10 GPIO2 -> GPIO2_EXT
 *     PC11 GPIO3 -> GPIO3_EXT
 *     PC12 GPIO4 -> GPIO4_EXT
 *     PB12 GPIO5 -> GPIO5_EXT
 *     PA1  GPIO6 -> GPIO6_EXT
*/


#define INPUT_PULL                \
{                                 \
    GPIO_PUPD_PULLUP,    /* 1 */  \
    GPIO_PUPD_PULLUP,    /* 2 */  \
    GPIO_PUPD_PULLUP,    /* 3 */  \
    GPIO_PUPD_PULLUP,    /* 4 */  \
    GPIO_PUPD_PULLUP,    /* 5 */  \
    GPIO_PUPD_PULLUP,    /* 6 */  \
}

#define OUTPUTS_PORT_N_PINS              \
{                                        \
    {GPIOA, GPIO10},    /* OUTPUT 1 */   \
    {GPIOD, GPIO2},     /* OUTPUT 2 */   \
    {GPIOB, GPIO8},     /* OUTPUT 3 */   \
    {GPIOB, GPIO9},     /* OUTPUT 4 */   \
}

/* schematics -> Connected to
 *   PA10 HS_OUT1 -> HS_CTRL_SW1
 *   PD2  HS_OUT3 -> HS_CTRL_SW3
 *   PB8  HS_OUT4 -> HS_CTRL_SW4
 *   PB9  RL1     -> RL1_OUT
 *
 *   HS_OUT2 is the PWM.
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
    {GPIOA, GPIO8},   /* RTD_CS 1 */     \
    {GPIOA, GPIO9},   /* RTD_CS 2 */     \
    {GPIOB, GPIO14},  /* RTD_CS 3 */     \
    {GPIOC, GPIO7},   /* RTD_CS 4 */     \
}

#define RTD_INT_PORT_N_PINS              \
{                                        \
    {GPIOC, GPIO8},   /* RTD_INT 1 */    \
    {GPIOC, GPIO9},   /* RTD_INT 2 */    \
    {GPIOB, GPIO13},  /* RTD_INT 3 */    \
    {GPIOB, GPIO15},  /* RTD_INT 4 */    \
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
    GPIO_AF2,              \
}

#endif //__PINMAPS__
