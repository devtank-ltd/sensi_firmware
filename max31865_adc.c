/*
  max31865.c

  SPI driver for the MAX31865 RTD device

  This file is adapted from the MikroElektonika (http://www.mikroe.com)
  miroSDK driver: __rtd_driver.c
 */
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <stdio.h>

#include "log.h"
#include "max31865_adc.h"

const uint8_t MAX31865_OK     = 0;                    /**< NO ERROR */
const uint8_t MAX31865_ERR    = 1;                  /**< ERROR OCCURED */

const uint8_t MAX31865_CHIP_ENABLE = 0;                /** Enable chip-select */
const uint8_t MAX31865_CHIP_DISABLE = 1;            /** Disable chip-select */

const uint16_t MAX31865_REF_RESISTANCE_470 = 470;    /**< Value of referent resistor in Ohms */

/* Registers - read address */
const uint8_t MAX31865_CONFIG               = 0x00;  /**< Configuration Register */
const uint8_t MAX31865_RTD_MSB              = 0x01;  /**< RTD Data MSB Register*/
const uint8_t MAX31865_RTD_LSB              = 0x02;  /**< RTD Data LSB Register*/
const uint8_t MAX31865_HI_FLT_THRHLD_MSB    = 0x03;  /**< High Fault Threshold MSB register */
const uint8_t MAX31865_HI_FLT_THRHLD_LSB    = 0x04;  /**< High Fault Threshold LSB register */
const uint8_t MAX31865_LO_FLT_THRHLD_MSB    = 0x05;  /**< Low Fault Threshold MSB register */
const uint8_t MAX31865_LO_FLT_THRHLD_LSB    = 0x06;  /**< Low Fault Threshold LSB register */
const uint8_t MAX31865_FAULT_STATUS         = 0x07;  /**< Fault Status Register */

/* Registers - write address */
const uint8_t MAX31865_WR_CONFIG            = 0x80;  /**< Configuration Register */
const uint8_t MAX31865_WR_RTD_MSB           = 0x81;  /**< RTD Data MSB Register*/
const uint8_t MAX31865_WR_RTD_LSB           = 0x82;  /**< RTD Data LSB Register*/
const uint8_t MAX31865_WR_HI_FLT_THRHLD_MSB = 0x83;  /**< High Fault Threshold MSB register */
const uint8_t MAX31865_WR_HI_FLT_THRHLD_LSB = 0x84;  /**< High Fault Threshold LSB register */
const uint8_t MAX31865_WR_LO_FLT_THRHLD_MSB = 0x85;  /**< Low Fault Threshold MSB register */
const uint8_t MAX31865_WR_LO_FLT_THRHLD_LSB = 0x86;  /**< Low Fault Threshold LSB register */


/* Configuration bits */
const uint8_t MAX31865_VBIAS_ON             = 0x80;  /**< VBIAS on */
const uint8_t MAX31865_CONVERSION_AUTO      = 0x40;  /**< Conversion mode auto*/
const uint8_t MAX31865_ONE_SHOT             = 0x20;  /**< Perform single-shot reading */
const uint8_t MAX31865_THREE_WIRE           = 0x10;  /**< Select 3-wire RTD */
const uint8_t MAX31865_DETECTION_CYCLE      = 0x06;  /**< Fault-detection cycle */
const uint8_t MAX31865_FAULT_STATUS_CLEAR   = 0x02;  /**< Clear fault-status */
const uint8_t MAX31865_50HZ                 = 0x01;  /**< Select 50Hz */


const port_n_pins_t rtd_cs_port_n_pins[]  = RTD_CS_PORT_N_PINS;
const port_n_pins_t rtd_int_port_n_pins[] = RTD_INT_PORT_N_PINS;

/**
 * @brief MAX31865 transfer buffer
 */
typedef union _max31865_xfer_t
{
    struct
    {
        uint8_t addr;    /**< Register offset address */
        uint8_t value;   /**< Register value */
    } reg;
    struct
    {
        uint8_t data_lo;
        uint8_t data_hi;
    } data;
    uint16_t buffer;
} max31865_xfer_t;

static void max31865_delay(uint32_t delay)
{
    for (unsigned n=0; n<delay; n++)
    {
        asm("nop");
    }
}

static void max31865_enable_device(uint8_t chip)
{
    port_n_pins_t port_n_pin = rtd_cs_port_n_pins[chip];
    gpio_clear(port_n_pin.port, port_n_pin.pins);
}

static void max31865_disable_device(uint8_t chip)
{
    port_n_pins_t port_n_pin = rtd_cs_port_n_pins[chip];
    max31865_delay(10000);
    gpio_set(port_n_pin.port, port_n_pin.pins);
}

static uint8_t max31865_spi_send(uint8_t addr, uint8_t data)
{
    max31865_xfer_t xfer;
    xfer.reg.addr  = addr;
    xfer.reg.value = data;

    log_debug(DEBUG_ADC_EX, "XFER WR:%04X", xfer.buffer);
    xfer.buffer = spi_xfer(MAX31865_SPI, xfer.buffer);
    log_debug(DEBUG_ADC_EX, "XFER RD:%04X", xfer.buffer);

    return spi_xfer(MAX31865_SPI, data);
}

void max31865_init(void)
{
    // Assign clock
    rcc_periph_clock_enable(PORT_TO_RCC(MAX31865_PORT));
    rcc_periph_clock_enable(MAX31865_RRC_SPI_CLK);

    // Assign GPIO
    for(unsigned n = 0; n < RTD_COUNT; n++)
    {
        gpio_mode_setup(rtd_cs_port_n_pins[n].port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,   rtd_cs_port_n_pins[n].pins);
        gpio_mode_setup(rtd_int_port_n_pins[n].port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, rtd_int_port_n_pins[n].pins);
    }

    // Assign alternative function 0 to SPI GPIO pins
    gpio_mode_setup(MAX31865_PORT,
                    GPIO_MODE_AF,
                    GPIO_PUPD_NONE,
                    MAX31865_SPI_AF_GPIOs);

    gpio_set_af(MAX31865_PORT,
                MAX31865_SPI_AF_GPIOS_F,
                MAX31865_SPI_AF_GPIOs);

    // TODO: CHIP1   gpio_mode_setup(MAX31865_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, MAX31865_RTD_EXTERNAL);

    spi_reset(MAX31865_SPI);

    // Init as SPI master and set data size = 8-bits
    spi_init_master(MAX31865_SPI,
                    MAX31865_SPI_DIVIDER,
                    SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_2,
                    SPI_CR1_MSBFIRST);

    spi_set_data_size(MAX31865_SPI, SPI_CR2_DS_8BIT);

    log_out("max31865_init: complete");
}

void max31865_config(uint8_t chip)
{
    log_out("max31865_config chip %u", chip);

    // Device config - 3-wire, one-shot mode, vbias on, clear any faults

    uint8_t config = MAX31865_VBIAS_ON | \
                     MAX31865_CONVERSION_AUTO | \
                     MAX31865_THREE_WIRE | \
                     MAX31865_FAULT_STATUS_CLEAR;

    log_out("max31865 write config chip %u", chip);

    max31865_enable_device(chip);
    max31865_write_register(MAX31865_WR_CONFIG, config);
    max31865_disable_device(chip);
}

void max31865_write_register(uint8_t addr, uint8_t value)
{
    max31865_xfer_t reg;
    reg.reg.addr  = addr;
    reg.reg.value = value;

    log_debug(DEBUG_ADC_EX, "max31865_write_register addr:%u value:%u", addr, value);
    max31865_spi_send(addr, value);
    log_debug(DEBUG_ADC_EX, "Writen:%04X", reg.buffer);
}

uint8_t max31865_read_register(uint8_t addr)
{
    max31865_xfer_t xfer;
    xfer.reg.addr  = addr;
    xfer.reg.value = 0xFF;

    log_debug(DEBUG_ADC_EX, "max31865_read_register addr:%u", addr);
    xfer.buffer = spi_xfer(MAX31865_SPI, xfer.buffer);
    log_debug(DEBUG_ADC_EX, "Read:%04X : %u", xfer.buffer, xfer.data.data_hi);

    return xfer.data.data_hi;
}

void max31865_wait_for_data_ready(uint8_t chip)
{
    port_n_pins_t port_n_pin = rtd_int_port_n_pins[chip];

    while (gpio_get(port_n_pin.port, port_n_pin.pins))
    {
        // Empty wait
        asm("nop");
    }
}

void max31865_start_reading(uint8_t chip)
{
    // Start a temperature reading
    uint8_t config;

    // Do a one-shot conversion
    max31865_enable_device(chip);

    config = max31865_read_register(MAX31865_CONFIG);

    config |= MAX31865_ONE_SHOT;

    max31865_write_register(MAX31865_WR_CONFIG, config);

    max31865_disable_device(chip);
}

int16_t max31865_read_temperature(uint8_t chip)
{
    volatile uint16_t returnValue;

    max31865_enable_device(chip);

    returnValue = max31865_read_register(MAX31865_RTD_MSB);
    returnValue <<= 8;
    returnValue |= max31865_read_register(MAX31865_RTD_LSB);

    max31865_disable_device(chip);

    return returnValue;
}


/* -------------------------------------------------------------------------- */
/*
  __rtd_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */
