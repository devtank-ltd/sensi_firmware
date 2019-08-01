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

/* Registers */
const uint8_t MAX31865_CONFIG              = 0x00;  /**< Configuration Register */
const uint8_t MAX31865_RTD_MSB             = 0x01;  /**< RTD Data MSB Register*/
const uint8_t MAX31865_RTD_LSB             = 0x02;  /**< RTD Data LSB Register*/
const uint8_t MAX31865_HI_FLT_THRHLD_MSB   = 0x03;  /**< High Fault Threshold MSB register */
const uint8_t MAX31865_HI_FLT_THRHLD_LSB   = 0x04;  /**< High Fault Threshold LSB register */
const uint8_t MAX31865_LO_FLT_THRHLD_MSB   = 0x05;  /**< Low Fault Threshold MSB register */
const uint8_t MAX31865_LO_FLT_THRHLD_LSB   = 0x06;  /**< Low Fault Threshold LSB register */
const uint8_t MAX31865_FAULT_STATUS        = 0x07;  /**< Fault Status Register */
const uint8_t MAX31865_REG_WRITE           = 0x80;  /**< Register address write bit */

/* Configuration bits */
const uint8_t MAX31865_VBIAS_ON            = 0x80;  /**< VBIAS on */
const uint8_t MAX31865_CONVERSION_AUTO     = 0x40;  /**< Conversion mode auto*/
const uint8_t MAX31865_ONE_SHOT            = 0x20;  /**< Perform single-shot reading */
const uint8_t MAX31865_THREE_WIRE          = 0x10;  /**< Select 3-wire RTD */
const uint8_t MAX31865_DETECTION_CYCLE     = 0x06;  /**< Fault-detection cycle */
const uint8_t MAX31865_FAULT_STATUS_CLEAR  = 0x02;  /**< Clear fault-status */
const uint8_t MAX31865_50HZ                = 0x01;  /**< Select 50Hz */

/**
 * @brief MAX31865 register buffer
 */
typedef union _max31865_register_t
{
    struct
    {
        uint8_t addr;    /**< Register offset address */
        uint8_t data;    /**< Register data */
    } u8;
    uint16_t buffer;
} max31865_register_t;

/**
 * @brief MAX31865 raw temperature buffer
 */
typedef union _raw_temperature_t
{
    struct
    {
        uint8_t msb;
        uint8_t lsb;
    } u8;
    uint16_t u16_temperature;
} raw_temperature_t;

static void max31865_delay(uint32_t delay)
{
    for (unsigned n=0; n<delay; n++)
    {
        asm("nop");
    }
}

static void max31865_enable_device(uint8_t chip)
{
    gpio_clear(MAX31865_PORT, chip);
}

static void max31865_disable_device(uint8_t chip)
{
    max31865_delay(500);
    gpio_set(MAX31865_PORT, chip);
}

//static uint8_t max31865_spi_send8(uint8_t data)
//{
////    uint16_t xfer_data = (data << 8) | 0xFF;
////    return spi_xfer(MAX31865_SPI, xfer_data);
//return spi_xfer(MAX31865_SPI, data);
//}

void max31865_init(void)
{
platform_raw_msg("max31865_init: start");

    // Assign chip select (active low)
    gpio_mode_setup(MAX31865_PORT,
                    GPIO_MODE_OUTPUT,
                    GPIO_PUPD_PULLUP,
                    MAX31865_SPI_CS_PIN0);

platform_raw_msg("max31865_init: init GPIO");

    // Assign alternative function 0 to SPI GPIO pins
    gpio_mode_setup(MAX31865_PORT,
                    GPIO_MODE_AF,
                    GPIO_PUPD_PULLUP,
                    MAX31865_SPI_AF_GPIOs);

    gpio_set_af(MAX31865_PORT,
                MAX31865_SPI_AF_GPIOS_F,
                MAX31865_SPI_AF_GPIOs);

    // TODO: CHIP1   gpio_mode_setup(MAX31865_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, MAX31865_SPI_CS_PIN1);

    // Assign data ready input (active low)
    gpio_mode_setup(MAX31865_DRDY_PORT,
                    GPIO_MODE_INPUT,
                    GPIO_PUPD_PULLUP,
                    MAX31865_DRDY_PIN);

//platform_raw_msg("max31865_init: spi_reset");
//    spi_reset(MAX31865_SPI);

    // Assign clock
    rcc_periph_clock_enable(PORT_TO_RCC(MAX31865_PORT));
    rcc_periph_clock_enable(MAX31865_RRC_SPI_CLK);

platform_raw_msg("max31865_init: spi_init_master");
    // Init as SPI master and set data size = 8-bits
    spi_init_master(MAX31865_SPI,
                    MAX31865_SPI_DIVIDER,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_MSBFIRST);

    spi_set_data_size(MAX31865_SPI, SPI_CR2_DS_8BIT);
platform_raw_msg("max31865_init: complete");
}

void max31865_config(void)
{
platform_raw_msg("max31865_config");

    // Device config - 3-wire, one-shot mode, vbias on, clear any faults
    uint8_t config = MAX31865_VBIAS_ON | MAX31865_THREE_WIRE | MAX31865_FAULT_STATUS_CLEAR;

platform_raw_msg("max31865 write config");

    max31865_enable_device(MAX31865_SPI_CS_0);
    max31865_write_register(MAX31865_CONFIG, config);
    max31865_disable_device(MAX31865_SPI_CS_0);

    // TODO: Chip 1
//    max31865_enable_device(MAX31865_SPI_CS_1);
//    max31865_write_register(MAX31865_CONFIG, config);
//    max31865_disable_device(MAX31865_SPI_CS_1);

}

void max31865_write_register(uint8_t addr, uint8_t data)
{
platform_raw_msg("max31865_write_register");
    max31865_register_t reg;
    reg.u8.addr = addr | MAX31865_REG_WRITE;
    reg.u8.data = data;

char buffer[48];
sprintf(buffer, "Write:%04X", reg.buffer);
platform_raw_msg(buffer);

    spi_write(MAX31865_SPI, reg.buffer);
}

uint8_t max31865_read_register(uint8_t addr)
{
    max31865_register_t reg;
    reg.u8.addr = addr;
    reg.u8.data = 0x00;

    spi_write(MAX31865_SPI, reg.buffer);
    reg.buffer = spi_read(MAX31865_SPI);

    return reg.u8.data;
}

void max31865_wait_for_data_ready(void)
{
    while (gpio_get(MAX31865_PORT, MAX31865_DRDY_PIN))
    {
        // Empty wait
        asm("nop");
    }
}

void max31865_start_reading(uint8_t chip)
{
    // Start a temperature reading
    uint8_t config;
    max31865_enable_device(chip);

    config = max31865_read_register(MAX31865_CONFIG);
    config |= MAX31865_ONE_SHOT;
    max31865_write_register(MAX31865_CONFIG, config);

    max31865_disable_device(chip);
}

uint16_t max31865_read_temperature(uint8_t chip)
{
    volatile uint16_t returnValue;

    max31865_enable_device(chip);

    returnValue = max31865_read_register(MAX31865_RTD_MSB);
    returnValue <<= 8;
    returnValue |= max31865_read_register(MAX31865_RTD_LSB);

    max31865_disable_device(chip);

    return returnValue;
}

double max31865_convert_temperature(uint16_t raw_value,
                                    uint16_t ref_resistance)
{
    // Algorithm and magic numbers courtesy of MikroElektonica...

    double value;
    double coefficient = (ref_resistance/400.0);

    raw_value >>= 1;
    value = (double)raw_value * coefficient;
    value /= 32;
    value -= 256;

    return value;
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
