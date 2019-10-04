/*
    max31865_adc.h

-----------------------------------------------------------------------------

  This file is part adapted from the mikroSDK RTD driver
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "stdint.h"

#include "pinmap.h"

#ifndef _MAX31865_ADC_H_
#define _MAX31865_ADC_H_

extern const uint8_t MAX31865_CHIP_ENABLE;/**< Enable chip-select */
extern const uint8_t MAX31865_CHIP_DISABLE;/**< Disable chip-select */

extern const uint16_t MAX31865_REF_RESISTANCE_470;/**< Value of referent resistor in Ohms */

/* Registers */
extern const uint8_t MAX31865_CONFIG;/**< Configuration Register */
extern const uint8_t MAX31865_RTD_MSB;/**< RTD Data MSB Register*/
extern const uint8_t MAX31865_RTD_LSB;/**< RTD Data LSB Register*/
extern const uint8_t MAX31865_HI_FLT_THRHLD_MSB;/**< High Fault Threshold MSB register */
extern const uint8_t MAX31865_HI_FLT_THRHLD_LSB;/**< High Fault Threshold LSB register */
extern const uint8_t MAX31865_LO_FLT_THRHLD_MSB;/**< Low Fault Threshold MSB register */
extern const uint8_t MAX31865_LO_FLT_THRHLD_LSB;/**< Low Fault Threshold LSB register */
extern const uint8_t MAX31865_FAULT_STATUS;/**< Fault Status Register */
extern const uint8_t MAX31865_REG_WRITE;/**< Register address write bit */

/* Configuration bits */
extern const uint8_t MAX31865_VBIAS_ON;/**< VBIAS on */
extern const uint8_t MAX31865_CONVERSION_AUTO;/**< Conversion mode auto*/
extern const uint8_t MAX31865_ONE_SHOT;/**< Perform single-shot reading */
extern const uint8_t MAX31865_THREE_WIRE;/**< Select 3-wire RTD */
extern const uint8_t MAX31865_DETECTION_CYCLE;/**< Fault-detection cycle */
extern const uint8_t MAX31865_FAULT_STATUS_CLEAR;/**< Clear fault-status */
extern const uint8_t MAX31865_50HZ_FILTER;/**< Select 50Hz filter */

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief Initialize SPI bus
 *
 */
void max31865_init(void);

/**
 * @brief Configure MAX31865 chip
 * 
 * @param[in] chip Chip select
 *
 */
void max31865_config(uint8_t chip);

/**
 * @brief Write to Register
 *
 * @param[in] chip Chip select
 * @param[in] addr Register address
 * @param[in] value Value to write
 *
 * RTD SPI Write function, that will write input data into the chosen register.
 *
 * @note
 * This function automatically adds write bit as needed by this device.
 */
void max31865_write_register(uint8_t addr, uint8_t value);

/**
 * @brief Read from Register
 *
 * @param[in] chip Chip select
 * @param[in] addr Register address
 * @retval Read data
 *
 * RTD SPI Read function, that will read data from the chosen register.
 */
uint8_t max31865_read_register(uint8_t addr);

/**
 * @brief Start temperature reading
 *
 * Request a temperature reading
 *
 * @param[in] chip Chip select
 */
void max31865_start_reading(uint8_t chip);

/**
 * @brief Wait for temperature reading to complete
 *
 * @param[in] chip Chip select
 */
void max31865_wait_for_data_ready(uint8_t chip);

/**
 *
 * @brief Read Temperature
 *
 * Read raw temperature from the ADC
 *
 * @param[in] chip Chip select
 * @retval Raw temperature data
 */
int16_t max31865_read_temperature(uint8_t chip);

/**
 * @brief Convert a temperature value from raw->scaled
 *
 * This value will take raw input data (such as supplied by the read
 * temperature function), and will convert it into readable format.
 * Returned double value is in degrees Celsius.
 *
 * @param[in] raw_value Raw temperature value as read from the MAX31865
 * @param[in] ref_resistance Value of external referent resistance (Ohms)
 * @retval Converted temperature data
 *
 * @note
 * Referent resistance on the click board can be changed to better match the
 * sensor in use. Input the value of the resistor used in Ohms to get correct
 * data.
 */
double max31865_convert_temperature(int16_t raw_value,
                                    uint16_t ref_resistance);

#ifdef __cplusplus
} // extern "C"
#endif
#endif

/* -------------------------------------------------------------------------- */
/*
  __rtd_driver.h

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
