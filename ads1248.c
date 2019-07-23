/*
 * ads1248.c
 *
 * ADS1248 device functions
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*///******************************************************************************
//   ADS1248 Function File for Demo Functions
//
//   Description: SPI master talks to SPI slave (ADS1248) using 4-wire mode. LaunchPad
//   is used for this example.
//   ACLK = n/a, MCLK = SMCLK = DCO ~24MHz, BRCLK = SMCLK/7
//
//
//
//                                         MSP430F5529
//                                      -----------------
//                                     |             P1.3|<- Data Interrupt (DRDY)
//                                     |                 |
//                             START <-|P6.0         P6.2|-> Device Select (CS)
//                                     |                 |
//                             RESET <-|P6.1         P3.0|-> Data Out (UCB0SIMO -> DIN)
//                                     |                 |
//                                   <-|P1.6         P3.1|<- Data In (DOUT -> UCB0SOMI)
//                                     |                 |
// Serial Clock Out (UCB0CLK - SCLK) <-|P3.2         P2.7|->
//                                     |                 |
//                 I2C SCL (UCB1SCL) <-|P4.2         P8.1|->
//                                     |                 |
//                 I2C SDA (UCB1SDA)<>-|P4.1         P2.6|->
//                                     |                 |
//                                     |             P3.7|->
//                                     |                 |
//
//
//******************************************************************************

#include "ads1248.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*
 * ======== Standard MSP430 includes ========
 */
 
#ifdef __MSP430F5529__
#include <msp430.h>
#elif defined(STM32F0)
#include "pinmap.h"
#include "log.h"
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

uint32_t ext_adc_cs = ADS1248_SPI_CS_PIN0;

static unsigned char stm32_spi_byte(unsigned char data)
{
platform_raw_msg("stm32_spi_byte");

    ADS1248AssertCS(0);
    union
	{
    	uint8_t u8data[2];
    	uint16_t u16data;
	} u_data;

	u_data.u8data[0] = data;
	u_data.u8data[1] = 0x00;

	uint16_t ret_data = spi_xfer(ADS1248_SPI, u_data.u16data);
	u_data.u16data = ret_data;

//    spi_send8(ADS1248_SPI, data);
//    unsigned char in_data = spi_read8(ADS1248_SPI);
    ADS1248AssertCS(1);
    return u_data.u8data[1];
}


static void stm32_delay()
{
    for(unsigned n = 0; n < 100000; n++)
        asm("nop");
}

//static void TEST_DELAY()
//{
//    for(unsigned n = 0; n < 1000; n++)
//        asm("nop");
//}


#endif

void InitSPI(void)
{
#if defined (__MSP430F5529__)
    UCB0CTL1 |= UCSWRST;                        // Hold peripheral in reset
    UCB0CTL0 = UCMST + UCSYNC + UCMSB;          // SPI master, synchronous
    UCB0CTL1 = UCSSEL_2 + UCSWRST;              // Use SMCLK for bit rate clock and keep in reset
    UCB0BR0 = 14;                               // SMCLK/12 = SCLK (2MHz)
    UCB0CTL1 &= ~UCSWRST;                       // Release peripheral for use
#elif defined (PART_TM4C1294NCPDT)
    extern uint32_t sysfreq;
    uint32_t dataRx;

    SysCtlPeripheralEnable(SYSCTL_SSI);
    #ifdef DIFFERENT_CS_PORT
        GPIOPinTypeGPIOOutput(NCS_PORT, ADS1248_CS);
        //nCS_DIS;
    #endif
    GPIOPinConfigure(ADS1248_SCLK);
    GPIOPinConfigure(ADS1248_DOUT);
    GPIOPinConfigure(ADS1248_DIN);
    GPIOPinTypeSSI(SPI_GPIO_PORT, SPI_GPIO_PINS );
    SSIConfigSetExpClk(SPI_BASE, sysfreq, SPI_MOTO_MODE, SPI_MODE, SPI_SPEED, SPI_WORD_SIZE);
    SSIEnable(SPI_BASE);
    // should happen AFTER SPI init.
    SSIAdvModeSet(SPI_BASE, SSI_ADV_MODE_READ_WRITE);
    SSIAdvFrameHoldEnable(SPI_BASE);
    // clear out any 'junk' that may be in the SPI RX fifo.
    while(SSIDataGetNonBlocking(SPI_BASE, &dataRx));
#elif defined (STM32F0)
    rcc_periph_clock_enable(PORT_TO_RCC(ADS1248_PORT));
    rcc_periph_clock_enable(ADS1248_RRC_SPI_CLK);

    gpio_mode_setup(ADS1248_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ADS1248_SPI_AF_GPIOs);
    gpio_set_af(ADS1248_PORT, ADS1248_SPI_AF_GPIOS_F, ADS1248_SPI_AF_GPIOs);

    gpio_mode_setup(ADS1248_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, ADS1248_SPI_CS_PIN0);
//@@@    gpio_mode_setup(ADS1248_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ADS1248_SPI_CS_PIN0);
//    gpio_mode_setup(ADS1248_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ADS1248_SPI_CS_PIN1);

    spi_reset(ADS1248_SPI);
    spi_init_master(ADS1248_SPI,
                    ADS1248_SPI_DIVIDER,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_MSBFIRST);

    spi_set_data_size(ADS1248_SPI, SPI_CR2_DS_8BIT);
#endif
}

void InitDevice(void)
{
#if defined (__MSP430F5529__)
    P3SEL |= ADS1248_DIN + ADS1248_DOUT + ADS1248_SCLK;
    P1SEL &= ~(ADS1248_DRDY);
    // define initial states
    P6OUT |= ADS1248_START;
    P6OUT |= ADS1248_CS;
    P6OUT |= ADS1248_RESET;
    // define inputs
    P1DIR &= ~(ADS1248_DRDY);                   // DRDY is an input to the micro
    P1IES |= ADS1248_DRDY;                      // and should be used as an interrupt to retrieve data
    // define outputs
    P6DIR |= ADS1248_START;
    P6DIR |= ADS1248_CS;
    P6DIR |= ADS1248_RESET;
#elif defined (PART_TM4C1294NCPDT)
    GPIOPinTypeGPIOOutput(START_PORT, ADS1248_START);   // start
    GPIOPinWrite(START_PORT, ADS1248_START, 0);
    GPIOPinTypeGPIOOutput(RESET_PORT, ADS1248_RESET);   // reset
    GPIOPinTypeGPIOInput(DRDY_PORT, ADS1248_DRDY);      // DRDY
    GPIOIntTypeSet(DRDY_PORT, ADS1248_DRDY, GPIO_FALLING_EDGE);   // GPIO_HIGH_LEVEL ?
    GPIOPinWrite(RESET_PORT,ADS1248_RESET, ADS1248_RESET);
#elif defined (STM32F0)
    ADS1248AssertCS(1);
#endif
}

/*
 * ADS1248 Initial Configuration
 */
void InitConfig(void)
{
    //establish some startup register settings
    unsigned regArray[4];
    // Send SDATAC command
    ADS1248SendSDATAC();
    ADS1248WaitForDataReady(0);
    ADS1248SendSDATAC();
    //write the desired default register settings for the first 4 registers NOTE: values shown are the POR values as per datasheet
    regArray[0] = 0x01;
    regArray[1] = 0x00;
    regArray[2] = 0x00;
    regArray[3] = 0x00;
    ADS1248WriteSequence(ADS1248_0_MUX0, 4, regArray);
    return;
}

/*
 * DRDY Polling Function
 * Timeout = 0 is wait until DRDY goes low no matter how long it takes, otherwise wait the specified number of cycles
 */
int ADS1248WaitForDataReady(int Timeout)
{
    /* This function shows a method for polling DRDY instead of using as interrupt function
     * -- Note: this method is not used in the demo, but if we the method was switched to polling from the interrupt method,
     * the desired port is PORT2 on the MSP430 as this demo is configured.
     */
#if defined (__MSP430F5529__)
    if (Timeout > 0)
    {
        // wait for /DRDY = 1 to make sure it is high before we look for the transition low
        while (!(P1IN & ADS1248_DRDY) && (Timeout-- >= 0));
        // wait for /DRDY = 0
        while ( (P1IN & ADS1248_DRDY) && (Timeout-- >= 0));
        if (Timeout < 0)
            return ADS1248_ERROR;                   //ADS1248_TIMEOUT_WARNING;
    }
    else
    {
        // wait for /DRDY = 1
        while (!(P1IN & ADS1248_DRDY));
        // wait for /DRDY = 0
        while ( (P1IN & ADS1248_DRDY));
    }
#elif defined (PART_TM4C1294NCPDT)
    /* wait for nDRDY_REG to deassert as a known valid data */
    if (Timeout > 0)
    {
        // wait for /DRDY = 1 to make sure it is high before we look for the transition low
        while (!(nDRDY_REG) && (Timeout-- >= 0));
        // wait for /DRDY = 0
        while ( (nDRDY_REG) && (Timeout-- >= 0));
        if (Timeout < 0)
            return ADS1248_ERROR;                   //ADS1248_TIMEOUT_WARNING;
    }
    else
    {
        // wait for /DRDY = 1
        while (!(nDRDY_REG));
        // wait for /DRDY = 0
        while(nDRDY_REG);
        }
#elif defined (STM32F0)

    platform_raw_msg("Start data wait");

    gpio_mode_setup(ADS1248_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, ADS1248_SPI_DOUT);

    if (Timeout > 0)
    {
        // wait for /DRDY = 1 to make sure it is high before we look for the transition low
        while (!gpio_get(ADS1248_PORT, ADS1248_SPI_DOUT) && (Timeout-- >= 0));
        // wait for /DRDY = 0
        while ( gpio_get(ADS1248_PORT, ADS1248_SPI_DOUT) && (Timeout-- >= 0));
        if (Timeout < 0)
            return ADS1248_ERROR;                   //ADS1248_TIMEOUT_WARNING;
    }
    else
    {
        // wait for /DRDY = 1
        while (!gpio_get(ADS1248_PORT, ADS1248_SPI_DOUT));
        // wait for /DRDY = 0
        while(gpio_get(ADS1248_PORT, ADS1248_SPI_DOUT));
    }
    platform_raw_msg("Done data wait");

    gpio_mode_setup(ADS1248_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, ADS1248_SPI_DOUT);
    gpio_set_af(ADS1248_PORT, ADS1248_SPI_AF_GPIOS_F, ADS1248_SPI_DOUT);

#endif

    return ADS1248_NO_ERROR;
}

/*
 * Primary Low Level Functions
 */
void ADS1248AssertCS( int fAssert)
{
#if defined (__MSP430F5529__)
    // This example is using PORT6 for GPIO CS control, ADS1248_CS is defined in ads1248.h
    if (fAssert){               // fAssert=0 is CS low, fAssert=1 is CS high
        _delay_cycles(50);      // Must delay minimum of 7 tosc periods from last falling SCLK to rising CS
        P6OUT |=  (ADS1248_CS);
    } else
        P6OUT &=  ~(ADS1248_CS);
#elif defined (PART_TM4C1294NCPDT)
    if (fAssert){               // fAssert=0 is CS low, fAssert=1 is CS high
        SysCtrlDelay(100);      // Must delay minimum of 7 tosc periods from last falling SCLK to rising CS
        GPIOPinWrite(NCS_PORT, ADS1248_CS, 0xFF);
    } else
        GPIOPinWrite(NCS_PORT, ADS1248_CS, 0);
#elif defined (STM32F0)
    if (fAssert){
platform_raw_msg("CS high");
        stm32_delay();
        gpio_set(ADS1248_PORT, ADS1248_SPI_CS_PIN);
    } else
platform_raw_msg("CS low");
        gpio_clear(ADS1248_PORT, ADS1248_SPI_CS_PIN);
#endif
}


void ADS1248SendByte(unsigned char Byte)
{
#if defined (__MSP430F5529__)
    char dummy;
    dummy = UCB0RXBUF;
    while(!(UCB0IFG&UCTXIFG));          // Make sure nothing is already in the TX buffer
    UCB0TXBUF = Byte;                   // Send the passed Byte out the SPI bus
    while(!(UCB0IFG&UCRXIFG));          // Before returning wait until transmission is complete and clear the RX buffer
    dummy = UCB0RXBUF;
#elif defined (PART_TM4C1294NCPDT)
    char dummy;
    HWREG(SPI_BASE + SSI_O_DR) = Byte;                  // set up data for the next xmit
    while(!(HWREG(SPI_BASE + SSI_O_SR) & SSI_SR_RNE));  // wait for data to appear
    dummy = HWREG(SPI_BASE+SSI_O_DR);                   // grab that data
#elif defined (STM32F0)
    platform_raw_msg("send byte");

    stm32_spi_byte(Byte);
#endif
}

unsigned char ADS1248ReceiveByte(void)
{
    unsigned char Result = 0;
#if defined (__MSP430F5529__)

    while(!(UCB0IFG&UCTXIFG));          // Make sure nothing is currently transmitting
    UCB0TXBUF = ADS1248_CMD_NOP;        // Send out NOP to initiate SCLK
    while(!(UCB0IFG&UCRXIFG));          // Wait until all data is transmitted (received)
    Result = UCB0RXBUF;                 // Capture the receive buffer and return the Result
#elif defined (PART_TM4C1294NCPDT)
    // MUST MUST MUST purge junk from fifo!!!!
    while(SSIDataGetNonBlocking(SPI_BASE, &junk));
    HWREG(SPI_BASE + SSI_O_DR) = ADS1248_CMD_NOP;       // Send out NOP to initiate SCLK
    while(!(HWREG(SPI_BASE + SSI_O_SR) & SSI_SR_RNE));  // wait for data to appear
    Result = HWREG(SPI_BASE+SSI_O_DR);                  // grab that data
#elif defined (STM32F0)
    Result = stm32_spi_byte(0xFF);
#endif
    return Result;
}

/*
 * ADS1248 Higher Level Functions and Commands
 */
void ADS1248SendWakeup(void)
{
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_WAKEUP);
    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

void ADS1248SendSleep(void)
{
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_SLEEP);

    /*
     * CS must remain low for the device to remain asleep by command...otherwise bring START low by pin control
     */
    return;
}

void ADS1248SendSync(void)
{
    // assert CS to start transfer
    ADS1248AssertCS(0);
    platform_raw_msg("send sync");
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_SYNC);
    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

void ADS1248SendResetCommand(void)
{
    // assert CS to start transfer
	ADS1248AssertCS(0);

    platform_raw_msg("post cs");

    // send the command byte
    ADS1248SendByte(ADS1248_CMD_RESET);

    platform_raw_msg("post send byte.");

    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

long ADS1248ReadData(void)
{
    long Data;
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_RDATA);
    // get the conversion result
#ifdef ADS1148
    Data = ADS1248ReceiveByte();
    Data = (Data << 8) | ADS1248ReceiveByte();
    // sign extend data if the MSB is high (16 to 32 bit sign extension)
    if (Data & 0x8000)
        Data |= 0xffff0000;
#else
    Data = ADS1248ReceiveByte();
    Data = (Data << 8) | ADS1248ReceiveByte();
    Data = (Data << 8) | ADS1248ReceiveByte();
    // sign extend data if the MSB is high (24 to 32 bit sign extension)
    if (Data & 0x800000)
        Data |= 0xff000000;
#endif
    // de-assert CS
    ADS1248AssertCS(1);
    return Data;
}

void ADS1248ReadRegister(int StartAddress, int NumRegs, unsigned * pData)
{
    int i;
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_RREG | (StartAddress & 0x0f));
    ADS1248SendByte((NumRegs-1) & 0x0f);
    // get the register content
    for (i=0; i< NumRegs; i++)
    {
        *pData++ = ADS1248ReceiveByte();
    }
    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

void ADS1248WriteRegister(int StartAddress, int NumRegs, unsigned * pData)
{
    int i;
    // set the CS low
    ADS1248AssertCS(0);
    // send the command byte
    platform_raw_msg("here 2b");
    ADS1248SendByte(ADS1248_CMD_WREG | (StartAddress & 0x0f));

    platform_raw_msg("here2c");
    ADS1248SendByte((NumRegs-1) & 0x0f);
    // send the data bytes
    for (i=0; i < NumRegs; i++)
    {
        ADS1248SendByte(*pData++);
    }
    // set the CS back high
    platform_raw_msg("here2e");
    ADS1248AssertCS(1);
}

void ADS1248WriteSequence(int StartAddress, int NumRegs, unsigned * pData)
{
    int i;
    // set the CS low
    ADS1248AssertCS(0);
#if defined (__MSP430F5529__)
    char dummy;
    // send the command byte
    dummy = UCB0RXBUF;
    while(!(UCB0IFG&UCTXIFG));          // Make sure nothing is already in the TX buffer
    UCB0TXBUF = ADS1248_CMD_WREG | (StartAddress & 0x0f);
    while(!(UCB0IFG&UCRXIFG));
    dummy = UCB0RXBUF;
    UCB0TXBUF = (NumRegs-1) & 0x0f;
    while(!(UCB0IFG&UCRXIFG));
    dummy = UCB0RXBUF;
    // send the data bytes
    for (i=0; i < NumRegs; i++)
    {
        UCB0TXBUF = *pData++;
        while(!(UCB0IFG&UCRXIFG));
        dummy = UCB0RXBUF;
    }
#elif defined (PART_TM4C1294NCPDT)
    char dummy;
    HWREG(SPI_BASE + SSI_O_DR) = ADS1248_CMD_WREG | (StartAddress & 0x0f);  // set up data for the next xmit
    while(!(HWREG(SPI_BASE + SSI_O_SR) & SSI_SR_RNE));  // wait for data to appear
    dummy = HWREG(SPI_BASE+SSI_O_DR);                   // grab that data
    HWREG(SPI_BASE + SSI_O_DR) = (NumRegs-1) & 0x0f;    // set up data for the next xmit
    while(!(HWREG(SPI_BASE + SSI_O_SR) & SSI_SR_RNE));  // wait for data to appear
    dummy = HWREG(SPI_BASE+SSI_O_DR);                   // grab that data
    // send the data bytes
    for (i=0; i < NumRegs; i++)
    {
        HWREG(SPI_BASE + SSI_O_DR) = *pData++;
        while(!(HWREG(SPI_BASE + SSI_O_SR) & SSI_SR_RNE));  // wait for data to appear
        dummy = HWREG(SPI_BASE+SSI_O_DR);                   // grab that data
    }
#elif defined (STM32F0)

    uint8_t cmd = ADS1248_CMD_WREG | (StartAddress & 0x0f);	// set up data for the next xmit
    stm32_spi_byte(cmd);

    cmd = (NumRegs-1) & 0x0f;
    stm32_spi_byte(cmd);

    for (i=0; i < NumRegs; i++)
        stm32_spi_byte((unsigned char)pData[i]);
#endif
    // set the CS back high
    ADS1248AssertCS(1);
}

void ADS1248SendRDATAC(void)
{
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_RDATAC);
    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

void ADS1248SendSDATAC(void)
{
#if !defined (STM32F0)
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_SDATAC);
    // de-assert CS
    ADS1248AssertCS(1);
    return;
#endif
}

void ADS1248SendSYSOCAL(void)
{
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_SYSOCAL);
    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

void ADS1248SendSYSGCAL(void)
{
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_SYSGCAL);
    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

void ADS1248SendSELFOCAL(void)
{
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // send the command byte
    ADS1248SendByte(ADS1248_CMD_SELFOCAL);
    // de-assert CS
    ADS1248AssertCS(1);
    return;
}

/*
 * Register Set Value Commands
 *
 * These commands need to strip out old settings (AND) and add (OR) the new contents to the register
 */
int ADS1248SetBurnOutSource(int BurnOut)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
    Temp &= 0x3f;
    switch(BurnOut) {
        case 0:
            Temp |= ADS1248_BCS_OFF;
            break;
        case 1:
            Temp |= ADS1248_BCS_500nA;
            break;
        case 2:
            Temp |= ADS1248_BCS_2uA;
            break;
        case 3:
            Temp |= ADS1248_BCS_10uA;
            break;
        default:
            dError = ADS1248_ERROR;
            Temp |= ADS1248_BCS_OFF;
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_0_MUX0, 0x01, &Temp);
    return dError;
}

int ADS1248SetChannel(int vMux, int pMux)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
    if (pMux==1) {
        Temp &= 0xf8;
        switch(vMux) {
            case 0:
                Temp |= ADS1248_AINN0;
                break;
            case 1:
                Temp |= ADS1248_AINN1;
                break;
            case 2:
                Temp |= ADS1248_AINN2;
                break;
            case 3:
                Temp |= ADS1248_AINN3;
                break;
            case 4:
                Temp |= ADS1248_AINN4;
                break;
            case 5:
                Temp |= ADS1248_AINN5;
                break;
            case 6:
                Temp |= ADS1248_AINN6;
                break;
            case 7:
                Temp |= ADS1248_AINN7;
                break;
            default:
                Temp |= ADS1248_AINN0;
                dError = ADS1248_ERROR;
        }

    } else {
        Temp &= 0xc7;
        switch(vMux) {
            case 0:
                Temp |= ADS1248_AINP0;
                break;
            case 1:
                Temp |= ADS1248_AINP1;
                break;
            case 2:
                Temp |= ADS1248_AINP2;
                break;
            case 3:
                Temp |= ADS1248_AINP3;
                break;
            case 4:
                Temp |= ADS1248_AINP4;
                break;
            case 5:
                Temp |= ADS1248_AINP5;
                break;
            case 6:
                Temp |= ADS1248_AINP6;
                break;
            case 7:
                Temp |= ADS1248_AINP7;
                break;
            default:
                Temp |= ADS1248_AINP0;
                dError = ADS1248_ERROR;
        }
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_0_MUX0, 0x01, &Temp);
    return dError;
}

int ADS1248SetBias(unsigned char vBias)
{
    unsigned Temp;
    Temp = ADS1248_VBIAS_OFF;
    if (vBias & 0x80)
        Temp |=  ADS1248_VBIAS7;
    if (vBias & 0x40)
        Temp |=  ADS1248_VBIAS6;
    if (vBias & 0x20)
        Temp |=  ADS1248_VBIAS5;
    if (vBias & 0x10)
        Temp |=  ADS1248_VBIAS4;
    if (vBias & 0x08)
        Temp |=  ADS1248_VBIAS3;
    if (vBias & 0x04)
        Temp |=  ADS1248_VBIAS2;
    if (vBias & 0x02)
        Temp |=  ADS1248_VBIAS1;
    if (vBias & 0x01)
        Temp |=  ADS1248_VBIAS0;
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_1_VBIAS, 0x01, &Temp);
    return ADS1248_NO_ERROR;
}

// Relate to Mux1
int ADS1248SetIntRef(int sRef)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
    Temp &= 0x1f;
    switch(sRef) {
        case 0:
            Temp |= ADS1248_INT_VREF_OFF;
            break;
        case 1:
            Temp |= ADS1248_INT_VREF_ON;
            break;
        case 2:
        case 3:
            Temp |= ADS1248_INT_VREF_CONV;
            break;
        default:
            Temp |= ADS1248_INT_VREF_OFF;
            dError = ADS1248_ERROR;

    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_2_MUX1, 0x01, &Temp);
    return dError;
}

int ADS1248SetVoltageReference(int VoltageRef)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
    Temp &= 0xe7;
    switch(VoltageRef) {
        case 0:
            Temp |= ADS1248_REF0;
            break;
        case 1:
            Temp |= ADS1248_REF1;
            break;
        case 2:
            Temp |= ADS1248_INT;
            break;
        case 3:
            Temp |= ADS1248_INT_REF0;
            break;
        default:
            Temp |= ADS1248_REF0;
            dError = ADS1248_ERROR;
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_2_MUX1, 0x01, &Temp);
    return dError;
}

int ADS1248SetSystemMonitor(int Monitor)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
    Temp &= 0x78;
    switch(Monitor) {
        case 0:
            Temp |= ADS1248_MEAS_NORM;
            break;
        case 1:
            Temp |= ADS1248_MEAS_OFFSET;
            break;
        case 2:
            Temp |= ADS1248_MEAS_GAIN;
            break;
        case 3:
            Temp |= ADS1248_MEAS_TEMP;
            break;
        case 4:
            Temp |= ADS1248_MEAS_REF1;
            break;
        case 5:
            Temp |= ADS1248_MEAS_REF0;
            break;
        case 6:
            Temp |= ADS1248_MEAS_AVDD;
            break;
        case 7:
            Temp |= ADS1248_MEAS_DVDD;
            break;
        default:
            Temp |= ADS1248_MEAS_NORM;
            dError = ADS1248_ERROR;
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_2_MUX1, 0x01, &Temp);
    return dError;
}

// Relate to SYS0
int ADS1248SetGain(int Gain)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
    Temp &= 0x0f;
    switch(Gain) {
        case 0:
            Temp |= ADS1248_GAIN_1;
            break;
        case 1:
            Temp |= ADS1248_GAIN_2;
            break;
        case 2:
            Temp |= ADS1248_GAIN_4;
            break;
        case 3:
            Temp |= ADS1248_GAIN_8;
            break;
        case 4:
            Temp |= ADS1248_GAIN_16;
            break;
        case 5:
            Temp |= ADS1248_GAIN_32;
            break;
        case 6:
            Temp |= ADS1248_GAIN_64;
            break;
        case 7:
            Temp |= ADS1248_GAIN_128;
            break;
        default:
            Temp |= ADS1248_GAIN_1;
            dError = ADS1248_ERROR;
        }
        // write the register value containing the new value back to the ADS
        ADS1248WriteRegister(ADS1248_3_SYS0, 0x01, &Temp);
        return dError;
}

int ADS1248SetDataRate(int DataRate)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
    Temp &= 0x70;
    switch(DataRate) {
        case 0:
            Temp |= ADS1248_DR_5;
            break;
        case 1:
            Temp |= ADS1248_DR_10;
            break;
        case 2:
            Temp |= ADS1248_DR_20;
            break;
        case 3:
            Temp |= ADS1248_DR_40;
            break;
        case 4:
            Temp |= ADS1248_DR_80;
            break;
        case 5:
            Temp |= ADS1248_DR_160;
            break;
        case 6:
            Temp |= ADS1248_DR_320;
            break;
        case 7:
            Temp |= ADS1248_DR_640;
            break;
        case 8:
            Temp |= ADS1248_DR_1000;
            break;
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
            Temp |= ADS1248_DR_2000;
            break;
        default:
            Temp |= ADS1248_DR_5;
            dError = ADS1248_ERROR;
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_3_SYS0, 0x01, &Temp);
    return dError;
}

// Relate to OFC (3 registers)
int ADS1248SetOFC(long RegOffset)
{
    // find the pointer to the variable so we can write the value as bytes
    unsigned *cptr=(unsigned *)(&RegOffset);
    int i;

    for (i=0; i<3; i++)
    {
        // write the register value containing the new value back to the ADS
        ADS1248WriteRegister((ADS1248_4_OFC0 + i), 0x01, &cptr[i]);
    }
    return ADS1248_NO_ERROR;
}

// Relate to FSC (3 registers)
int ADS1248SetFSC(long RegGain)
{
    // find the pointer to the variable so we can write the value as bytes
    unsigned *cptr=(unsigned *)(&RegGain);
    int i;
    for (i=0; i<3; i++)
    {
        // write the register value containing the new value back to the ADS
        ADS1248WriteRegister((ADS1248_7_FSC0 + i), 0x01, &cptr[i]);
    }
    return ADS1248_NO_ERROR;
}

// Relate to IDAC0
int ADS1248SetDRDYMode(int DRDYMode)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
    Temp &= 0xf7;
    switch(DRDYMode) {
        case 0:
            Temp |= ADS1248_DRDY_OFF;
            break;
        case 1:
            Temp |= ADS1248_DRDY_ON;
            break;
        default:
            Temp |= ADS1248_DRDY_OFF;
            dError = ADS1248_ERROR;
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_10_IDAC0, 0x01, &Temp);
    return dError;
}

int ADS1248SetCurrentDACOutput(int CurrentOutput)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
    Temp &= 0xf8;
    switch(CurrentOutput) {
        case 0:
            Temp |= ADS1248_IDAC_OFF;
            break;
        case 1:
            Temp |= ADS1248_IDAC_50;
            break;
        case 2:
            Temp |= ADS1248_IDAC_100;
            break;
        case 3:
            Temp |= ADS1248_IDAC_250;
            break;
        case 4:
            Temp |= ADS1248_IDAC_500;
            break;
        case 5:
            Temp |= ADS1248_IDAC_750;
            break;
        case 6:
            Temp |= ADS1248_IDAC_1000;
            break;
        case 7:
            Temp |= ADS1248_IDAC_1500;
            break;
        default:
            Temp |= ADS1248_IDAC_OFF;
            dError = ADS1248_ERROR;
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_10_IDAC0, 0x01, &Temp);
    return dError;
}

// Relate to IDAC1
int ADS1248SetIDACRouting(int IDACroute, int IDACdir)       // IDACdir (0 = I1DIR, 1 = I2DIR)
{
    unsigned Temp;
    int dError = ADS1248_NO_ERROR;
    ADS1248ReadRegister(ADS1248_11_IDAC1, 0x01, &Temp);
    if (IDACdir>0){
        Temp &= 0xf0;
        switch(IDACroute) {
            case 0:
                Temp |= ADS1248_IDAC2_A0;
                break;
            case 1:
                Temp |= ADS1248_IDAC2_A1;
                break;
            case 2:
                Temp |= ADS1248_IDAC2_A2;
                break;
            case 3:
                Temp |= ADS1248_IDAC2_A3;
                break;
            case 4:
                Temp |= ADS1248_IDAC2_A4;
                break;
            case 5:
                Temp |= ADS1248_IDAC2_A5;
                break;
            case 6:
                Temp |= ADS1248_IDAC2_A6;
                break;
            case 7:
                Temp |= ADS1248_IDAC2_A7;
                break;
            case 8:
                Temp |= ADS1248_IDAC2_EXT1;
                break;
            case 9:
                Temp |= ADS1248_IDAC2_EXT2;
                break;
            case 10:
                Temp |= ADS1248_IDAC2_EXT1;
                break;
            case 11:
                Temp |= ADS1248_IDAC2_EXT2;
                break;
            case 12:
            case 13:
            case 14:
            case 15:
                Temp |= ADS1248_IDAC2_OFF;
                break;
            default:
                Temp |= ADS1248_IDAC2_OFF;
                dError = ADS1248_ERROR;
        }

    } else {
        Temp &= 0x0f;
        switch(IDACroute) {
            case 0:
                Temp |= ADS1248_IDAC1_A0;
                break;
            case 1:
                Temp |= ADS1248_IDAC1_A1;
                break;
            case 2:
                Temp |= ADS1248_IDAC1_A2;
                break;
            case 3:
                Temp |= ADS1248_IDAC1_A3;
                break;
            case 4:
                Temp |= ADS1248_IDAC1_A4;
                break;
            case 5:
                Temp |= ADS1248_IDAC1_A5;
                break;
            case 6:
                Temp |= ADS1248_IDAC1_A6;
                break;
            case 7:
                Temp |= ADS1248_IDAC1_A7;
                break;
            case 8:
                Temp |= ADS1248_IDAC1_EXT1;
                break;
            case 9:
                Temp |= ADS1248_IDAC1_EXT2;
                break;
            case 10:
                Temp |= ADS1248_IDAC1_EXT1;
                break;
            case 11:
                Temp |= ADS1248_IDAC1_EXT2;
                break;
            case 12:
            case 13:
            case 14:
            case 15:
                Temp |= ADS1248_IDAC1_OFF;
                break;
            default:
                Temp |= ADS1248_IDAC1_OFF;
                dError = ADS1248_ERROR;
        }
    }
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_11_IDAC1, 0x01, &Temp);
    return dError;
}

// Relate to GPIOCFG
int ADS1248SetGPIOConfig(unsigned char cdata)
{
    unsigned Temp;
    Temp = 0x00;
    if (cdata & 0x80)
        Temp |=  ADS1248_GPIO_7;
    if (cdata & 0x40)
        Temp |=  ADS1248_GPIO_6;
    if (cdata & 0x20)
        Temp |=  ADS1248_GPIO_5;
    if (cdata & 0x10)
        Temp |=  ADS1248_GPIO_4;
    if (cdata & 0x08)
        Temp |=  ADS1248_GPIO_3;
    if (cdata & 0x04)
        Temp |=  ADS1248_GPIO_2;
    if (cdata & 0x02)
        Temp |=  ADS1248_GPIO_1;
    if (cdata & 0x01)
        Temp |=  ADS1248_GPIO_0;
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_12_GPIOCFG, 0x01, &Temp);
    return ADS1248_NO_ERROR;
}

// Relate to GPIODIR
int ADS1248SetGPIODir(unsigned char cdata)
{
    unsigned Temp;
    Temp = 0x00;
    if (cdata & 0x80)
        Temp |=  ADS1248_IO_7;
    if (cdata & 0x40)
        Temp |=  ADS1248_IO_6;
    if (cdata & 0x20)
        Temp |=  ADS1248_IO_5;
    if (cdata & 0x10)
        Temp |=  ADS1248_IO_4;
    if (cdata & 0x08)
        Temp |=  ADS1248_IO_3;
    if (cdata & 0x04)
        Temp |=  ADS1248_IO_2;
    if (cdata & 0x02)
        Temp |=  ADS1248_IO_1;
    if (cdata & 0x01)
        Temp |=  ADS1248_IO_0;
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_13_GPIODIR, 0x01, &Temp);
    return ADS1248_NO_ERROR;
}

// Relate to GPIODAT
int ADS1248SetGPIO(unsigned char cdata)
{
    unsigned Temp;
    Temp = 0x00;
    if (cdata & 0x80)
        Temp |=  ADS1248_OUT_7;
    if (cdata & 0x40)
        Temp |=  ADS1248_OUT_6;
    if (cdata & 0x20)
        Temp |=  ADS1248_OUT_5;
    if (cdata & 0x10)
        Temp |=  ADS1248_OUT_4;
    if (cdata & 0x08)
        Temp |=  ADS1248_OUT_3;
    if (cdata & 0x04)
        Temp |=  ADS1248_OUT_2;
    if (cdata & 0x02)
        Temp |=  ADS1248_OUT_1;
    if (cdata & 0x01)
        Temp |=  ADS1248_OUT_0;
    // write the register value containing the new value back to the ADS
    ADS1248WriteRegister(ADS1248_14_GPIODAT, 0x01, &Temp);
    return ADS1248_NO_ERROR;
}

/* Register Get Value Commands */
// Relate to MUX0
int ADS1248GetBurnOutSource(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
    return ((Temp >> 6) & 0x03);
}

int ADS1248GetChannel(int cMux)         // cMux = 0, AINP; cMux = 1, AINN
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_0_MUX0, 0x01, &Temp);
    if (cMux==0)
        return ((Temp >> 3) & 0x07);
    else
        return (Temp  & 0x07);
}

// Relate to VBIAS
unsigned char ADS1248GetBias(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_1_VBIAS, 0x01, &Temp);
    return (Temp & 0xff);
}

//Relate to MUX1
int ADS1248GetCLKSTAT(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
    return ((Temp >> 7) & 0x01);
}

int ADS1248GetIntRef(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
    return ((Temp >> 5) & 0x03);
}

int ADS1248GetVoltageReference(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
    return ((Temp >> 3) & 0x03);
}

int ADS1248GetSystemMonitor(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_2_MUX1, 0x01, &Temp);
    return (Temp & 0x07);
}

// Relate to SYS0
int ADS1248GetGain(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
    return ((Temp >> 4) & 0x07);
}

int ADS1248GetDataRate(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_3_SYS0, 0x01, &Temp);
    return (Temp & 0x0f);
}

// Relate to OFC (3 registers)
long ADS1248GetOFC(void)
{
    long rData=0;
    unsigned rValue=0;
    unsigned regArray[3];
    int i;
    //write the desired default register settings for the first 4 registers NOTE: values shown are the POR values as per datasheet
    regArray[0] = 0x00;
    regArray[1] = 0x00;
    regArray[2] = 0x00;
    for (i=0; i<3; i++)
    {
        // read the register value for the OFC
        ADS1248ReadRegister((ADS1248_4_OFC0 + i), 0x01, &rValue);
        regArray[i] = rValue;
    }
    rData = regArray[2];
    rData = (rData<<8) | regArray[1];
    rData = (rData<<8) | regArray[0];
    return rData;
}

// Relate to FSC (3 registers)
long ADS1248GetFSC(void)
{
    long rData=0;
    unsigned rValue=0;
    unsigned regArray[3];
    int i;
    //write the desired default register settings for the first 4 registers NOTE: values shown are the POR values as per datasheet
    regArray[0] = 0x00;
    regArray[1] = 0x00;
    regArray[2] = 0x00;
    for (i=0; i<3; i++)
    {
        // read the register value for the OFC
        ADS1248ReadRegister((ADS1248_7_FSC0 + i), 0x01, &rValue);
        regArray[i] = rValue;
    }
    rData = regArray[2];
    rData = (rData<<8) | regArray[1];
    rData = (rData<<8) | regArray[0];
    return rData;
}

// Relate to IDAC0
int ADS1248GetID(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
    return ((Temp>>4) & 0x0f);
}

int ADS1248GetDRDYMode(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
    return ((Temp>>3) & 0x01);
}

int ADS1248GetCurrentDACOutput(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_10_IDAC0, 0x01, &Temp);
    return (Temp & 0x07);
}

// Relate to IDAC1
int ADS1248GetIDACRouting(int WhichOne)         // IDACRoute (0 = I1DIR, 1 = I2DIR)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_11_IDAC1, 0x01, &Temp);
    if (WhichOne==0)
        return ((Temp>>4) & 0x0f);
    else
        return (Temp & 0x0f);
}

// Relate to GPIOCFG
unsigned char ADS1248GetGPIOConfig(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_12_GPIOCFG, 0x01, &Temp);
    return (Temp & 0xff);
}

// Relate to GPIODIR
unsigned char ADS1248GetGPIODir(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_13_GPIODIR, 0x01, &Temp);
    return (Temp & 0xff);
}

// Relate to GPIODAT
unsigned char ADS1248GetGPIO(void)
{
    unsigned Temp;
    ADS1248ReadRegister(ADS1248_14_GPIODAT, 0x01, &Temp);
    return (Temp & 0xff);
}

/* Miscellaneous Commands */
long ADS1248RDATACRead(void)        // reads data directly based on RDATAC mode (writes NOP) and 32 SCLKs
{
    long Data;
    // assert CS to start transfer
    ADS1248AssertCS(0);
    // get the conversion result
#ifdef ADS1148
    Data = ADS1248ReceiveByte();
    Data = (Data << 8) | ADS1248ReceiveByte();
    // sign extend data if the MSB is high (16 to 32 bit sign extension)
    if (Data & 0x8000)
        Data |= 0xffff0000;
#else
    Data = ADS1248ReceiveByte();
    Data = (Data << 8) | ADS1248ReceiveByte();
    Data = (Data << 8) | ADS1248ReceiveByte();
    // sign extend data if the MSB is high (24 to 32 bit sign extension)
    if (Data & 0x800000)
        Data |= 0xff000000;
#endif
    // de-assert CS
    ADS1248AssertCS(1);
    return Data;
}

/* Hardware Control Functions for Device Pin Control */
// Possible Need for Reset, Start (power down) (0-low, 1-high, 2-pulse)
int ADS1248SetStart(int nStart)
{
    /*
     * Code can be added here to set high or low the state of the pin for controlling the START pin
     * which will differ depending on controller used and port pin assigned
     */

#if defined (__MSP430F5529__)
    // This example is using PORT6 for GPIO Start control, ADS1248_START is defined in ads1248.h
    if (nStart)             // nStart=0 is START low, nStart=1 is START high
        P6OUT |=  (ADS1248_START);
    else
        P6OUT &=  ~(ADS1248_START);
#elif defined (PART_TM4C1294NCPDT)
    if (nStart)             // nStart=0 is START low, nStart=1 is START high
        GPIOPinWrite(START_PORT, ADS1248_START, 0xFF);
    else
        GPIOPinWrite(START_PORT, ADS1248_START, 0);
#elif defined (STM32F0)
    nStart = nStart;
#endif
    return ADS1248_NO_ERROR;
}

int ADS1248SetReset(int nReset)
{
    /*
     * Code can be added here to set high or low the state of the pin for controlling the RESET pin
     * which will differ depending on controller used and port pin assigned
     */
#if defined (__MSP430F5529__)
    // This example is using PORT6 for GPIO Reset Control, ADS1248_RESET is defined in ads1248.h
    if (nReset)             // nReset=0 is RESET low, nReset=1 is RESET high
        P6OUT |=  (ADS1248_RESET);
    else
        P6OUT &=  ~(ADS1248_RESET);
#elif defined (PART_TM4C1294NCPDT)
    if (nReset)             // nReset=0 is RESET low, nReset=1 is RESET high
        GPIOPinWrite(RESET_PORT, ADS1248_RESET, 0xFF);
    else
        GPIOPinWrite(RESET_PORT, ADS1248_RESET, 0);
#elif defined (STM32F0)
    nReset = nReset;
#endif
    return ADS1248_NO_ERROR;
}

