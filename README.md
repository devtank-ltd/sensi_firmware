Devtank IO Board
================

This is a firmware for a STM32F072RB as a generic IO board.

It has:

* 4 UARTs
    - 3 via USB
        - 1 at 115200 for control
        - 1 at 115200 to/from STM32 UART4
        - 1 at 9600 to/from STM32 UART3
    - 1 to/from STM32 UART2 for debug.
* 9 Input GPIOs
* 16 Output GPIOs
* 2 Pulses per second counters
* 10 ADCs sampled at 5k getting min,max and average.
