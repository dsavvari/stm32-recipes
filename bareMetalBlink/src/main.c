/*
 * ----------------------------------------------------------------------------
 * Project: Bare-Metal Blink LED on STM32L4S5VI (B-L4S5I-IOT01A)
 * Author:  Dimitris Savvaris
 * License: MIT License (See below)
 * ----------------------------------------------------------------------------
 * Description:
 * This is a minimal bare-metal implementation of a "Blink LED" program for the
 * STM32L4S5VI microcontroller on the B-L4S5I-IOT01A development board.
 *
 * - Configures GPIOA Pin 5 (PA5-LED1) as an output.
 * - Toggles the LED in an infinite loop with a simple delay.
 * - Uses direct register access without any HAL or CMSIS libraries.
 * - Demonstrates basic clock setup and GPIO control.
 *
 * ----------------------------------------------------------------------------
 * MIT License
 *
 * Copyright (c) 2025 Dimitris Savvaris
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ----------------------------------------------------------------------------
 */

#include <stdint.h>
#include "stm32l4s5xx.h"

/* ----------------------------------------------------------------------------
 * Constants
 *
 * Register addresses - see STM32L4S5 RM0432
 */
#define RCC_AHB2ENR                                                            \
    (*(volatile uint32_t*)(RCC_BASE + 0x4C)) // Enable GPIO clocks

#define GPIOA_MODER                                                            \
    (*(volatile uint32_t*)(GPIOA_BASE + 0x00)) // GPIO mode register
#define GPIOA_ODR                                                              \
    (*(volatile uint32_t*)(GPIOA_BASE + 0x14)) // GPIO output data register

#define GPIOB_MODER                                                            \
    (*(volatile uint32_t*)(GPIOB_BASE + 0x00)) // GPIO mode register
#define GPIOB_ODR                                                              \
    (*(volatile uint32_t*)(GPIOB_BASE + 0x14)) // GPIO output data register

/*PA5 GPIO connects to on-board LED-1*/
/* Bit positions */
#define RCC_AHB2ENR_GPIOAEN (1U << 0)  // Enable GPIOA clock
#define GPIOA5_MODER_OUTPUT (1U << 10) // Set PA5 as output
#define GPIOA5_MODER_MASK (3U << 10)   // Mask for PA5 mode
#define GPIOA5_ODR (1U << 5)           // Control PA5 output state


/*PB14 GPIO connects to on-board LED-2*/
/* Bit positions */
#define RCC_AHB2ENR_GPIOBEN (1U << 1)  // Enable GPIOB clock
#define GPIOB14_MODER_OUTPUT (1U << 28) // Set PB14 as output
#define GPIOB14_MODER_MASK (3U << 28)   // Mask for PB14 mode
#define GPIOB14_ODR (1U << 14)          // Control PB14 output state



/* Delay counters */
#define DELAY_COUNTER (100000U)


/**
 * ----------------------------------------------------------------------------
 * @brief   Simple delay loop.
 * @details Implements a blocking delay by running an empty loop.
 *
 * @param[in] count  Number of iterations for the delay.
 *
 * @note     This is a basic delay and is not precise.
 * ----------------------------------------------------------------------------
 **/
static void delay(volatile uint32_t count)
{
    while (count--)
    {
        /* No operation to prevent any loop optimisation */
        __asm("nop");
    }
}
/**
 * ----------------------------------------------------------------------------
 * @brief   Main function for Blink LED on STM32L4S5VI.
 * @details Initializes GPIOA, sets PA5 as an output,
 *          and toggles the LED in an infinite loop.
 *
 * @return  This function never returns (infinite loop).
 *
 * @pre     System clock should be correctly set up.
 * @post    LED should blink continuously with a fixed delay.
 * ----------------------------------------------------------------------------
 **/
int main(void)
{

    // 1. Enable clock for GPIOA
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // 2. Enable clock for GPIOB
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // 3. Set PA5 as an output (MODER register: bits 11:10 = 0b01)
    GPIOA_MODER &= ~GPIOA5_MODER_MASK;
    GPIOA_MODER |= GPIOA5_MODER_OUTPUT;

    // 4. Set PB14 as an output (MODER register: bits 29:28 = 0b01)
    GPIOB_MODER &= ~GPIOB14_MODER_MASK;
    GPIOB_MODER |= GPIOB14_MODER_OUTPUT;

    unsigned int i = 0;

    // 5. Main loop: Toggle LED forever
    while (1)
    {
        GPIOA_ODR ^= GPIOA5_ODR;

        if ( i%60 == 0)
        {
            // Toggle PB14 every 60 iterations
            GPIOB_ODR ^= GPIOB14_ODR;
        }

        delay(DELAY_COUNTER);
        i++;
    }

    return 0;
}