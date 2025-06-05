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
 * - Configures GPIOA Pin 5 (PA5) as an output.
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
#include "stm32l4xx.h"

#if 0
/* ----------------------------------------------------------------------------
 * Constants
 *
 * Register addresses - see STM32L4S5 RM0432
 */
#define RCC_BASE 0x40021000U
#define GPIOA_BASE 0x48000000U // GPIOA base address
#define GPIOB_BASE 0x48000400U // GPIOB base address

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

#endif



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
#if 0 
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
#else


void init_gpio_pa0_pwm(void) {
   // Enable GPIOA and TIM2 peripheral clocks
   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
   RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

   // Set PA0 to Alternate Function mode (MODER = 10)
   GPIOA->MODER &= ~(0b11 << (0 * 2)); // Clear bits for PA0
   GPIOA->MODER |=  (0b10 << (0 * 2)); // Set to AF mode

   // Set Alternate Function 1 (TIM2_CH1) for PA0
   GPIOA->AFR[0] &= ~(0xF << (0 * 4)); // Clear AF bits
   GPIOA->AFR[0] |=  (0x1 << (0 * 4)); // AF1 = TIM2_CH1
}

void init_tim2_pwm_20hz_1ms_pulse(void) {
   // Set prescaler to divide 80 MHz to 10 kHz (1 tick = 0.1 ms)
   TIM2->PSC = 7999;

   // Set ARR for 50 ms period: 10 kHz / 20 Hz = 500 ticks
   TIM2->ARR = 499;

   // Set pulse width = 1 ms = 10 ticks (2% duty cycle)
   TIM2->CCR1 = 10;

   // Set PWM Mode 1 on CH1: active until match
   TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
   TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1
   TIM2->CCMR1 |= TIM_CCMR1_OC1PE;           // Output compare preload enable

   // Enable output for channel 1
   TIM2->CCER |= TIM_CCER_CC1E;

   // Enable auto-reload preload
   TIM2->CR1 |= TIM_CR1_ARPE;

   // Enable counter
   TIM2->CR1 |= TIM_CR1_CEN;
}

int main(void) {
   init_gpio_pa0_pwm();
   init_tim2_pwm_20hz_1ms_pulse();

   while (1) {
       // Nothing to do in main loop; PWM runs on hardware
   }
}


#endif // 0