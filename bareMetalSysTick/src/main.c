/*
 * ----------------------------------------------------------------------------
 * Project: Bare-Metal SysTick Demo on STM32L4S5VI (B-L4S5I-IOT01A)
 * Author:  Dimitris Savvaris
 * License: MIT License (See below)
 * ----------------------------------------------------------------------------
 * Description:
 * This is a minimal bare-metal implementation of a SysTick timer demo for the
 * STM32L4S5VI microcontroller on the B-L4S5I-IOT01A development board.
 *
 * - Configures GPIOA Pin 5 (PA5-LED1) and GPIOB Pin 14 (PB14-LED2) as outputs.
 * - Uses SysTick timer to generate precise 1ms interrupts.
 * - Toggles LED1 every 500ms and LED2 every 1000ms using the SysTick timer.
 * - Demonstrates SysTick configuration, interrupt handling, and precise timing.
 *
 * Features:
 * - SysTick configured for 1ms interrupt at 4MHz system clock
 * - Non-blocking LED control using timer-based state machine
 * - Proper interrupt vector table setup
 * - System tick counter for timekeeping
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
 * Constants and Definitions
 */

// GPIO Register definitions
#define RCC_AHB2ENR     (*(volatile uint32_t*)(RCC_BASE + 0x4C))
#define GPIOA_MODER     (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_ODR       (*(volatile uint32_t*)(GPIOA_BASE + 0x14))
#define GPIOB_MODER     (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR       (*(volatile uint32_t*)(GPIOB_BASE + 0x14))

// SysTick Register definitions (Cortex-M4 System Control Block)
#define SYSTICK_BASE    0xE000E010UL
#define SYSTICK_CSR     (*(volatile uint32_t*)(SYSTICK_BASE + 0x00))  // Control and Status
#define SYSTICK_RVR     (*(volatile uint32_t*)(SYSTICK_BASE + 0x04))  // Reload Value
#define SYSTICK_CVR     (*(volatile uint32_t*)(SYSTICK_BASE + 0x08))  // Current Value

// SysTick Control and Status Register bits
#define SYSTICK_CSR_ENABLE      (1U << 0)   // Enable SysTick timer
#define SYSTICK_CSR_TICKINT     (1U << 1)   // Enable SysTick interrupt
#define SYSTICK_CSR_CLKSOURCE   (1U << 2)   // Use processor clock
#define SYSTICK_CSR_COUNTFLAG   (1U << 16)  // Count flag

// GPIO bit definitions for PA5 (LED1)
#define RCC_AHB2ENR_GPIOAEN     (1U << 0)
#define GPIOA5_MODER_OUTPUT     (1U << 10)
#define GPIOA5_MODER_MASK       (3U << 10)
#define GPIOA5_ODR              (1U << 5)

// GPIO bit definitions for PB14 (LED2)
#define RCC_AHB2ENR_GPIOBEN     (1U << 1)
#define GPIOB14_MODER_OUTPUT    (1U << 28)
#define GPIOB14_MODER_MASK      (3U << 28)
#define GPIOB14_ODR             (1U << 14)

// Timing constants
#define SYSTEM_CLOCK_HZ         4000000UL   // 4MHz MSI clock (default)
#define SYSTICK_FREQ_HZ         1000UL      // 1kHz = 1ms period
#define SYSTICK_RELOAD_VALUE    ((SYSTEM_CLOCK_HZ / SYSTICK_FREQ_HZ) - 1)

#define LED1_TOGGLE_PERIOD_MS   500         // Toggle LED1 every 500ms
#define LED2_TOGGLE_PERIOD_MS   1000        // Toggle LED2 every 1000ms

/* ----------------------------------------------------------------------------
 * Global Variables
 */
static volatile uint32_t system_tick_counter = 0;
static volatile uint32_t led1_last_toggle = 0;
static volatile uint32_t led2_last_toggle = 0;

/* ----------------------------------------------------------------------------
 * Function Prototypes
 */
static void gpio_init(void);
static void systick_init(void);
static void led_update(void);

/* ----------------------------------------------------------------------------
 * SysTick Interrupt Handler
 */
void SysTick_Handler(void)
{
    // Increment the system tick counter
    system_tick_counter++;

    // Update LED states based on timing
    led_update();
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize GPIO pins for LEDs.
 * @details Configures PA5 (LED1) and PB14 (LED2) as output pins.
 * ----------------------------------------------------------------------------
 */
static void gpio_init(void)
{
    // Enable clocks for GPIOA and GPIOB
    RCC_AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Configure PA5 as output
    GPIOA_MODER &= ~GPIOA5_MODER_MASK;      // Clear mode bits
    GPIOA_MODER |= GPIOA5_MODER_OUTPUT;     // Set as output

    // Configure PB14 as output
    GPIOB_MODER &= ~GPIOB14_MODER_MASK;     // Clear mode bits
    GPIOB_MODER |= GPIOB14_MODER_OUTPUT;    // Set as output

    // Turn off both LEDs initially
    GPIOA_ODR &= ~GPIOA5_ODR;
    GPIOB_ODR &= ~GPIOB14_ODR;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize SysTick timer.
 * @details Configures SysTick to generate 1ms interrupts using the processor clock.
 * ----------------------------------------------------------------------------
 */
static void systick_init(void)
{
    // Disable SysTick during configuration
    SYSTICK_CSR = 0;

    // Set reload value for 1ms interrupt (assuming 4MHz clock)
    SYSTICK_RVR = SYSTICK_RELOAD_VALUE;

    // Clear current value
    SYSTICK_CVR = 0;

    // Configure and enable SysTick
    SYSTICK_CSR = SYSTICK_CSR_ENABLE |      // Enable timer
                  SYSTICK_CSR_TICKINT |     // Enable interrupt
                  SYSTICK_CSR_CLKSOURCE;    // Use processor clock
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Update LED states based on timing.
 * @details Called from SysTick interrupt to toggle LEDs at different intervals.
 * ----------------------------------------------------------------------------
 */
static void led_update(void)
{
    // Toggle LED1 every 500ms
    if ((system_tick_counter - led1_last_toggle) >= LED1_TOGGLE_PERIOD_MS)
    {
        GPIOA_ODR ^= GPIOA5_ODR;            // Toggle LED1
        led1_last_toggle = system_tick_counter;
    }

    // Toggle LED2 every 1000ms
    if ((system_tick_counter - led2_last_toggle) >= LED2_TOGGLE_PERIOD_MS)
    {
        GPIOB_ODR ^= GPIOB14_ODR;           // Toggle LED2
        led2_last_toggle = system_tick_counter;
    }
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Get current system tick count.
 * @details Returns the current value of the system tick counter.
 * @return  Current tick count in milliseconds.
 * ----------------------------------------------------------------------------
 */
uint32_t get_system_tick(void)
{
    return system_tick_counter;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Main function for SysTick Demo on STM32L4S5VI.
 * @details Initializes GPIO and SysTick timer, then enters an infinite loop.
 *          The actual LED toggling is handled by the SysTick interrupt.
 *
 * @return  This function never returns (infinite loop).
 *
 * @pre     System should be using the default MSI clock (4MHz).
 * @post    LED1 blinks every 500ms, LED2 blinks every 1000ms.
 * ----------------------------------------------------------------------------
 */
int main(void)
{
    // Initialize peripherals
    gpio_init();
    systick_init();

    // Main loop - all timing is handled by SysTick interrupt
    while (1)
    {
        // Main loop can perform other tasks here
        // LEDs are controlled automatically by SysTick interrupt

        // Optional: Add a simple heartbeat or other non-blocking operations
        __asm("wfi");  // Wait for interrupt (low power mode)
    }

    return 0;  // Never reached
}
