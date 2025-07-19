/*
 * ----------------------------------------------------------------------------
 * Project: Bare-Metal SysTick Demo on STM32L4S5VI (B-L4S5I-IOT01A)
 * Author:  Dimitris Savvaris
 * License: MIT License (See below)
 * ----------------------------------------------------------------------------
 * Description:
 *
 * What is SysTick Timer?
 * SysTick (System Tick Timer) is a 24-bit down-counting timer that's part of the
 * ARM Cortex-M core (not an STM32 peripheral). It provides a consistent timing
 * reference for operating systems and applications.
 *
 * Key Characteristics:
 * - 24-bit counter: Can count from 0 to 16,777,215 (0xFFFFFF)
 * - Down-counting: Counts from reload value down to 0
 * - Built into Cortex-M core: Available on all ARM Cortex-M processors
 * - High priority: Uses exception priority -1 (higher than all interrupts)
 * - Clock sources: Processor clock (HCLK) or external reference (HCLK/8)
 * - Hardware-based: Provides precise timing independent of software execution
 *
 * This is a minimal bare-metal implementation of a SysTick timer demo for the
 * STM32L4S5VI microcontroller on the B-L4S5I-IOT01A development board.
 *
 * - Configures GPIO pins: PA5 (LED1), PB14 (LED2), PA0 (External LED - ARD-D1), and PC13 (Button).
 * - Uses SysTick timer to generate precise 1ms interrupts.
 * - All three LEDs (LED1, LED2, and External LED) blink at user-selectable frequencies (button-controlled).
 * - External LED on PA0 (ARD-D1) provides variable frequency square wave for oscilloscope measurements.
 * - Button on PC13 cycles through blink periods: 1000ms → 500ms → 300ms → 250ms → 200ms → repeat.
 * - Demonstrates SysTick configuration, interrupt handling, button debouncing, and precise timing.
 *
 * Features:
 * - SysTick configured for 1ms interrupt at 4MHz system clock
 * - Non-blocking LED control using timer-based state machine
 * - User button for frequency selection with edge detection
 * - Multiple blink frequencies from 1Hz down to 5Hz (200ms minimum period)
 * - All three LEDs blink synchronously at the selected frequency for easy oscilloscope measurements
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

// GPIO Register definitions - Use CMSIS-style access
// No need to redefine - will use RCC->, GPIOA->, GPIOB->, GPIOC-> directly

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
#define GPIOA5_MODER_OUTPUT     (1U << 10)
#define GPIOA5_MODER_MASK       (3U << 10)
#define GPIOA5_ODR              (1U << 5)

// GPIO bit definitions for PB14 (LED2)
#define GPIOB14_MODER_OUTPUT    (1U << 28)
#define GPIOB14_MODER_MASK      (3U << 28)
#define GPIOB14_ODR             (1U << 14)

// GPIO bit definitions for PA0 (External LED - ARD-D1)
#define GPIOA0_MODER_OUTPUT     (1U << 0)
#define GPIOA0_MODER_MASK       (3U << 0)
#define GPIOA0_ODR              (1U << 0)

// GPIO bit definitions for PC13 (User Button)
#define GPIOC13_MODER_INPUT     (0U << 26)
#define GPIOC13_MODER_MASK      (3U << 26)
#define GPIOC13_PUPDR_PULLUP    (1U << 26)
#define GPIOC13_PUPDR_MASK      (3U << 26)
#define GPIOC13_IDR             (1U << 13)

// Timing constants
#define SYSTEM_CLOCK_HZ         4000000UL   // 4MHz MSI clock (default)
#define SYSTICK_FREQ_HZ         1000UL      // 1kHz = 1ms period
#define SYSTICK_RELOAD_VALUE    ((SYSTEM_CLOCK_HZ / SYSTICK_FREQ_HZ) - 1)

// LED timing periods (in milliseconds)
#define NUM_BLINK_PERIODS       5           // Number of different blink periods
#define BLINK_PERIODS_MS        {1000, 500, 300, 250, 200}  // Available periods
#define EXT_LED_TOGGLE_PERIOD_MS 1000       // External LED always at 1Hz for scope reference

/* ----------------------------------------------------------------------------
 * Global Variables
 */
static volatile uint32_t system_tick_counter = 0;
static volatile uint32_t led1_last_toggle = 0;
static volatile uint32_t led2_last_toggle = 0;
static volatile uint32_t ext_led_last_toggle = 0;

// Button and frequency control variables
static uint8_t current_period_index = 0;    // Index into blink_periods array
static const uint32_t blink_periods[NUM_BLINK_PERIODS] = BLINK_PERIODS_MS;

/* ----------------------------------------------------------------------------
 * Function Prototypes
 */
static void gpio_init(void);
static void button_init(void);
static void systick_init(void);
static void led_update(void);
static int is_button_pressed(void);

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
 * @details Configures PA5 (LED1), PB14 (LED2), and PA0 (External LED) as output pins.
 *          All three LEDs will blink synchronously at the user-selected frequency.
 * ----------------------------------------------------------------------------
 */
static void gpio_init(void)
{
    // Enable clocks for GPIOA and GPIOB
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Configure PA5 as output
    GPIOA->MODER &= ~GPIOA5_MODER_MASK;      // Clear mode bits
    GPIOA->MODER |= GPIOA5_MODER_OUTPUT;     // Set as output

    // Configure PA0 as output (External LED - ARD-D1)
    GPIOA->MODER &= ~GPIOA0_MODER_MASK;      // Clear mode bits
    GPIOA->MODER |= GPIOA0_MODER_OUTPUT;     // Set as output

    // Configure PB14 as output
    GPIOB->MODER &= ~GPIOB14_MODER_MASK;     // Clear mode bits
    GPIOB->MODER |= GPIOB14_MODER_OUTPUT;    // Set as output

    // Turn off all LEDs initially
    GPIOA->ODR &= ~(GPIOA5_ODR | GPIOA0_ODR);
    GPIOB->ODR &= ~GPIOB14_ODR;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize user button (PC13) as input with pull-up.
 * @details Enables GPIOC clock, sets PC13 as input, and enables pull-up.
 * ----------------------------------------------------------------------------
 */
static void button_init(void)
{
    // Enable clock for GPIOC
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // Set PC13 as input (default state)
    GPIOC->MODER &= ~GPIOC13_MODER_MASK;     // Clear mode bits (input mode)

    // Enable pull-up on PC13
    GPIOC->PUPDR &= ~GPIOC13_PUPDR_MASK;     // Clear pull-up/down bits
    GPIOC->PUPDR |= GPIOC13_PUPDR_PULLUP;    // Set pull-up
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
 * @details Called from SysTick interrupt to toggle all LEDs at the same interval.
 *          LED1, LED2, and External LED all use the current blink period selected by button press.
 *          All three LEDs blink synchronously at the user-selected frequency.
 * ----------------------------------------------------------------------------
 */
static void led_update(void)
{
    uint32_t current_period = blink_periods[current_period_index];

    // Toggle LED1 using current blink period
    if ((system_tick_counter - led1_last_toggle) >= current_period)
    {
        GPIOA->ODR ^= GPIOA5_ODR;            // Toggle LED1
        led1_last_toggle = system_tick_counter;
    }

    // Toggle LED2 using current blink period
    if ((system_tick_counter - led2_last_toggle) >= current_period)
    {
        GPIOB->ODR ^= GPIOB14_ODR;           // Toggle LED2
        led2_last_toggle = system_tick_counter;
    }

    // Toggle External LED using current blink period (same as LED1 and LED2)
    if ((system_tick_counter - ext_led_last_toggle) >= current_period)
    {
        GPIOA->ODR ^= GPIOA0_ODR;            // Toggle External LED (PA0)
        ext_led_last_toggle = system_tick_counter;
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
 * @details Initializes GPIO, button, and SysTick timer, then enters an infinite loop.
 *          The actual LED toggling is handled by the SysTick interrupt.
 *          Button presses cycle through different blink frequencies.
 *
 * @return  This function never returns (infinite loop).
 *
 * @pre     System should be using the default MSI clock (4MHz).
 * @post    All three LEDs (LED1, LED2, and External LED) blink synchronously at
 *          user-selectable frequencies (1000/500/300/250/200ms).
 *          External LED (PA0) provides variable frequency signal for oscilloscope measurements.
 * ----------------------------------------------------------------------------
 */
int main(void)
{
    // Initialize peripherals
    gpio_init();
    button_init();
    systick_init();

    // Main loop - all timing is handled by SysTick interrupt
    while (1)
    {
        // Check for button press to change blink frequency
        if (is_button_pressed())
        {
            // Cycle through blink periods: 1000ms -> 500ms -> 300ms -> 250ms -> 200ms -> repeat
            current_period_index = (current_period_index + 1) % NUM_BLINK_PERIODS;
        }

        // Wait for interrupt (low power mode)
        __asm("wfi");
    }

    return 0;  // Never reached
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Detect falling edge on user button (PC13).
 * @details Uses static variable to track button state and detect press events.
 * @retval  1 if button was just pressed, 0 otherwise.
 * ----------------------------------------------------------------------------
 */
static int is_button_pressed(void)
{
    static int prev_state = 1;  // Button is pulled up (1 = not pressed)
    int current_state = (GPIOC->IDR & GPIOC13_IDR) ? 1 : 0;

    if (prev_state == 1 && current_state == 0) {
        prev_state = 0;
        return 1; // Falling edge detected (button pressed)
    } else if (current_state == 1) {
        prev_state = 1;
    }
    return 0;
}
