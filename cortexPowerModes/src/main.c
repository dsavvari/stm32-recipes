/*
 * ----------------------------------------------------------------------------
 * Project: Cortex-M Power Modes Demo on STM32L4S5VI
 * Author:  Dimitris Savvaris
 * License: MIT License
 * ----------------------------------------------------------------------------
 * Description:
 *
 * This project explores ARM Cortex-M power management and sleep modes:
 *
 * 🔋 What You'll Learn:
 * - ARM Cortex-M power modes (Normal, Sleep, Deep Sleep)
 * - WFI (Wait For Interrupt) vs WFE (Wait For Event)
 * - Sleep-on-Exit for interrupt-driven applications
 * - Power consumption optimization techniques
 * - Wake-up sources and latency considerations
 * - Real-time constraints vs power savings
 *
 * 🎯 Features Demonstrated:
 * - Cycling through different power modes with button press
 * - Current consumption measurement points
 * - Wake-up latency measurements
 * - Event-driven architecture with minimal power usage
 * - Standby and Stop modes (STM32L4 specific)
 *
 * 🔧 Hardware Setup:
 * - LED1 (PA5): Power mode indicator
 * - LED2 (PB14): Activity indicator (blinks on wake-up)
 * - External LED (PA0): Current measurement probe point
 * - Button (PC13): Mode switching and wake-up source
 * - Optional: Current measurement via IDD measurement pins
 *
 * 📚 Cortex-M Power Modes:
 * Mode          | CPU | Clocks | RAM | Wake-up Sources
 * --------------|-----|--------|-----|----------------
 * Normal Run    | ✓   | ✓      | ✓   | N/A
 * Sleep         | ✗   | ✓      | ✓   | Any interrupt
 * Deep Sleep    | ✗   | Partial| ✓   | Selected interrupts
 * Standby       | ✗   | ✗      | ✗   | Very limited sources
 *
 * 🔍 STM32L4 Power Modes (extends Cortex-M):
 * - Run Mode: Normal operation
 * - Sleep Mode: CPU stopped, peripherals running
 * - Stop 0/1/2: Different levels of clock stopping
 * - Standby: Lowest power, most wake-up latency
 * - Shutdown: Minimal power, reset on wake-up
 *
 * ----------------------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32l4s5xx.h"

/* ----------------------------------------------------------------------------
 * Constants and Definitions
 */

// GPIO pin definitions
#define LED1_PIN                5           // PA5 - Power mode indicator
#define LED2_PIN                14          // PB14 - Activity indicator
#define CURRENT_PROBE_PIN       0           // PA0 - Current measurement probe
#define BUTTON_PIN              13          // PC13 - Mode switch and wake-up

// Power mode definitions
typedef enum {
    POWER_MODE_RUN = 0,                     // Normal run mode
    POWER_MODE_SLEEP,                       // Sleep mode (WFI)
    POWER_MODE_SLEEP_WFE,                   // Sleep mode (WFE)
    POWER_MODE_STOP0,                       // Stop 0 mode
    POWER_MODE_STOP1,                       // Stop 1 mode
    POWER_MODE_STANDBY,                     // Standby mode
    POWER_MODE_COUNT                        // Number of modes
} power_mode_t;

// Timing constants
#define SYSTEM_CLOCK_HZ         4000000UL   // 4MHz MSI clock
#define ACTIVITY_BLINK_MS       100         // Activity LED blink duration
#define MODE_SWITCH_DEBOUNCE    200         // Button debounce time
#define AUTO_SLEEP_TIMEOUT      5000        // Auto-sleep after 5 seconds

/* ----------------------------------------------------------------------------
 * Global Variables
 */
static volatile uint32_t system_tick_ms = 0;
static volatile uint32_t last_activity_time = 0;
static volatile uint32_t wake_up_count = 0;
static volatile uint32_t button_press_time = 0;
static volatile power_mode_t current_power_mode = POWER_MODE_RUN;
static volatile uint8_t mode_change_requested = 0;

// Power consumption measurement
static volatile uint32_t wake_up_latency_us = 0;
static volatile uint32_t sleep_entry_time = 0;

/* ----------------------------------------------------------------------------
 * Function Prototypes
 */
static void system_init(void);
static void gpio_init(void);
static void systick_init(void);
static void exti_init(void);
static void rtc_init(void);
static void enter_power_mode(power_mode_t mode);
static void update_mode_indicator(power_mode_t mode);
static void measure_wake_up_latency(void);
static void activity_pulse(void);
static const char* get_mode_name(power_mode_t mode);

/* ----------------------------------------------------------------------------
 * Interrupt Handlers
 */

/**
 * @brief SysTick handler for system timing (only in run/sleep modes)
 */
void SysTick_Handler(void)
{
    system_tick_ms++;

    // Check for auto-sleep timeout in run mode
    if (current_power_mode == POWER_MODE_RUN) {
        if ((system_tick_ms - last_activity_time) > AUTO_SLEEP_TIMEOUT) {
            current_power_mode = POWER_MODE_SLEEP;
            mode_change_requested = 1;
        }
    }
}

/**
 * @brief Button press handler - mode switching and wake-up source
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << BUTTON_PIN)) {
        EXTI->PR1 = (1U << BUTTON_PIN);     // Clear interrupt flag

        uint32_t current_time = system_tick_ms;

        // Debounce button press
        if ((current_time - button_press_time) > MODE_SWITCH_DEBOUNCE) {
            button_press_time = current_time;
            last_activity_time = current_time;
            wake_up_count++;

            // Measure wake-up latency if coming from sleep
            if (current_power_mode != POWER_MODE_RUN) {
                measure_wake_up_latency();
            }

            // Cycle to next power mode
            current_power_mode = (current_power_mode + 1) % POWER_MODE_COUNT;
            mode_change_requested = 1;

            // Indicate activity
            activity_pulse();
        }
    }
}

/**
 * @brief RTC wake-up timer handler (for deep sleep modes)
 */
void RTC_WKUP_IRQHandler(void)
{
    if (RTC->ISR & RTC_ISR_WUTF) {
        RTC->ISR &= ~RTC_ISR_WUTF;          // Clear wake-up flag

        // Auto-wake from deep sleep modes
        wake_up_count++;
        measure_wake_up_latency();
        activity_pulse();

        // Return to run mode after wake-up
        current_power_mode = POWER_MODE_RUN;
        mode_change_requested = 1;
    }
}

/* ----------------------------------------------------------------------------
 * Initialization Functions
 */

/**
 * @brief Initialize system and enable low-power features
 */
static void system_init(void)
{
    // Enable power control clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;

    // Configure voltage regulator for low power
    PWR->CR1 &= ~PWR_CR1_VOS;               // Clear voltage scaling
    PWR->CR1 |= PWR_CR1_VOS_1;              // Voltage scaling range 2 (1.2V)

    // Enable low-power run mode capability
    PWR->CR1 |= PWR_CR1_LPR;                // Low-power run mode enable

    // Configure for sleep-on-exit (optional)
    // SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // Uncomment for sleep-on-exit
}

/**
 * @brief Initialize GPIO for power mode demonstration
 */
static void gpio_init(void)
{
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // Configure LEDs as outputs with initial state OFF
    GPIOA->MODER &= ~(3U << (LED1_PIN * 2));        // Clear PA5 mode
    GPIOA->MODER |= (1U << (LED1_PIN * 2));         // PA5 as output

    GPIOB->MODER &= ~(3U << (LED2_PIN * 2));        // Clear PB14 mode
    GPIOB->MODER |= (1U << (LED2_PIN * 2));         // PB14 as output

    GPIOA->MODER &= ~(3U << (CURRENT_PROBE_PIN * 2)); // Clear PA0 mode
    GPIOA->MODER |= (1U << (CURRENT_PROBE_PIN * 2));  // PA0 as output

    // Configure for low power - set outputs low initially
    GPIOA->ODR &= ~((1U << LED1_PIN) | (1U << CURRENT_PROBE_PIN));
    GPIOB->ODR &= ~(1U << LED2_PIN);

    // Configure button (PC13) with pull-up
    GPIOC->MODER &= ~(3U << (BUTTON_PIN * 2));      // Input mode
    GPIOC->PUPDR &= ~(3U << (BUTTON_PIN * 2));      // Clear pull config
    GPIOC->PUPDR |= (1U << (BUTTON_PIN * 2));       // Enable pull-up

    // Set GPIO speed to low for power savings
    GPIOA->OSPEEDR &= ~((3U << (LED1_PIN * 2)) | (3U << (CURRENT_PROBE_PIN * 2)));
    GPIOB->OSPEEDR &= ~(3U << (LED2_PIN * 2));
}

/**
 * @brief Initialize SysTick for 1ms interrupts
 */
static void systick_init(void)
{
    uint32_t reload_value = (SYSTEM_CLOCK_HZ / 1000) - 1; // 1ms

    SysTick->CTRL = 0;                              // Disable during setup
    SysTick->LOAD = reload_value;                   // Set reload value
    SysTick->VAL = 0;                              // Clear current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |   // Use processor clock
                    SysTick_CTRL_TICKINT_Msk |     // Enable interrupt
                    SysTick_CTRL_ENABLE_Msk;       // Enable SysTick
}

/**
 * @brief Initialize external interrupt for button wake-up
 */
static void exti_init(void)
{
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PC13 as EXTI13 source
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    // Configure EXTI13 for falling edge (button press)
    EXTI->IMR1 |= (1U << BUTTON_PIN);               // Enable interrupt
    EXTI->FTSR1 |= (1U << BUTTON_PIN);              // Falling edge trigger
    EXTI->RTSR1 &= ~(1U << BUTTON_PIN);             // Disable rising edge

    // Enable EXTI interrupt in NVIC
    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Initialize RTC for wake-up timer (deep sleep modes)
 */
static void rtc_init(void)
{
    // Enable PWR and backup domain access
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 |= PWR_CR1_DBP;                        // Enable backup domain write

    // Enable LSI for RTC (low-power internal oscillator)
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY));           // Wait for LSI ready

    // Select LSI as RTC clock source and enable RTC
    RCC->BDCR |= (1U << 8) | RCC_BDCR_RTCEN;        // LSI selection (bits 9:8 = 01)

    // Configure RTC wake-up timer for 10 second intervals
    RTC->WPR = 0xCA;                                // Unlock RTC write protection
    RTC->WPR = 0x53;

    RTC->CR &= ~RTC_CR_WUTE;                        // Disable wake-up timer
    while (!(RTC->ISR & RTC_ISR_WUTWF));            // Wait for WUTWF flag

    RTC->WUTR = 10 * 32000 / 16;                    // 10 seconds (LSI ~32kHz, prescaler 16)
    RTC->CR |= RTC_CR_WUTIE | RTC_CR_WUTE;          // Enable wake-up timer and interrupt

    RTC->WPR = 0xFF;                                // Re-lock RTC write protection

    // Enable RTC wake-up interrupt in NVIC
    NVIC_SetPriority(RTC_WKUP_IRQn, 1);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
}

/* ----------------------------------------------------------------------------
 * Power Management Functions
 */

/**
 * @brief Enter specified power mode
 */
static void enter_power_mode(power_mode_t mode)
{
    // Update mode indicator
    update_mode_indicator(mode);

    // Set current probe high to measure current consumption
    GPIOA->ODR |= (1U << CURRENT_PROBE_PIN);

    switch (mode) {
        case POWER_MODE_RUN:
            // Already in run mode - nothing special to do
            break;

        case POWER_MODE_SLEEP:
            // Sleep mode - CPU stopped, peripherals running
            sleep_entry_time = system_tick_ms;
            __WFI();                                // Wait For Interrupt
            break;

        case POWER_MODE_SLEEP_WFE:
            // Sleep mode using WFE instead of WFI
            sleep_entry_time = system_tick_ms;
            __SEV();                                // Set Event (clear any pending events)
            __WFE();                                // Clear the event
            __WFE();                                // Wait For Event
            break;

        case POWER_MODE_STOP0:
            // Stop 0 mode - clocks stopped, RAM retained, fast wake-up
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;      // Enable deep sleep
            PWR->CR1 &= ~PWR_CR1_LPMS;              // Clear low-power mode selection
            PWR->CR1 |= PWR_CR1_LPMS_STOP0;         // Select Stop 0 mode
            sleep_entry_time = system_tick_ms;
            __WFI();
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;     // Disable deep sleep
            break;

        case POWER_MODE_STOP1:
            // Stop 1 mode - more clocks stopped, slower wake-up
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            PWR->CR1 &= ~PWR_CR1_LPMS;
            PWR->CR1 |= PWR_CR1_LPMS_STOP1;
            sleep_entry_time = system_tick_ms;
            __WFI();
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            break;

        case POWER_MODE_STANDBY:
            // Standby mode - lowest power, RAM lost, slow wake-up
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            PWR->CR1 &= ~PWR_CR1_LPMS;
            PWR->CR1 |= PWR_CR1_LPMS_STANDBY;

            // Enable wake-up pin (if needed)
            PWR->CR3 |= PWR_CR3_EWUP1;              // Enable WKUP1 pin
            PWR->SCR |= PWR_SCR_CWUF;               // Clear wake-up flags

            sleep_entry_time = system_tick_ms;
            __WFI();
            // Note: After standby mode, the system resets
            break;

        default:
            break;
    }

    // Clear current probe after wake-up
    GPIOA->ODR &= ~(1U << CURRENT_PROBE_PIN);
}

/**
 * @brief Update LED1 to indicate current power mode
 */
static void update_mode_indicator(power_mode_t mode)
{
    // LED1 pattern indicates power mode:
    // Run: Solid ON
    // Sleep/Sleep_WFE: OFF (will wake up quickly)
    // Stop0: 1 blink
    // Stop1: 2 blinks
    // Standby: 3 blinks then OFF

    // Turn off LED first
    GPIOA->ODR &= ~(1U << LED1_PIN);

    switch (mode) {
        case POWER_MODE_RUN:
            GPIOA->ODR |= (1U << LED1_PIN);         // Solid ON
            break;

        case POWER_MODE_SLEEP:
        case POWER_MODE_SLEEP_WFE:
            // LED OFF - will wake up quickly
            break;

        case POWER_MODE_STOP0:
            // 1 blink
            for (int i = 0; i < 1; i++) {
                GPIOA->ODR |= (1U << LED1_PIN);
                for (volatile uint32_t j = 0; j < 100000; j++) __NOP();
                GPIOA->ODR &= ~(1U << LED1_PIN);
                for (volatile uint32_t j = 0; j < 100000; j++) __NOP();
            }
            break;

        case POWER_MODE_STOP1:
            // 2 blinks
            for (int i = 0; i < 2; i++) {
                GPIOA->ODR |= (1U << LED1_PIN);
                for (volatile uint32_t j = 0; j < 100000; j++) __NOP();
                GPIOA->ODR &= ~(1U << LED1_PIN);
                for (volatile uint32_t j = 0; j < 100000; j++) __NOP();
            }
            break;

        case POWER_MODE_STANDBY:
            // 3 blinks then stay OFF
            for (int i = 0; i < 3; i++) {
                GPIOA->ODR |= (1U << LED1_PIN);
                for (volatile uint32_t j = 0; j < 100000; j++) __NOP();
                GPIOA->ODR &= ~(1U << LED1_PIN);
                for (volatile uint32_t j = 0; j < 100000; j++) __NOP();
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Measure wake-up latency from sleep modes
 */
static void measure_wake_up_latency(void)
{
    // Simple latency measurement (in a real application, use a high-resolution timer)
    uint32_t wake_time = system_tick_ms;
    if (wake_time > sleep_entry_time) {
        wake_up_latency_us = (wake_time - sleep_entry_time) * 1000; // Convert to microseconds
    }
}

/**
 * @brief Pulse LED2 to indicate activity/wake-up
 */
static void activity_pulse(void)
{
    GPIOB->ODR |= (1U << LED2_PIN);                 // Turn on LED2
    for (volatile uint32_t i = 0; i < 200000; i++) __NOP(); // Short delay
    GPIOB->ODR &= ~(1U << LED2_PIN);                // Turn off LED2
}

/**
 * @brief Get human-readable name for power mode
 */
static const char* get_mode_name(power_mode_t mode)
{
    switch (mode) {
        case POWER_MODE_RUN:        return "Run";
        case POWER_MODE_SLEEP:      return "Sleep (WFI)";
        case POWER_MODE_SLEEP_WFE:  return "Sleep (WFE)";
        case POWER_MODE_STOP0:      return "Stop 0";
        case POWER_MODE_STOP1:      return "Stop 1";
        case POWER_MODE_STANDBY:    return "Standby";
        default:                    return "Unknown";
    }
}

/* ----------------------------------------------------------------------------
 * Main Function
 */

/**
 * @brief Main function demonstrating Cortex-M power modes
 * @return Never returns (infinite loop)
 */
int main(void)
{
    // Initialize all subsystems
    system_init();
    gpio_init();
    exti_init();
    rtc_init();
    systick_init();

    // Enable global interrupts
    __enable_irq();

    // Initialize state
    last_activity_time = system_tick_ms;
    current_power_mode = POWER_MODE_RUN;

    printf("Cortex-M Power Modes Demo Started!\n");
    printf("Press button (PC13) to cycle through power modes:\n");
    printf("- Run: LED1 solid, auto-sleep after 5s\n");
    printf("- Sleep (WFI): LED1 off, quick wake-up\n");
    printf("- Sleep (WFE): LED1 off, event-driven wake-up\n");
    printf("- Stop 0: LED1 blinks 1x, medium power saving\n");
    printf("- Stop 1: LED1 blinks 2x, higher power saving\n");
    printf("- Standby: LED1 blinks 3x, maximum power saving\n");
    printf("LED2 pulses on wake-up/activity\n");
    printf("PA0 goes high during sleep for current measurement\n\n");

    // Main loop
    while (1)
    {
        // Handle mode changes
        if (mode_change_requested) {
            mode_change_requested = 0;

            printf("Switching to %s mode...\n", get_mode_name(current_power_mode));

            // Small delay to allow UART transmission to complete
            for (volatile uint32_t i = 0; i < 100000; i++) __NOP();
        }

        // Enter the current power mode
        enter_power_mode(current_power_mode);

        // When we reach here, we've woken up from a sleep mode
        if (current_power_mode != POWER_MODE_RUN) {
            printf("Woke up from %s mode (wake-up #%lu, latency: %lu us)\n",
                   get_mode_name(current_power_mode), wake_up_count, wake_up_latency_us);
        }

        // Reset to run mode after waking from deep sleep modes
        if (current_power_mode >= POWER_MODE_STOP0) {
            current_power_mode = POWER_MODE_RUN;
            last_activity_time = system_tick_ms;
        }
    }

    return 0;  // Never reached
}
