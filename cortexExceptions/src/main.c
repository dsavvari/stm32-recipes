/*
 * ----------------------------------------------------------------------------
 * Project: Cortex-M Exception and Interrupt Demo on STM32L4S5VI
 * Author:  Dimitris Savvaris
 * License: MIT License
 * ----------------------------------------------------------------------------
 * Description:
 *
 * This project explores fundamental ARM Cortex-M exception and interrupt handling:
 *
 * 🔍 What You'll Learn:
 * - Exception vs Interrupt concepts
 * - NVIC (Nested Vector Interrupt Controller)
 * - Interrupt priorities and preemption
 * - Exception handlers and vector table
 * - Pending and active interrupt states
 * - Critical sections and interrupt masking
 *
 * 🎯 Features Demonstrated:
 * - Multiple interrupt sources with different priorities
 * - SysTick (highest priority) for system timing
 * - External interrupt (EXTI) on user button
 * - Timer interrupt (TIM2) for periodic tasks
 * - Priority-based preemption examples
 * - Safe critical section implementation
 *
 * 🔧 Hardware Setup:
 * - LED1 (PA5): Controlled by SysTick (1Hz, highest priority)
 * - LED2 (PB14): Controlled by Timer2 (2Hz, medium priority)
 * - External LED (PA0): Controlled by button press (lowest priority)
 * - Button (PC13): Triggers external interrupt
 * - UART Debug output for interrupt monitoring
 *
 * 📚 Cortex-M Exception System:
 * Exception Number | Exception Type     | Priority | Handler
 * ------------------|-------------------|----------|------------------
 * 1                 | Reset             | -3       | Reset_Handler
 * 2                 | NMI               | -2       | NMI_Handler
 * 3                 | Hard Fault        | -1       | HardFault_Handler
 * 11                | SVCall            | Config   | SVC_Handler
 * 14                | PendSV            | Config   | PendSV_Handler
 * 15                | SysTick           | Config   | SysTick_Handler
 * 16+               | IRQ0-239          | Config   | IRQn_Handler
 *
 * ----------------------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32l4s5xx.h"

/* ----------------------------------------------------------------------------
 * Constants and Definitions
 */

// System configuration
#define SYSTEM_CLOCK_HZ         4000000UL   // 4MHz MSI clock
#define SYSTICK_FREQ_HZ         1000UL      // 1kHz = 1ms period

// GPIO definitions for LEDs
#define LED1_PIN                5           // PA5
#define LED2_PIN                14          // PB14
#define EXT_LED_PIN             0           // PA0 (ARD-D1)
#define BUTTON_PIN              13          // PC13

// Interrupt priorities (0 = highest, 15 = lowest for STM32L4)
#define SYSTICK_PRIORITY        0           // Highest priority - system timing
#define TIM2_PRIORITY           4           // Medium priority - periodic task
#define EXTI13_PRIORITY         8           // Lower priority - user input

// Timing constants (in milliseconds)
#define LED1_TOGGLE_PERIOD      1000        // 1Hz - SysTick controlled
#define LED2_TOGGLE_PERIOD      500         // 2Hz - Timer2 controlled
#define DEBOUNCE_DELAY          50          // Button debounce delay

/* ----------------------------------------------------------------------------
 * Global Variables
 */
static volatile uint32_t system_tick_ms = 0;
static volatile uint32_t led1_last_toggle = 0;
static volatile uint32_t button_press_count = 0;
static volatile uint32_t interrupt_stats[3] = {0}; // [systick, timer2, exti13]

// Critical section flag
static volatile uint8_t in_critical_section = 0;

/* ----------------------------------------------------------------------------
 * Function Prototypes
 */
static void system_init(void);
static void gpio_init(void);
static void systick_init(void);
static void timer2_init(void);
static void exti_init(void);
static void nvic_init(void);
static void uart_debug_init(void);
static void enter_critical_section(void);
static void exit_critical_section(void);
static void debug_print_stats(void);

/* ----------------------------------------------------------------------------
 * Exception and Interrupt Handlers
 */

/**
 * @brief SysTick interrupt handler - Highest Priority
 * @note  This runs every 1ms and controls LED1 at 1Hz
 */
void SysTick_Handler(void)
{
    interrupt_stats[0]++;           // Count SysTick interrupts
    system_tick_ms++;               // Increment system time

    // Toggle LED1 at 1Hz (every 1000ms)
    if ((system_tick_ms - led1_last_toggle) >= LED1_TOGGLE_PERIOD) {
        GPIOA->ODR ^= (1U << LED1_PIN);     // Toggle LED1 (PA5)
        led1_last_toggle = system_tick_ms;
    }

    // Print debug stats every 5 seconds
    if ((system_tick_ms % 5000) == 0) {
        debug_print_stats();
    }
}

/**
 * @brief Timer2 interrupt handler - Medium Priority
 * @note  This runs every 250ms and controls LED2 at 2Hz
 */
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {        // Check update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;        // Clear interrupt flag

        interrupt_stats[1]++;            // Count Timer2 interrupts
        GPIOB->ODR ^= (1U << LED2_PIN);  // Toggle LED2 (PB14)
    }
}

/**
 * @brief External interrupt handler for PC13 (Button) - Lower Priority
 * @note  This can be preempted by SysTick and Timer2
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << BUTTON_PIN)) {   // Check if PC13 triggered interrupt
        EXTI->PR1 = (1U << BUTTON_PIN);     // Clear interrupt flag

        interrupt_stats[2]++;                // Count EXTI interrupts
        button_press_count++;               // Count button presses

        // Toggle External LED (this may be interrupted by higher priority handlers)
        GPIOA->ODR ^= (1U << EXT_LED_PIN);  // Toggle External LED (PA0)

        // Simulate some work that can be preempted
        for (volatile uint32_t i = 0; i < 100000; i++) {
            __NOP();  // This delay can be interrupted by higher priority interrupts
        }
    }
}

/**
 * @brief Hard Fault handler - demonstrate fault debugging
 */
void HardFault_Handler(void)
{
    // In a real application, you'd analyze fault registers here
    while (1) {
        // Blink all LEDs rapidly to indicate fault
        GPIOA->ODR ^= (1U << LED1_PIN) | (1U << EXT_LED_PIN);
        GPIOB->ODR ^= (1U << LED2_PIN);

        for (volatile uint32_t i = 0; i < 200000; i++) __NOP();
    }
}

/* ----------------------------------------------------------------------------
 * Initialization Functions
 */

/**
 * @brief Initialize system clocks and core components
 */
static void system_init(void)
{
    // Enable FPU if available (Cortex-M4F)
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // Enable CP10 and CP11 Full Access
    #endif

    // Configure system for debug
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
}

/**
 * @brief Initialize GPIO for LEDs and button
 */
static void gpio_init(void)
{
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // Configure LEDs as outputs
    // LED1 (PA5)
    GPIOA->MODER &= ~(3U << (LED1_PIN * 2));        // Clear mode bits
    GPIOA->MODER |= (1U << (LED1_PIN * 2));         // Set as output

    // LED2 (PB14)
    GPIOB->MODER &= ~(3U << (LED2_PIN * 2));        // Clear mode bits
    GPIOB->MODER |= (1U << (LED2_PIN * 2));         // Set as output

    // External LED (PA0)
    GPIOA->MODER &= ~(3U << (EXT_LED_PIN * 2));     // Clear mode bits
    GPIOA->MODER |= (1U << (EXT_LED_PIN * 2));      // Set as output

    // Configure button (PC13) as input with pull-up
    GPIOC->MODER &= ~(3U << (BUTTON_PIN * 2));      // Input mode (default)
    GPIOC->PUPDR &= ~(3U << (BUTTON_PIN * 2));      // Clear pull-up/down bits
    GPIOC->PUPDR |= (1U << (BUTTON_PIN * 2));       // Enable pull-up

    // Turn off all LEDs initially
    GPIOA->ODR &= ~((1U << LED1_PIN) | (1U << EXT_LED_PIN));
    GPIOB->ODR &= ~(1U << LED2_PIN);
}

/**
 * @brief Initialize SysTick timer for 1ms interrupts
 */
static void systick_init(void)
{
    uint32_t reload_value = (SYSTEM_CLOCK_HZ / SYSTICK_FREQ_HZ) - 1;

    SysTick->CTRL = 0;                              // Disable SysTick during setup
    SysTick->LOAD = reload_value;                   // Set reload value
    SysTick->VAL  = 0;                             // Clear current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |   // Use processor clock
                    SysTick_CTRL_TICKINT_Msk |     // Enable interrupt
                    SysTick_CTRL_ENABLE_Msk;       // Enable SysTick
}

/**
 * @brief Initialize Timer2 for periodic interrupts (2Hz = 500ms period)
 */
static void timer2_init(void)
{
    // Enable Timer2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Configure Timer2 for 500ms interrupts
    // Timer clock = 4MHz, want 2Hz interrupts
    // Prescaler = 4000-1 (1kHz), Auto-reload = 500-1 (500ms)
    TIM2->PSC = 4000 - 1;                          // Prescaler for 1kHz
    TIM2->ARR = 500 - 1;                           // Auto-reload for 500ms
    TIM2->DIER |= TIM_DIER_UIE;                    // Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN;                      // Enable timer
}

/**
 * @brief Initialize external interrupt for button (PC13)
 */
static void exti_init(void)
{
    // Enable SYSCFG clock for EXTI configuration
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PC13 as EXTI13 source
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;   // Clear EXTI13 config
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;  // Set PC13 as EXTI13 source

    // Configure EXTI13 for falling edge (button press)
    EXTI->IMR1 |= (1U << BUTTON_PIN);               // Enable interrupt mask
    EXTI->FTSR1 |= (1U << BUTTON_PIN);              // Enable falling edge trigger
    EXTI->RTSR1 &= ~(1U << BUTTON_PIN);             // Disable rising edge trigger
}

/**
 * @brief Configure NVIC priorities and enable interrupts
 */
static void nvic_init(void)
{
    // Set interrupt priorities
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORITY);
    NVIC_SetPriority(TIM2_IRQn, TIM2_PRIORITY);
    NVIC_SetPriority(EXTI15_10_IRQn, EXTI13_PRIORITY);

    // Enable interrupts in NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // SysTick interrupt is enabled in systick_init()
}

/**
 * @brief Initialize UART for debug output (simplified)
 */
static void uart_debug_init(void)
{
    // For now, just a placeholder - would implement UART/ITM in real scenario
    // This keeps the example focused on exception handling
}

/* ----------------------------------------------------------------------------
 * Critical Section Functions
 */

/**
 * @brief Enter critical section by disabling interrupts
 */
static void enter_critical_section(void)
{
    __disable_irq();                    // Disable all interrupts
    in_critical_section = 1;
}

/**
 * @brief Exit critical section by re-enabling interrupts
 */
static void exit_critical_section(void)
{
    in_critical_section = 0;
    __enable_irq();                     // Re-enable interrupts
}

/**
 * @brief Print debug statistics (placeholder for UART/ITM output)
 */
static void debug_print_stats(void)
{
    // In a real application, this would output via UART or ITM
    // For now, we'll use variables that can be observed in debugger
    static uint32_t debug_systick_count = 0;
    static uint32_t debug_timer2_count = 0;
    static uint32_t debug_exti_count = 0;
    static uint32_t debug_button_presses = 0;

    // Copy volatile variables to local ones for debugger observation
    enter_critical_section();
    debug_systick_count = interrupt_stats[0];
    debug_timer2_count = interrupt_stats[1];
    debug_exti_count = interrupt_stats[2];
    debug_button_presses = button_press_count;
    exit_critical_section();

    // These variables can be watched in the debugger
    (void)debug_systick_count;
    (void)debug_timer2_count;
    (void)debug_exti_count;
    (void)debug_button_presses;
}

/* ----------------------------------------------------------------------------
 * Main Function
 */

/**
 * @brief Main function demonstrating Cortex-M exception handling
 * @return Never returns (infinite loop)
 */
int main(void)
{
    // Initialize all subsystems
    system_init();
    gpio_init();
    uart_debug_init();

    // Initialize interrupt sources (order matters for priorities)
    exti_init();
    timer2_init();
    systick_init();      // Start this last as it has highest priority
    nvic_init();

    // Enable global interrupts
    __enable_irq();

    printf("Cortex-M Exception Demo Started!\n");
    printf("- LED1 (PA5): SysTick controlled (1Hz, Priority %d)\n", SYSTICK_PRIORITY);
    printf("- LED2 (PB14): Timer2 controlled (2Hz, Priority %d)\n", TIM2_PRIORITY);
    printf("- External LED (PA0): Button controlled (Priority %d)\n", EXTI13_PRIORITY);
    printf("- Press button (PC13) to trigger lower priority interrupt\n");
    printf("- Watch how higher priority interrupts preempt lower ones\n\n");

    // Main loop - demonstrate different power modes
    while (1)
    {
        // Enter sleep mode - wake up on any interrupt
        __WFI();        // Wait For Interrupt

        // When we wake up, all interrupt processing is complete
        // This is where you'd do non-time-critical background tasks

        // Example: Check if we need to do any non-interrupt work
        static uint32_t last_maintenance = 0;
        if ((system_tick_ms - last_maintenance) > 10000) {  // Every 10 seconds
            // Do some maintenance work that's not time-critical
            last_maintenance = system_tick_ms;

            // This work could be interrupted by any enabled interrupt
            for (volatile uint32_t i = 0; i < 500000; i++) {
                __NOP();    // Simulate background work
            }
        }
    }

    return 0;  // Never reached
}

/* ----------------------------------------------------------------------------
 * Additional Exception Handlers (optional)
 */

/**
 * @brief NMI Handler - Non-Maskable Interrupt
 */
void NMI_Handler(void)
{
    // Handle non-maskable interrupt
    while (1) {
        // In real application, handle critical system errors
    }
}

/**
 * @brief Memory Management Fault Handler
 */
void MemManage_Handler(void)
{
    // Handle memory management faults
    while (1) {
        // Analyze SCB->CFSR for fault details
    }
}

/**
 * @brief Bus Fault Handler
 */
void BusFault_Handler(void)
{
    // Handle bus faults
    while (1) {
        // Analyze SCB->CFSR for fault details
    }
}

/**
 * @brief Usage Fault Handler
 */
void UsageFault_Handler(void)
{
    // Handle usage faults (undefined instructions, etc.)
    while (1) {
        // Analyze SCB->CFSR for fault details
    }
}
