/*
 * ----------------------------------------------------------------------------
 * Project: Cortex-M System Control and Stack Management on STM32L4S5VI
 * Author:  Dimitris Savvaris
 * License: MIT License
 * ----------------------------------------------------------------------------
 * Description:
 *
 * This project explores ARM Cortex-M system control registers and stack management:
 *
 * 🎯 What You'll Learn:
 * - MSP (Main Stack Pointer) vs PSP (Process Stack Pointer)
 * - CONTROL register and privilege levels
 * - Thread mode vs Handler mode
 * - Privileged vs Unprivileged execution
 * - System Control Block (SCB) registers
 * - Stack switching and dual-stack operation
 * - CPU ID and feature detection
 * - Reset types and system state
 *
 * 🔧 Features Demonstrated:
 * - MSP/PSP switching demonstration
 * - Privilege level changes (Privileged ↔ Unprivileged)
 * - Stack pointer monitoring and validation
 * - CONTROL register manipulation
 * - SCB register inspection and configuration
 * - System information extraction (CPUID, features)
 * - Stack frame analysis
 * - Reset source detection
 *
 * 📚 Key ARM Cortex-M Concepts:
 *
 * Stack Pointers:
 * - MSP: Main Stack Pointer (privileged, exception handlers)
 * - PSP: Process Stack Pointer (user tasks, RTOS threads)
 *
 * CONTROL Register (3 bits):
 * - Bit 0 (nPRIV): 0=Privileged, 1=Unprivileged in Thread mode
 * - Bit 1 (SPSEL): 0=Use MSP, 1=Use PSP in Thread mode
 * - Bit 2 (FPCA):  Floating-point context active (Cortex-M4F)
 *
 * Execution Modes:
 * - Thread Mode: Normal program execution
 * - Handler Mode: Exception/interrupt execution (always privileged, always MSP)
 *
 * 🔧 Hardware Setup:
 * - LED1 (PA5): Stack pointer indicator (MSP=ON, PSP=Blink)
 * - LED2 (PB14): Privilege level indicator (Privileged=ON, Unprivileged=Blink)
 * - External LED (PA0): CONTROL register bit pattern
 * - Button (PC13): Toggle between MSP/PSP and privilege levels
 *
 * ----------------------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include "stm32l4s5xx.h"

/* ----------------------------------------------------------------------------
 * Constants and Definitions
 */

// GPIO definitions
#define LED1_PIN                5           // PA5 - Stack pointer indicator
#define LED2_PIN                14          // PB14 - Privilege level indicator
#define CONTROL_LED_PIN         0           // PA0 - CONTROL register indicator
#define BUTTON_PIN              13          // PC13 - Mode switching

// Stack configuration
#define PSP_STACK_SIZE          1024        // Process stack size (1KB)
#define STACK_GUARD_PATTERN     0xDEADBEEF  // Stack corruption detection

// System states
typedef enum {
    SYSTEM_STATE_MSP_PRIVILEGED = 0,        // MSP + Privileged
    SYSTEM_STATE_MSP_UNPRIVILEGED,          // MSP + Unprivileged
    SYSTEM_STATE_PSP_PRIVILEGED,            // PSP + Privileged
    SYSTEM_STATE_PSP_UNPRIVILEGED,          // PSP + Unprivileged
    SYSTEM_STATE_COUNT
} system_state_t;

/* ----------------------------------------------------------------------------
 * Global Variables
 */
static volatile uint32_t system_tick_ms = 0;
static volatile uint32_t mode_switch_count = 0;
static volatile system_state_t current_state = SYSTEM_STATE_MSP_PRIVILEGED;
static volatile uint8_t state_change_requested = 0;

// Process stack for PSP mode
static uint32_t process_stack[PSP_STACK_SIZE / sizeof(uint32_t)] __attribute__((aligned(8)));
static uint32_t* process_stack_top = &process_stack[PSP_STACK_SIZE / sizeof(uint32_t) - 1];

// System information structure
typedef struct {
    uint32_t cpuid;             // CPU ID register
    uint32_t implementer;       // Implementer code
    uint32_t variant;           // Variant number
    uint32_t architecture;      // Architecture version
    uint32_t part_number;       // Part number
    uint32_t revision;          // Revision number
    uint32_t msp_value;         // Current MSP
    uint32_t psp_value;         // Current PSP
    uint32_t control_value;     // CONTROL register
    uint32_t primask_value;     // PRIMASK register
    uint32_t reset_flags;       // Reset source flags
} system_info_t;

static system_info_t system_info;

/* ----------------------------------------------------------------------------
 * Function Prototypes
 */
static void system_init(void);
static void gpio_init(void);
static void systick_init(void);
static void exti_init(void);
static void setup_process_stack(void);
static void switch_to_state(system_state_t state);
static void update_led_indicators(void);
static void collect_system_info(void);
static void print_system_info(void);
static void demonstrate_stack_switch(void);
static void demonstrate_privilege_switch(void);
static void analyze_stack_frame(void);
static const char* get_state_name(system_state_t state);
static uint32_t get_current_stack_pointer(void);
static uint8_t is_using_psp(void);
static uint8_t is_privileged(void);

/* ----------------------------------------------------------------------------
 * Assembly Helper Functions
 */

// Assembly functions for low-level operations
__attribute__((naked)) void switch_to_psp(uint32_t psp_value)
{
    __asm volatile (
        "msr psp, r0        \n"  // Set PSP to provided value
        "mrs r0, control    \n"  // Read CONTROL register
        "orr r0, r0, #2     \n"  // Set SPSEL bit (use PSP)
        "msr control, r0    \n"  // Write back CONTROL
        "isb                \n"  // Instruction Synchronization Barrier
        "bx lr              \n"  // Return
    );
}

__attribute__((naked)) void switch_to_msp(void)
{
    __asm volatile (
        "mrs r0, control    \n"  // Read CONTROL register
        "bic r0, r0, #2     \n"  // Clear SPSEL bit (use MSP)
        "msr control, r0    \n"  // Write back CONTROL
        "isb                \n"  // Instruction Synchronization Barrier
        "bx lr              \n"  // Return
    );
}

__attribute__((naked)) void drop_to_unprivileged(void)
{
    __asm volatile (
        "mrs r0, control    \n"  // Read CONTROL register
        "orr r0, r0, #1     \n"  // Set nPRIV bit (unprivileged)
        "msr control, r0    \n"  // Write back CONTROL
        "isb                \n"  // Instruction Synchronization Barrier
        "bx lr              \n"  // Return
    );
}

// Note: Can only return to privileged mode via exception handler
void return_to_privileged(void)
{
    // Trigger SVC call to return to privileged mode
    __asm volatile ("svc #0");
}

/* ----------------------------------------------------------------------------
 * Interrupt Handlers
 */

/**
 * @brief SysTick handler - always runs in Handler mode (privileged, MSP)
 */
void SysTick_Handler(void)
{
    system_tick_ms++;

    // Update LED indicators based on current state
    update_led_indicators();

    // Collect system info periodically
    if ((system_tick_ms % 1000) == 0) {
        collect_system_info();
    }
}

/**
 * @brief Button press handler - mode switching
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << BUTTON_PIN)) {
        EXTI->PR1 = (1U << BUTTON_PIN);     // Clear interrupt flag

        mode_switch_count++;
        state_change_requested = 1;

        // Cycle to next state
        current_state = (current_state + 1) % SYSTEM_STATE_COUNT;
    }
}

/**
 * @brief SVC handler - used to return to privileged mode
 */
void SVC_Handler(void)
{
    // This handler runs in privileged mode and can change CONTROL register
    uint32_t control = __get_CONTROL();
    control &= ~1;  // Clear nPRIV bit (return to privileged)
    __set_CONTROL(control);
    __ISB();
}

/* ----------------------------------------------------------------------------
 * Initialization Functions
 */

/**
 * @brief Initialize system and enable system control features
 */
static void system_init(void)
{
    // Initialize process stack
    setup_process_stack();

    // Collect initial system information
    collect_system_info();

    printf("ARM Cortex-M System Control Demo Starting...\n");
    print_system_info();
}

/**
 * @brief Initialize GPIO for system control demonstration
 */
static void gpio_init(void)
{
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // Configure LEDs as outputs
    GPIOA->MODER &= ~((3U << (LED1_PIN * 2)) | (3U << (CONTROL_LED_PIN * 2)));
    GPIOA->MODER |= (1U << (LED1_PIN * 2)) | (1U << (CONTROL_LED_PIN * 2));

    GPIOB->MODER &= ~(3U << (LED2_PIN * 2));
    GPIOB->MODER |= (1U << (LED2_PIN * 2));

    // Configure button with pull-up
    GPIOC->MODER &= ~(3U << (BUTTON_PIN * 2));      // Input mode
    GPIOC->PUPDR &= ~(3U << (BUTTON_PIN * 2));
    GPIOC->PUPDR |= (1U << (BUTTON_PIN * 2));       // Pull-up

    // Initialize LEDs OFF
    GPIOA->ODR &= ~((1U << LED1_PIN) | (1U << CONTROL_LED_PIN));
    GPIOB->ODR &= ~(1U << LED2_PIN);
}

/**
 * @brief Initialize SysTick for timing
 */
static void systick_init(void)
{
    uint32_t reload_value = (4000000 / 1000) - 1;  // 1ms at 4MHz

    SysTick->CTRL = 0;
    SysTick->LOAD = reload_value;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief Initialize external interrupt for button
 */
static void exti_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    EXTI->IMR1 |= (1U << BUTTON_PIN);
    EXTI->FTSR1 |= (1U << BUTTON_PIN);
    EXTI->RTSR1 &= ~(1U << BUTTON_PIN);

    NVIC_SetPriority(EXTI15_10_IRQn, 2);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Setup process stack with guard patterns
 */
static void setup_process_stack(void)
{
    // Fill stack with guard pattern for debugging
    for (size_t i = 0; i < PSP_STACK_SIZE / sizeof(uint32_t); i++) {
        process_stack[i] = STACK_GUARD_PATTERN;
    }

    // Set up initial stack frame at top of process stack
    // This mimics what happens during exception entry
    process_stack_top -= 8;  // Reserve space for exception frame

    printf("Process stack setup:\n");
    printf("  Base: 0x%08lX\n", (uint32_t)process_stack);
    printf("  Top:  0x%08lX\n", (uint32_t)process_stack_top);
    printf("  Size: %d bytes\n", PSP_STACK_SIZE);
}

/* ----------------------------------------------------------------------------
 * System Control Functions
 */

/**
 * @brief Switch to specified system state
 */
static void switch_to_state(system_state_t state)
{
    printf("\nSwitching to %s...\n", get_state_name(state));

    // Always start from privileged mode to make changes
    if (!is_privileged()) {
        return_to_privileged();
    }

    switch (state) {
        case SYSTEM_STATE_MSP_PRIVILEGED:
            switch_to_msp();
            // Already in privileged mode
            break;

        case SYSTEM_STATE_MSP_UNPRIVILEGED:
            switch_to_msp();
            drop_to_unprivileged();
            break;

        case SYSTEM_STATE_PSP_PRIVILEGED:
            switch_to_psp((uint32_t)process_stack_top);
            // Stay in privileged mode
            break;

        case SYSTEM_STATE_PSP_UNPRIVILEGED:
            switch_to_psp((uint32_t)process_stack_top);
            drop_to_unprivileged();
            break;

        default:
            break;
    }

    // Update system info after state change
    collect_system_info();

    printf("State changed successfully!\n");
    printf("  Using %s, %s mode\n",
           is_using_psp() ? "PSP" : "MSP",
           is_privileged() ? "Privileged" : "Unprivileged");
}

/**
 * @brief Update LED indicators based on current system state
 */
static void update_led_indicators(void)
{
    uint32_t control = __get_CONTROL();

    // LED1: Stack pointer indicator
    if (control & 0x02) {  // SPSEL bit
        // Using PSP - blink LED1
        if ((system_tick_ms % 500) < 250) {
            GPIOA->ODR |= (1U << LED1_PIN);
        } else {
            GPIOA->ODR &= ~(1U << LED1_PIN);
        }
    } else {
        // Using MSP - solid on
        GPIOA->ODR |= (1U << LED1_PIN);
    }

    // LED2: Privilege level indicator
    if (control & 0x01) {  // nPRIV bit
        // Unprivileged - blink LED2
        if ((system_tick_ms % 300) < 150) {
            GPIOB->ODR |= (1U << LED2_PIN);
        } else {
            GPIOB->ODR &= ~(1U << LED2_PIN);
        }
    } else {
        // Privileged - solid on
        GPIOB->ODR |= (1U << LED2_PIN);
    }

    // CONTROL LED: Show CONTROL register bit pattern
    if ((system_tick_ms % 1000) < (control * 100 + 100)) {
        GPIOA->ODR |= (1U << CONTROL_LED_PIN);
    } else {
        GPIOA->ODR &= ~(1U << CONTROL_LED_PIN);
    }
}

/**
 * @brief Collect current system information
 */
static void collect_system_info(void)
{
    system_info.cpuid = SCB->CPUID;
    system_info.msp_value = __get_MSP();
    system_info.psp_value = __get_PSP();
    system_info.control_value = __get_CONTROL();
    system_info.primask_value = __get_PRIMASK();

    // Extract CPUID fields
    system_info.implementer = (system_info.cpuid >> 24) & 0xFF;
    system_info.variant = (system_info.cpuid >> 20) & 0xF;
    system_info.architecture = (system_info.cpuid >> 16) & 0xF;
    system_info.part_number = (system_info.cpuid >> 4) & 0xFFF;
    system_info.revision = system_info.cpuid & 0xF;

    // Check reset flags (simplified)
    system_info.reset_flags = RCC->CSR;
}

/**
 * @brief Print comprehensive system information
 */
static void print_system_info(void)
{
    printf("\n=== ARM Cortex-M System Information ===\n");
    printf("CPUID: 0x%08lX\n", system_info.cpuid);
    printf("  Implementer: 0x%02lX (%s)\n", system_info.implementer,
           (system_info.implementer == 0x41) ? "ARM" : "Unknown");
    printf("  Architecture: 0x%lX (%s)\n", system_info.architecture,
           (system_info.architecture == 0xF) ? "ARMv7-M" : "Unknown");
    printf("  Part Number: 0x%03lX (%s)\n", system_info.part_number,
           (system_info.part_number == 0xC24) ? "Cortex-M4" : "Unknown");
    printf("  Variant: 0x%lX\n", system_info.variant);
    printf("  Revision: 0x%lX\n", system_info.revision);

    printf("\n=== Stack Pointers ===\n");
    printf("MSP: 0x%08lX\n", system_info.msp_value);
    printf("PSP: 0x%08lX\n", system_info.psp_value);
    printf("Current SP: 0x%08lX (%s)\n", get_current_stack_pointer(),
           is_using_psp() ? "PSP" : "MSP");

    printf("\n=== Control Registers ===\n");
    printf("CONTROL: 0x%08lX\n", system_info.control_value);
    printf("  nPRIV (bit 0): %lu (%s)\n",
           (system_info.control_value & 1),
           (system_info.control_value & 1) ? "Unprivileged" : "Privileged");
    printf("  SPSEL (bit 1): %lu (%s)\n",
           (system_info.control_value >> 1) & 1,
           ((system_info.control_value >> 1) & 1) ? "PSP" : "MSP");
    printf("  FPCA (bit 2):  %lu (%s)\n",
           (system_info.control_value >> 2) & 1,
           ((system_info.control_value >> 2) & 1) ? "FP context active" : "No FP context");

    printf("PRIMASK: 0x%08lX (%s)\n", system_info.primask_value,
           system_info.primask_value ? "Interrupts disabled" : "Interrupts enabled");

    printf("\n");
}

/**
 * @brief Demonstrate stack switching
 */
static void demonstrate_stack_switch(void)
{
    printf("\n=== Stack Switching Demonstration ===\n");

    uint32_t sp_before = get_current_stack_pointer();
    printf("Before switch - SP: 0x%08lX (%s)\n", sp_before, is_using_psp() ? "PSP" : "MSP");

    // Allocate some local variables to show stack usage
    volatile uint32_t local_array[16];
    for (int i = 0; i < 16; i++) {
        local_array[i] = i * 0x12345678;
    }

    uint32_t sp_after = get_current_stack_pointer();
    printf("After allocation - SP: 0x%08lX (used %ld bytes)\n",
           sp_after, sp_before - sp_after);

    printf("Local array address: 0x%08lX\n", (uint32_t)local_array);
    printf("Local array[0] = 0x%08lX\n", local_array[0]);
}

/**
 * @brief Demonstrate privilege switching
 */
static void demonstrate_privilege_switch(void)
{
    printf("\n=== Privilege Level Demonstration ===\n");

    printf("Current privilege: %s\n", is_privileged() ? "Privileged" : "Unprivileged");

    if (is_privileged()) {
        printf("Can access system registers...\n");
        uint32_t aircr = SCB->AIRCR;
        printf("  SCB->AIRCR = 0x%08lX\n", aircr);

        // This would cause a fault in unprivileged mode
        printf("  Successfully read system register!\n");
    } else {
        printf("Cannot access system registers in unprivileged mode\n");
        printf("  (Would cause usage fault if attempted)\n");
    }
}

/* ----------------------------------------------------------------------------
 * Helper Functions
 */

/**
 * @brief Get human-readable state name
 */
static const char* get_state_name(system_state_t state)
{
    switch (state) {
        case SYSTEM_STATE_MSP_PRIVILEGED:   return "MSP + Privileged";
        case SYSTEM_STATE_MSP_UNPRIVILEGED: return "MSP + Unprivileged";
        case SYSTEM_STATE_PSP_PRIVILEGED:   return "PSP + Privileged";
        case SYSTEM_STATE_PSP_UNPRIVILEGED: return "PSP + Unprivileged";
        default:                            return "Unknown";
    }
}

/**
 * @brief Get current stack pointer value
 */
static uint32_t get_current_stack_pointer(void)
{
    if (is_using_psp()) {
        return __get_PSP();
    } else {
        return __get_MSP();
    }
}

/**
 * @brief Check if currently using PSP
 */
static uint8_t is_using_psp(void)
{
    return (__get_CONTROL() & 0x02) ? 1 : 0;
}

/**
 * @brief Check if currently in privileged mode
 */
static uint8_t is_privileged(void)
{
    return (__get_CONTROL() & 0x01) ? 0 : 1;
}

/**
 * @brief Analyze current stack frame
 */
static void analyze_stack_frame(void)
{
    printf("\n=== Stack Frame Analysis ===\n");

    uint32_t current_sp = get_current_stack_pointer();
    printf("Current SP: 0x%08lX\n", current_sp);

    // Show some stack content (be careful with memory access)
    printf("Stack content (top 8 words):\n");
    for (int i = 0; i < 8; i++) {
        uint32_t addr = current_sp + (i * 4);
        printf("  [0x%08lX]: 0x%08lX\n", addr, *(volatile uint32_t*)addr);
    }
}

/* ----------------------------------------------------------------------------
 * Main Function
 */

/**
 * @brief Main function for system control exploration
 * @return Never returns (infinite loop)
 */
int main(void)
{
    // Initialize all subsystems
    system_init();
    gpio_init();
    exti_init();
    systick_init();

    // Enable global interrupts
    __enable_irq();

    printf("\n🎛️  ARM Cortex-M System Control Explorer Started!\n");
    printf("=====================================================\n");
    printf("- LED1 (PA5): Stack pointer (MSP=solid, PSP=blink)\n");
    printf("- LED2 (PB14): Privilege (Priv=solid, Unpriv=blink)\n");
    printf("- CONTROL LED (PA0): CONTROL register bit pattern\n");
    printf("- Button (PC13): Cycle through system states\n\n");

    printf("System States:\n");
    for (int i = 0; i < SYSTEM_STATE_COUNT; i++) {
        printf("  %d: %s\n", i, get_state_name((system_state_t)i));
    }
    printf("\n");

    // Main loop
    while (1)
    {
        if (state_change_requested) {
            state_change_requested = 0;

            printf("\n--- Mode Switch #%lu ---\n", mode_switch_count);

            switch_to_state(current_state);
            demonstrate_stack_switch();
            demonstrate_privilege_switch();
            analyze_stack_frame();

            printf("Press button to continue to next state...\n");
        }

        // Sleep until next event
        __WFI();
    }

    return 0;  // Never reached
}
