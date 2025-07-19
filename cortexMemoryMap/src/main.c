/*
 * ----------------------------------------------------------------------------
 * Project: Cortex-M Memory Map Exploration on STM32L4S5VI
 * Author:  Dimitris Savvaris
 * License: MIT License
 * ----------------------------------------------------------------------------
 * Description:
 *
 * This project explores the ARM Cortex-M memory architecture and memory map:
 *
 * 🧠 What You'll Learn:
 * - ARM Cortex-M memory map regions and their purposes
 * - Stack pointer (MSP vs PSP) management
 * - Memory protection unit (MPU) basics
 * - Memory barriers and cache coherency
 * - Linker script understanding
 * - Stack overflow detection
 * - Memory debugging techniques
 *
 * 🎯 Features Demonstrated:
 * - Memory region identification and access
 * - Stack pointer switching and monitoring
 * - Memory protection violations (controlled)
 * - Stack usage measurement
 * - Memory layout visualization
 * - Pointer arithmetic and address validation
 *
 * 🔧 Hardware Setup:
 * - LED1 (PA5): Memory access indicator
 * - LED2 (PB14): Stack operation indicator
 * - External LED (PA0): Memory protection violation indicator
 * - Button (PC13): Trigger memory operations
 * - UART output for memory map information
 *
 * 📚 Cortex-M Memory Map (Generic):
 * Region               | Start      | End        | Purpose
 * ---------------------|------------|------------|------------------
 * Code                 | 0x00000000 | 0x1FFFFFFF | Flash, ROM, Vectors
 * SRAM                 | 0x20000000 | 0x3FFFFFFF | Data, Stack, Heap
 * Peripheral           | 0x40000000 | 0x5FFFFFFF | APB, AHB Peripherals
 * External RAM         | 0x60000000 | 0x9FFFFFFF | External memory
 * External Device      | 0xA0000000 | 0xDFFFFFFF | External devices
 * System               | 0xE0000000 | 0xFFFFFFFF | Cortex-M system
 *
 * 🔍 STM32L4S5VI Specific Memory:
 * - Flash: 0x08000000 - 0x081FFFFF (2MB)
 * - SRAM1: 0x20000000 - 0x2004FFFF (320KB)
 * - SRAM2: 0x20050000 - 0x2006FFFF (128KB)
 * - System: 0x1FFF0000 - 0x1FFF7FFF (System memory)
 *
 * ----------------------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32l4s5xx.h"

/* ----------------------------------------------------------------------------
 * Constants and Definitions
 */

// GPIO definitions
#define LED1_PIN                5           // PA5 - Memory access indicator
#define LED2_PIN                14          // PB14 - Stack operation indicator
#define PROTECTION_LED_PIN      0           // PA0 - Memory protection indicator
#define BUTTON_PIN              13          // PC13 - Operation trigger

// Memory region definitions (Cortex-M standard)
#define CODE_REGION_START       0x00000000UL
#define CODE_REGION_END         0x1FFFFFFFUL
#define SRAM_REGION_START       0x20000000UL
#define SRAM_REGION_END         0x3FFFFFFFUL
#define PERIPHERAL_REGION_START 0x40000000UL
#define PERIPHERAL_REGION_END   0x5FFFFFFFUL
#define SYSTEM_REGION_START     0xE0000000UL
#define SYSTEM_REGION_END       0xFFFFFFFFUL

// STM32L4S5VI specific memory regions
#define FLASH_START             0x08000000UL
#define FLASH_SIZE              (2 * 1024 * 1024)      // 2MB
#define SRAM1_START             0x20000000UL
#define SRAM1_SIZE              (320 * 1024)           // 320KB
#define SRAM2_START             0x20050000UL
#define SRAM2_SIZE              (128 * 1024)           // 128KB
#define SYSTEM_MEMORY_START     0x1FFF0000UL
#define SYSTEM_MEMORY_SIZE      (32 * 1024)            // 32KB

// Stack monitoring
#define STACK_GUARD_PATTERN     0xDEADBEEF
#define STACK_USAGE_THRESHOLD   70          // Alert when 70% stack used

/* ----------------------------------------------------------------------------
 * Memory Layout Structure
 */
typedef struct {
    uint32_t start_address;
    uint32_t size;
    const char* name;
    const char* description;
    uint8_t readable;
    uint8_t writable;
    uint8_t executable;
} memory_region_t;

/* ----------------------------------------------------------------------------
 * Global Variables
 */
static volatile uint32_t system_tick_ms = 0;
static volatile uint32_t memory_access_count = 0;
static volatile uint32_t stack_operations = 0;
static volatile uint8_t memory_test_requested = 0;

// Stack monitoring variables
static uint32_t stack_start_address;
static uint32_t stack_size;
static uint32_t max_stack_usage = 0;

// Memory test arrays in different regions
static uint32_t sram1_test_array[256] __attribute__((section(".data")));
static const uint32_t flash_test_array[256] __attribute__((section(".rodata"))) = {1,2,3,4,5};

/* ----------------------------------------------------------------------------
 * Memory Region Definitions
 */
static const memory_region_t memory_regions[] = {
    // ARM Cortex-M standard regions
    {CODE_REGION_START,     0x20000000, "Code Region",      "Flash, ROM, Vector Table", 1, 0, 1},
    {SRAM_REGION_START,     0x20000000, "SRAM Region",      "Data, Stack, Heap",         1, 1, 0},
    {PERIPHERAL_REGION_START, 0x20000000, "Peripheral",     "APB/AHB Peripherals",       1, 1, 0},
    {SYSTEM_REGION_START,   0x20000000, "System Region",   "Cortex-M System Control",   1, 1, 0},

    // STM32L4S5VI specific regions
    {FLASH_START,           FLASH_SIZE,  "Flash Memory",    "Program Code + Constants",  1, 0, 1},
    {SRAM1_START,           SRAM1_SIZE,  "SRAM1",           "Main RAM Bank 1",           1, 1, 0},
    {SRAM2_START,           SRAM2_SIZE,  "SRAM2",           "Main RAM Bank 2",           1, 1, 0},
    {SYSTEM_MEMORY_START,   SYSTEM_MEMORY_SIZE, "System Memory", "Bootloader ROM",      1, 0, 1},

    // System Control Block regions
    {0xE000E000,            0x1000,      "SCB",             "System Control Block",     1, 1, 0},
    {0xE000E010,            0x10,        "SysTick",         "System Tick Timer",        1, 1, 0},
    {0xE000E100,            0x300,       "NVIC",            "Interrupt Controller",     1, 1, 0},
    {0xE000ED00,            0x100,       "SCB Registers",   "System Control Registers", 1, 1, 0},
};

#define NUM_MEMORY_REGIONS (sizeof(memory_regions) / sizeof(memory_region_t))

/* ----------------------------------------------------------------------------
 * Function Prototypes
 */
static void system_init(void);
static void gpio_init(void);
static void systick_init(void);
static void exti_init(void);
static void stack_init_monitoring(void);
static void print_memory_map(void);
static void test_memory_regions(void);
static void test_stack_operations(void);
static void measure_stack_usage(void);
static uint8_t is_address_valid(uint32_t address);
static const char* get_memory_region_name(uint32_t address);
static void demonstrate_pointer_arithmetic(void);
static void stack_overflow_test(void);

/* ----------------------------------------------------------------------------
 * Interrupt Handlers
 */

/**
 * @brief SysTick handler for system timing
 */
void SysTick_Handler(void)
{
    system_tick_ms++;

    // Periodic stack usage monitoring
    if ((system_tick_ms % 1000) == 0) {  // Every second
        measure_stack_usage();
    }
}

/**
 * @brief Button press handler - trigger memory tests
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << BUTTON_PIN)) {
        EXTI->PR1 = (1U << BUTTON_PIN);     // Clear interrupt flag

        memory_test_requested = 1;

        // Indicate activity
        GPIOA->ODR ^= (1U << LED1_PIN);
    }
}

/**
 * @brief Hard Fault handler with memory debugging
 */
void HardFault_Handler(void)
{
    // Turn on protection LED to indicate fault
    GPIOA->ODR |= (1U << PROTECTION_LED_PIN);

    // In a real application, analyze fault registers:
    // - SCB->HFSR: Hard Fault Status Register
    // - SCB->CFSR: Configurable Fault Status Register
    // - SCB->MMFAR: Memory Management Fault Address Register
    // - SCB->BFAR: Bus Fault Address Register

    uint32_t hfsr = SCB->HFSR;
    uint32_t cfsr = SCB->CFSR;
    uint32_t mmfar = SCB->MMFAR;
    uint32_t bfar = SCB->BFAR;

    printf("Hard Fault Occurred!\n");
    printf("HFSR: 0x%08lX\n", hfsr);
    printf("CFSR: 0x%08lX\n", cfsr);

    if (cfsr & SCB_CFSR_MMARVALID_Msk) {
        printf("Memory Management Fault Address: 0x%08lX\n", mmfar);
    }

    if (cfsr & SCB_CFSR_BFARVALID_Msk) {
        printf("Bus Fault Address: 0x%08lX\n", bfar);
    }

    while (1) {
        // Blink LEDs to indicate fault state
        for (volatile uint32_t i = 0; i < 500000; i++) __NOP();
        GPIOA->ODR ^= (1U << LED1_PIN) | (1U << PROTECTION_LED_PIN);
        GPIOB->ODR ^= (1U << LED2_PIN);
    }
}

/* ----------------------------------------------------------------------------
 * Initialization Functions
 */

/**
 * @brief Initialize system for memory exploration
 */
static void system_init(void)
{
    // Enable usage fault, bus fault, and memory management fault
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk |
                  SCB_SHCSR_BUSFAULTENA_Msk |
                  SCB_SHCSR_MEMFAULTENA_Msk;

    // Initialize stack monitoring
    stack_init_monitoring();
}

/**
 * @brief Initialize GPIO for memory demonstration
 */
static void gpio_init(void)
{
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // Configure LEDs as outputs
    GPIOA->MODER &= ~((3U << (LED1_PIN * 2)) | (3U << (PROTECTION_LED_PIN * 2)));
    GPIOA->MODER |= (1U << (LED1_PIN * 2)) | (1U << (PROTECTION_LED_PIN * 2));

    GPIOB->MODER &= ~(3U << (LED2_PIN * 2));
    GPIOB->MODER |= (1U << (LED2_PIN * 2));

    // Configure button with pull-up
    GPIOC->MODER &= ~(3U << (BUTTON_PIN * 2));      // Input mode
    GPIOC->PUPDR &= ~(3U << (BUTTON_PIN * 2));
    GPIOC->PUPDR |= (1U << (BUTTON_PIN * 2));       // Pull-up

    // Initialize LEDs OFF
    GPIOA->ODR &= ~((1U << LED1_PIN) | (1U << PROTECTION_LED_PIN));
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
 * @brief Initialize stack monitoring
 */
static void stack_init_monitoring(void)
{
    // Get current stack pointer
    uint32_t current_sp = __get_MSP();

    // Estimate stack start (this is simplified - in real apps use linker symbols)
    stack_start_address = SRAM1_START + SRAM1_SIZE;  // Assume stack grows down from top of SRAM1
    stack_size = 4096;  // Assume 4KB stack (adjust based on linker script)

    printf("Stack Monitoring Initialized:\n");
    printf("Current SP: 0x%08lX\n", current_sp);
    printf("Estimated Stack Start: 0x%08lX\n", stack_start_address);
    printf("Estimated Stack Size: %lu bytes\n", stack_size);
}

/* ----------------------------------------------------------------------------
 * Memory Exploration Functions
 */

/**
 * @brief Print complete memory map
 */
static void print_memory_map(void)
{
    printf("\n=== ARM Cortex-M Memory Map ===\n");
    printf("Region               Start      End        Size       R W X Description\n");
    printf("-------------------- ---------- ---------- ---------- - - - -----------\n");

    for (size_t i = 0; i < NUM_MEMORY_REGIONS; i++) {
        const memory_region_t* region = &memory_regions[i];
        printf("%-20s 0x%08lX 0x%08lX %8lu %c %c %c %s\n",
               region->name,
               region->start_address,
               region->start_address + region->size - 1,
               region->size,
               region->readable ? 'R' : '-',
               region->writable ? 'W' : '-',
               region->executable ? 'X' : '-',
               region->description);
    }
    printf("\n");
}

/**
 * @brief Test memory region access
 */
static void test_memory_regions(void)
{
    printf("=== Memory Region Access Tests ===\n");

    // Test 1: Read from Flash (should work)
    printf("Test 1: Reading from Flash...");
    uint32_t flash_value = *(volatile uint32_t*)FLASH_START;
    printf(" Value: 0x%08lX ✓\n", flash_value);

    // Test 2: Read from SRAM1 (should work)
    printf("Test 2: Reading from SRAM1...");
    uint32_t sram1_value = *(volatile uint32_t*)SRAM1_START;
    printf(" Value: 0x%08lX ✓\n", sram1_value);

    // Test 3: Write to SRAM1 (should work)
    printf("Test 3: Writing to SRAM1...");
    *(volatile uint32_t*)SRAM1_START = 0x12345678;
    uint32_t verify_value = *(volatile uint32_t*)SRAM1_START;
    printf(" Written: 0x12345678, Read: 0x%08lX %s\n",
           verify_value, (verify_value == 0x12345678) ? "✓" : "✗");

    // Test 4: Access System Control Block
    printf("Test 4: Reading SCB CPUID...");
    uint32_t cpuid = SCB->CPUID;
    printf(" CPUID: 0x%08lX ✓\n", cpuid);

    // Test 5: Access SysTick registers
    printf("Test 5: Reading SysTick...");
    uint32_t systick_ctrl = SysTick->CTRL;
    printf(" SysTick CTRL: 0x%08lX ✓\n", systick_ctrl);

    // Test 6: Demonstrate array access in different regions
    printf("Test 6: Array access tests...\n");

    // SRAM array
    sram1_test_array[0] = 0xAABBCCDD;
    printf("  SRAM1 array[0] = 0x%08lX at address 0x%08lX\n",
           sram1_test_array[0], (uint32_t)&sram1_test_array[0]);

    // Flash array (read-only)
    printf("  Flash array[0] = 0x%08lX at address 0x%08lX\n",
           flash_test_array[0], (uint32_t)&flash_test_array[0]);

    memory_access_count++;
    GPIOA->ODR |= (1U << LED1_PIN);  // Indicate successful memory access
}

/**
 * @brief Test stack operations
 */
static void test_stack_operations(void)
{
    printf("=== Stack Operations Test ===\n");

    // Get current stack pointer
    uint32_t sp_before = __get_MSP();
    printf("Stack pointer before: 0x%08lX\n", sp_before);

    // Allocate local variables (uses stack)
    volatile uint32_t local_array[64];  // 256 bytes on stack

    // Fill array to ensure it's actually allocated
    for (int i = 0; i < 64; i++) {
        local_array[i] = i * 0x12345678;
    }

    uint32_t sp_after = __get_MSP();
    printf("Stack pointer after allocation: 0x%08lX\n", sp_after);
    printf("Stack used for local array: %ld bytes\n", sp_before - sp_after);

    // Demonstrate stack frame
    printf("Local array address: 0x%08lX\n", (uint32_t)local_array);
    printf("Local array[0] = 0x%08lX\n", local_array[0]);

    stack_operations++;
    GPIOB->ODR |= (1U << LED2_PIN);  // Indicate stack operation

    // Measure stack usage
    measure_stack_usage();
}

/**
 * @brief Measure current stack usage
 */
static void measure_stack_usage(void)
{
    uint32_t current_sp = __get_MSP();
    uint32_t stack_used = stack_start_address - current_sp;
    uint32_t stack_usage_percent = (stack_used * 100) / stack_size;

    if (stack_used > max_stack_usage) {
        max_stack_usage = stack_used;
    }

    if (stack_usage_percent > STACK_USAGE_THRESHOLD) {
        printf("⚠️  Stack usage warning: %lu%% (%lu/%lu bytes)\n",
               stack_usage_percent, stack_used, stack_size);
        GPIOA->ODR |= (1U << PROTECTION_LED_PIN);  // Warning indicator
    }

    // Print stack stats every 10 measurements
    static uint32_t measurement_count = 0;
    measurement_count++;

    if ((measurement_count % 10) == 0) {
        printf("Stack Stats: Current: %lu bytes, Max: %lu bytes, Usage: %lu%%\n",
               stack_used, max_stack_usage, stack_usage_percent);
    }
}

/**
 * @brief Check if an address is valid/accessible
 */
static uint8_t is_address_valid(uint32_t address)
{
    // Check against known memory regions
    for (size_t i = 0; i < NUM_MEMORY_REGIONS; i++) {
        const memory_region_t* region = &memory_regions[i];
        if (address >= region->start_address &&
            address < (region->start_address + region->size)) {
            return region->readable;
        }
    }
    return 0;  // Address not in any known region
}

/**
 * @brief Get memory region name for an address
 */
static const char* get_memory_region_name(uint32_t address)
{
    for (size_t i = 0; i < NUM_MEMORY_REGIONS; i++) {
        const memory_region_t* region = &memory_regions[i];
        if (address >= region->start_address &&
            address < (region->start_address + region->size)) {
            return region->name;
        }
    }
    return "Unknown Region";
}

/**
 * @brief Demonstrate pointer arithmetic and address validation
 */
static void demonstrate_pointer_arithmetic(void)
{
    printf("=== Pointer Arithmetic Demonstration ===\n");

    // Different types of pointers
    uint32_t* sram_ptr = (uint32_t*)SRAM1_START;
    const uint32_t* flash_ptr = (const uint32_t*)FLASH_START;
    uint32_t* peripheral_ptr = (uint32_t*)GPIOA_BASE;

    printf("SRAM pointer: 0x%08lX (%s)\n",
           (uint32_t)sram_ptr, get_memory_region_name((uint32_t)sram_ptr));
    printf("Flash pointer: 0x%08lX (%s)\n",
           (uint32_t)flash_ptr, get_memory_region_name((uint32_t)flash_ptr));
    printf("Peripheral pointer: 0x%08lX (%s)\n",
           (uint32_t)peripheral_ptr, get_memory_region_name((uint32_t)peripheral_ptr));

    // Demonstrate pointer arithmetic
    uint32_t* ptr_offset = sram_ptr + 100;  // Add 100 words (400 bytes)
    printf("SRAM pointer + 100 words: 0x%08lX\n", (uint32_t)ptr_offset);

    // Size calculations
    printf("Size of uint32_t: %zu bytes\n", sizeof(uint32_t));
    printf("Size of pointer: %zu bytes\n", sizeof(void*));
    printf("Distance between pointers: %ld bytes\n",
           (uint8_t*)ptr_offset - (uint8_t*)sram_ptr);

    // Address validation
    printf("Address validation:\n");
    printf("  0x%08lX: %s\n", (uint32_t)sram_ptr,
           is_address_valid((uint32_t)sram_ptr) ? "Valid" : "Invalid");
    printf("  0x%08lX: %s\n", 0x12345678UL,
           is_address_valid(0x12345678UL) ? "Valid" : "Invalid");
}

/**
 * @brief Controlled stack overflow test (for educational purposes)
 */
static void stack_overflow_test(void)
{
    printf("=== Stack Overflow Test (Controlled) ===\n");
    printf("⚠️  This test may trigger a fault - that's expected!\n");

    // Recursive function to consume stack
    static int recursion_count = 0;
    recursion_count++;

    if (recursion_count < 10) {  // Limit recursion to avoid actual overflow
        uint32_t large_array[100];  // 400 bytes per recursion

        // Use the array to prevent optimization
        for (int i = 0; i < 100; i++) {
            large_array[i] = recursion_count * i;
        }

        printf("Recursion level %d, SP: 0x%08lX\n", recursion_count, __get_MSP());

        stack_overflow_test();  // Recurse

        // Use array again to prevent optimization
        printf("Returning from level %d, array[0] = 0x%08lX\n",
               recursion_count, large_array[0]);
    }

    recursion_count--;
}

/* ----------------------------------------------------------------------------
 * Main Function
 */

/**
 * @brief Main function for memory map exploration
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

    printf("\n");
    printf("🧠 Cortex-M Memory Map Explorer Started!\n");
    printf("==========================================\n");
    printf("- LED1 (PA5): Memory access indicator\n");
    printf("- LED2 (PB14): Stack operation indicator\n");
    printf("- Protection LED (PA0): Warning/fault indicator\n");
    printf("- Button (PC13): Trigger memory tests\n\n");

    // Print initial memory map
    print_memory_map();

    // Print system information
    printf("=== System Information ===\n");
    printf("CPUID: 0x%08lX\n", SCB->CPUID);
    printf("MSP: 0x%08lX\n", __get_MSP());
    printf("PSP: 0x%08lX\n", __get_PSP());
    printf("CONTROL: 0x%08lX\n", __get_CONTROL());
    printf("\n");

    // Demonstrate basic pointer operations
    demonstrate_pointer_arithmetic();

    // Main loop
    uint32_t test_cycle = 0;
    while (1)
    {
        if (memory_test_requested) {
            memory_test_requested = 0;

            printf("\n--- Test Cycle %lu ---\n", ++test_cycle);

            // Cycle through different tests
            switch (test_cycle % 4) {
                case 1:
                    test_memory_regions();
                    break;
                case 2:
                    test_stack_operations();
                    break;
                case 3:
                    demonstrate_pointer_arithmetic();
                    break;
                case 0:
                    stack_overflow_test();
                    break;
            }

            printf("Memory accesses: %lu, Stack operations: %lu\n",
                   memory_access_count, stack_operations);

            // Turn off activity LEDs after test
            GPIOA->ODR &= ~(1U << LED1_PIN);
            GPIOB->ODR &= ~(1U << LED2_PIN);
        }

        // Sleep between tests
        __WFI();
    }

    return 0;  // Never reached
}
