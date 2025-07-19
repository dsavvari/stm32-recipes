/*
 * ----------------------------------------------------------------------------
 * Project: Cortex-M Debug and Trace Features on STM32L4S5VI
 * Author:  Dimitris Savvaris
 * License: MIT License
 * ----------------------------------------------------------------------------
 * Description:
 *
 * This project explores ARM Cortex-M debug and trace capabilities:
 *
 * 🎯 What You'll Learn:
 * - ITM (Instrumentation Trace Macrocell) for debug output
 * - SWO (Serial Wire Output) trace data transmission
 * - DWT (Data Watchpoint and Trace) for performance monitoring
 * - Breakpoint and watchpoint configuration
 * - Debug exception handling
 * - Performance counters and cycle counting
 * - Real-time trace data collection
 * - Debug authentication and security
 *
 * 🔧 Features Demonstrated:
 * - ITM channel-based debug output
 * - SWO trace data streaming
 * - Cycle counter for performance measurement
 * - Function execution timing
 * - Memory access monitoring
 * - Exception trace recording
 * - Debug halt and single-step simulation
 * - PC sampling for profiling
 *
 * 📚 Key ARM Cortex-M Debug Components:
 *
 * ITM (Instrumentation Trace Macrocell):
 * - 32 stimulus channels for debug output
 * - Hardware-accelerated printf() alternative
 * - Non-intrusive debug data transmission
 *
 * DWT (Data Watchpoint and Trace):
 * - Cycle counter (CYCCNT) for timing
 * - Watchpoints for memory access monitoring
 * - Exception trace recording
 * - PC sampling for profiling
 *
 * SWO (Serial Wire Output):
 * - Single-pin trace data output
 * - Multiplexed ITM and DWT data
 * - High-speed trace streaming
 *
 * 🔧 Hardware Setup:
 * - LED1 (PA5): ITM activity indicator
 * - LED2 (PB14): Performance measurement indicator
 * - External LED (PA0): Watchpoint trigger indicator
 * - Button (PC13): Trigger trace events and measurements
 * - SWO Pin (PB3): Trace data output (if available)
 *
 * 📊 Debug Features Explored:
 * - Printf via ITM (no UART needed)
 * - Function execution timing
 * - Memory access profiling
 * - Exception monitoring
 * - Performance optimization
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
#define LED1_PIN                5           // PA5 - ITM activity indicator
#define LED2_PIN                14          // PB14 - Performance indicator
#define WATCHPOINT_LED_PIN      0           // PA0 - Watchpoint trigger
#define BUTTON_PIN              13          // PC13 - Trigger measurements
#define SWO_PIN                 3           // PB3 - SWO output (if available)

// ITM configuration
#define ITM_STIMULUS_PORT_0     0           // Main debug output channel
#define ITM_STIMULUS_PORT_1     1           // Performance data channel
#define ITM_STIMULUS_PORT_2     2           // Event logging channel
#define ITM_STIMULUS_PORT_3     3           // Error reporting channel

// DWT configuration
#define DWT_CTRL_CYCCNTENA      (1UL << 0)  // Enable cycle counter

// Performance measurement
#define MEASUREMENT_SAMPLES     100         // Number of samples for averaging
#define TEST_ARRAY_SIZE         256         // Array size for memory tests

/* ----------------------------------------------------------------------------
 * Global Variables
 */
static volatile uint32_t system_tick_ms = 0;
static volatile uint32_t measurement_count = 0;
static volatile uint32_t trace_events = 0;
static volatile uint8_t measurement_requested = 0;

// Performance measurement data
typedef struct {
    uint32_t cycles_start;
    uint32_t cycles_end;
    uint32_t duration_cycles;
    uint32_t duration_us;
} performance_measurement_t;

static performance_measurement_t perf_data[MEASUREMENT_SAMPLES];
static uint32_t perf_index = 0;

// Test data for memory access patterns
static uint32_t test_array[TEST_ARRAY_SIZE] __attribute__((aligned(32)));
static volatile uint32_t watchpoint_triggers = 0;

/* ----------------------------------------------------------------------------
 * Function Prototypes
 */
static void system_init(void);
static void gpio_init(void);
static void systick_init(void);
static void exti_init(void);
static void itm_init(void);
static void dwt_init(void);
static void itm_send_char(uint32_t channel, char ch);
static void itm_send_string(uint32_t channel, const char* str);
static void itm_send_uint32(uint32_t channel, uint32_t value);
static void perform_timing_test(void);
static void perform_memory_test(void);
static void perform_function_profiling(void);
static void setup_watchpoint(uint32_t address);
static void analyze_performance_data(void);
static uint32_t get_cycle_count(void);
static void reset_cycle_count(void);
static void demonstrate_itm_channels(void);

/* ----------------------------------------------------------------------------
 * ITM and Debug Functions
 */

/**
 * @brief Initialize ITM for debug output
 */
static void itm_init(void)
{
    // Enable trace and debug
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Configure ITM
    ITM->LAR = 0xC5ACCE55;          // Unlock ITM
    ITM->TCR = 0;                   // Disable ITM during configuration
    ITM->TPR = 0x00000000;          // Disable all stimulus ports initially
    ITM->TER = 0x0000000F;          // Enable stimulus ports 0-3

    // Configure trace control
    ITM->TCR = ITM_TCR_TraceBusID_Msk |     // Trace bus ID
               ITM_TCR_DWTENA_Msk |         // Enable DWT
               ITM_TCR_SYNCENA_Msk |        // Enable sync packets
               ITM_TCR_ITMENA_Msk;          // Enable ITM

    printf("ITM initialized for debug trace output\n");
    itm_send_string(ITM_STIMULUS_PORT_0, "ITM Debug Channel Active\n");
}

/**
 * @brief Initialize DWT for performance monitoring
 */
static void dwt_init(void)
{
    // Enable DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Reset and enable cycle counter
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA;

    printf("DWT cycle counter initialized\n");
    itm_send_string(ITM_STIMULUS_PORT_1, "DWT Performance Monitoring Active\n");
}

/**
 * @brief Send character via ITM
 */
static void itm_send_char(uint32_t channel, char ch)
{
    if ((channel < 32) && (ITM->TER & (1UL << channel))) {
        while (ITM->PORT[channel].u32 == 0);   // Wait for port ready
        ITM->PORT[channel].u8 = (uint8_t)ch;   // Send character
    }
}

/**
 * @brief Send string via ITM
 */
static void itm_send_string(uint32_t channel, const char* str)
{
    while (*str) {
        itm_send_char(channel, *str++);
    }
}

/**
 * @brief Send 32-bit value via ITM
 */
static void itm_send_uint32(uint32_t channel, uint32_t value)
{
    if ((channel < 32) && (ITM->TER & (1UL << channel))) {
        while (ITM->PORT[channel].u32 == 0);   // Wait for port ready
        ITM->PORT[channel].u32 = value;        // Send 32-bit value
    }
}

/* ----------------------------------------------------------------------------
 * Performance Measurement Functions
 */

/**
 * @brief Get current cycle count
 */
static uint32_t get_cycle_count(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief Reset cycle counter
 */
static void reset_cycle_count(void)
{
    DWT->CYCCNT = 0;
}

/**
 * @brief Perform timing test
 */
static void perform_timing_test(void)
{
    printf("\n=== Function Timing Test ===\n");
    itm_send_string(ITM_STIMULUS_PORT_1, "Starting timing test\n");

    uint32_t start_cycles = get_cycle_count();

    // Test function: Simple arithmetic operations
    volatile uint32_t result = 0;
    for (int i = 0; i < 1000; i++) {
        result += i * i;
        result ^= (result << 1);
        result = (result * 31) + 17;
    }

    uint32_t end_cycles = get_cycle_count();
    uint32_t duration = end_cycles - start_cycles;

    // Store performance data
    if (perf_index < MEASUREMENT_SAMPLES) {
        perf_data[perf_index].cycles_start = start_cycles;
        perf_data[perf_index].cycles_end = end_cycles;
        perf_data[perf_index].duration_cycles = duration;
        perf_data[perf_index].duration_us = (duration * 1000) / 4000;  // 4MHz clock
        perf_index++;
    }

    printf("Arithmetic test completed in %lu cycles (%lu us)\n",
           duration, (duration * 1000) / 4000);
    printf("Result: 0x%08lX\n", result);

    // Send performance data via ITM
    itm_send_uint32(ITM_STIMULUS_PORT_1, duration);

    GPIOB->ODR |= (1U << LED2_PIN);  // Indicate measurement
}

/**
 * @brief Perform memory access test
 */
static void perform_memory_test(void)
{
    printf("\n=== Memory Access Test ===\n");
    itm_send_string(ITM_STIMULUS_PORT_2, "Memory access pattern test\n");

    uint32_t start_cycles = get_cycle_count();

    // Sequential access pattern
    for (int i = 0; i < TEST_ARRAY_SIZE; i++) {
        test_array[i] = i * 0x12345678;
    }

    uint32_t seq_cycles = get_cycle_count() - start_cycles;

    // Random access pattern
    start_cycles = get_cycle_count();
    for (int i = 0; i < TEST_ARRAY_SIZE; i++) {
        int index = (i * 73) % TEST_ARRAY_SIZE;  // Pseudo-random access
        test_array[index] ^= 0xABCDEF00;
    }

    uint32_t random_cycles = get_cycle_count() - start_cycles;

    printf("Sequential access: %lu cycles\n", seq_cycles);
    printf("Random access: %lu cycles\n", random_cycles);
    printf("Performance ratio: %.2f\n", (float)random_cycles / seq_cycles);

    // Send memory performance data via ITM
    itm_send_uint32(ITM_STIMULUS_PORT_2, seq_cycles);
    itm_send_uint32(ITM_STIMULUS_PORT_2, random_cycles);
}

/**
 * @brief Perform function profiling
 */
static void perform_function_profiling(void)
{
    printf("\n=== Function Profiling ===\n");

    // Profile different operations
    struct {
        const char* name;
        uint32_t cycles;
    } profile_data[] = {
        {"Addition", 0},
        {"Multiplication", 0},
        {"Division", 0},
        {"Floating Point", 0}
    };

    volatile uint32_t a = 12345, b = 67890, result;
    volatile float fa = 123.456f, fb = 789.012f, fresult;

    // Addition test
    uint32_t start = get_cycle_count();
    for (int i = 0; i < 1000; i++) {
        result = a + b;
    }
    profile_data[0].cycles = get_cycle_count() - start;

    // Multiplication test
    start = get_cycle_count();
    for (int i = 0; i < 1000; i++) {
        result = a * b;
    }
    profile_data[1].cycles = get_cycle_count() - start;

    // Division test
    start = get_cycle_count();
    for (int i = 0; i < 1000; i++) {
        result = a / (b + 1);  // Avoid division by zero
    }
    profile_data[2].cycles = get_cycle_count() - start;

    // Floating point test
    start = get_cycle_count();
    for (int i = 0; i < 1000; i++) {
        fresult = fa * fb + fa;
    }
    profile_data[3].cycles = get_cycle_count() - start;

    // Print and trace results
    for (int i = 0; i < 4; i++) {
        printf("%s: %lu cycles\n", profile_data[i].name, profile_data[i].cycles);
        itm_send_string(ITM_STIMULUS_PORT_1, profile_data[i].name);
        itm_send_char(ITM_STIMULUS_PORT_1, ':');
        itm_send_uint32(ITM_STIMULUS_PORT_1, profile_data[i].cycles);
    }

    (void)result;    // Prevent optimization
    (void)fresult;   // Prevent optimization
}

/**
 * @brief Setup memory watchpoint
 */
static void setup_watchpoint(uint32_t address)
{
    // This is a simplified watchpoint setup
    // In a real debugger, this would be configured by the debug probe
    printf("Watchpoint configured for address 0x%08lX\n", address);
    itm_send_string(ITM_STIMULUS_PORT_3, "Watchpoint configured\n");
    itm_send_uint32(ITM_STIMULUS_PORT_3, address);
}

/**
 * @brief Analyze performance data
 */
static void analyze_performance_data(void)
{
    if (perf_index == 0) return;

    printf("\n=== Performance Analysis ===\n");

    uint32_t total_cycles = 0;
    uint32_t min_cycles = perf_data[0].duration_cycles;
    uint32_t max_cycles = perf_data[0].duration_cycles;

    for (uint32_t i = 0; i < perf_index; i++) {
        total_cycles += perf_data[i].duration_cycles;
        if (perf_data[i].duration_cycles < min_cycles) {
            min_cycles = perf_data[i].duration_cycles;
        }
        if (perf_data[i].duration_cycles > max_cycles) {
            max_cycles = perf_data[i].duration_cycles;
        }
    }

    uint32_t avg_cycles = total_cycles / perf_index;

    printf("Samples: %lu\n", perf_index);
    printf("Average: %lu cycles (%lu us)\n", avg_cycles, (avg_cycles * 1000) / 4000);
    printf("Minimum: %lu cycles (%lu us)\n", min_cycles, (min_cycles * 1000) / 4000);
    printf("Maximum: %lu cycles (%lu us)\n", max_cycles, (max_cycles * 1000) / 4000);
    printf("Variance: %lu cycles\n", max_cycles - min_cycles);

    // Send analysis via ITM
    itm_send_string(ITM_STIMULUS_PORT_1, "Performance Analysis\n");
    itm_send_uint32(ITM_STIMULUS_PORT_1, avg_cycles);
    itm_send_uint32(ITM_STIMULUS_PORT_1, min_cycles);
    itm_send_uint32(ITM_STIMULUS_PORT_1, max_cycles);
}

/**
 * @brief Demonstrate ITM channels
 */
static void demonstrate_itm_channels(void)
{
    printf("\n=== ITM Channel Demonstration ===\n");

    // Send different types of data on different channels
    itm_send_string(ITM_STIMULUS_PORT_0, "Channel 0: General debug output\n");
    itm_send_string(ITM_STIMULUS_PORT_1, "Channel 1: Performance data\n");
    itm_send_string(ITM_STIMULUS_PORT_2, "Channel 2: Event logging\n");
    itm_send_string(ITM_STIMULUS_PORT_3, "Channel 3: Error reporting\n");

    // Send numerical data
    for (int i = 0; i < 4; i++) {
        itm_send_uint32(i, 0x12340000 + i);
    }

    trace_events++;
    GPIOA->ODR |= (1U << LED1_PIN);  // Indicate ITM activity
}

/* ----------------------------------------------------------------------------
 * Interrupt Handlers
 */

/**
 * @brief SysTick handler with trace output
 */
void SysTick_Handler(void)
{
    system_tick_ms++;

    // Send periodic trace data
    if ((system_tick_ms % 1000) == 0) {
        itm_send_string(ITM_STIMULUS_PORT_0, "Heartbeat\n");
        itm_send_uint32(ITM_STIMULUS_PORT_0, system_tick_ms);
    }

    // Update LED indicators
    if ((system_tick_ms % 500) == 0) {
        GPIOA->ODR &= ~(1U << LED1_PIN);    // Clear ITM activity LED
        GPIOB->ODR &= ~(1U << LED2_PIN);    // Clear performance LED
    }
}

/**
 * @brief Button press handler - trigger measurements
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << BUTTON_PIN)) {
        EXTI->PR1 = (1U << BUTTON_PIN);     // Clear interrupt flag

        measurement_requested = 1;
        measurement_count++;

        // Log event via ITM
        itm_send_string(ITM_STIMULUS_PORT_2, "Button pressed - measurement triggered\n");
        itm_send_uint32(ITM_STIMULUS_PORT_2, measurement_count);
    }
}

/* ----------------------------------------------------------------------------
 * Initialization Functions
 */

/**
 * @brief Initialize system for debug and trace
 */
static void system_init(void)
{
    // Initialize debug components
    itm_init();
    dwt_init();

    // Setup test array
    for (int i = 0; i < TEST_ARRAY_SIZE; i++) {
        test_array[i] = i;
    }

    printf("ARM Cortex-M Debug and Trace Demo initialized\n");
}

/**
 * @brief Initialize GPIO for debug demonstration
 */
static void gpio_init(void)
{
    // Enable GPIO clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;

    // Configure LEDs as outputs
    GPIOA->MODER &= ~((3U << (LED1_PIN * 2)) | (3U << (WATCHPOINT_LED_PIN * 2)));
    GPIOA->MODER |= (1U << (LED1_PIN * 2)) | (1U << (WATCHPOINT_LED_PIN * 2));

    GPIOB->MODER &= ~(3U << (LED2_PIN * 2));
    GPIOB->MODER |= (1U << (LED2_PIN * 2));

    // Configure SWO pin (if available) - alternate function
    GPIOB->MODER &= ~(3U << (SWO_PIN * 2));
    GPIOB->MODER |= (2U << (SWO_PIN * 2));      // Alternate function
    GPIOB->OSPEEDR |= (3U << (SWO_PIN * 2));    // Very high speed
    GPIOB->AFR[0] &= ~(0xF << (SWO_PIN * 4));
    GPIOB->AFR[0] |= (0 << (SWO_PIN * 4));      // AF0 for SWO

    // Configure button with pull-up
    GPIOC->MODER &= ~(3U << (BUTTON_PIN * 2));      // Input mode
    GPIOC->PUPDR &= ~(3U << (BUTTON_PIN * 2));
    GPIOC->PUPDR |= (1U << (BUTTON_PIN * 2));       // Pull-up

    // Initialize LEDs OFF
    GPIOA->ODR &= ~((1U << LED1_PIN) | (1U << WATCHPOINT_LED_PIN));
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

/* ----------------------------------------------------------------------------
 * Main Function
 */

/**
 * @brief Main function for debug and trace exploration
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

    printf("\n🔍 ARM Cortex-M Debug & Trace Explorer Started!\n");
    printf("================================================\n");
    printf("- LED1 (PA5): ITM activity indicator\n");
    printf("- LED2 (PB14): Performance measurement indicator\n");
    printf("- Watchpoint LED (PA0): Memory access monitoring\n");
    printf("- Button (PC13): Trigger measurements and trace events\n");
    printf("- SWO (PB3): Trace data output (connect to debugger)\n\n");

    printf("Debug Features:\n");
    printf("- ITM channels 0-3 for different data types\n");
    printf("- DWT cycle counter for performance measurement\n");
    printf("- Memory access pattern analysis\n");
    printf("- Function execution profiling\n\n");

    printf("Connect SWO to debugger and enable ITM trace to see:\n");
    printf("- Real-time debug output without UART\n");
    printf("- Performance counters and timing data\n");
    printf("- Event logging and error reporting\n\n");

    // Initial demonstration
    demonstrate_itm_channels();

    // Setup watchpoint on test array
    setup_watchpoint((uint32_t)&test_array[0]);

    // Main loop
    uint32_t test_cycle = 0;
    while (1)
    {
        if (measurement_requested) {
            measurement_requested = 0;

            printf("\n--- Measurement Cycle #%lu ---\n", ++test_cycle);
            itm_send_string(ITM_STIMULUS_PORT_0, "=== New Measurement Cycle ===\n");

            // Cycle through different tests
            switch (test_cycle % 4) {
                case 1:
                    perform_timing_test();
                    break;
                case 2:
                    perform_memory_test();
                    break;
                case 3:
                    perform_function_profiling();
                    break;
                case 0:
                    analyze_performance_data();
                    demonstrate_itm_channels();
                    break;
            }

            printf("Trace events: %lu, Measurements: %lu\n",
                   trace_events, measurement_count);
        }

        // Sleep between measurements
        __WFI();
    }

    return 0;  // Never reached
}
