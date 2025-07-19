/*
 * ----------------------------------------------------------------------------
 * Project:   Bare-Metal UART Example for STM32L4S5VI (B-L4S5I-IOT01A)
 * Author:    Dimitris Savvaris
 * License:   MIT License (see below)
 * ----------------------------------------------------------------------------
 * Description:
 *   This program demonstrates how to configure and use UART2 on the STM32L4S5VI
 *   for serial communication. The code uses direct register access without any
 *   HAL or CMSIS drivers to configure the system clock, GPIO, and UART peripheral.
 *
 *   Features:
 *     - UART2 configured for 115200 baud, 8N1 (8 data bits, no parity, 1 stop bit)
 *     - PA2 (TX) and PA3 (RX) pins configured for UART2 (AF7)
 *     - Transmit and receive functions with polling
 *     - Echo functionality: receives characters and sends them back
 *     - LED feedback: PA5 (LED1) blinks on each character received
 *     - String transmission functions for easy debugging output
 *
 *   Hardware Connections:
 *     - PA2 (UART2_TX) - Connect to RX of USB-to-Serial adapter
 *     - PA3 (UART2_RX) - Connect to TX of USB-to-Serial adapter
 *     - GND - Connect to GND of USB-to-Serial adapter
 *     - PA5 (LED1) - On-board LED for visual feedback
 *
 *   Usage:
 *     - Connect a USB-to-Serial adapter to PA2/PA3
 *     - Open a terminal at 115200 baud
 *     - Type characters and see them echoed back
 *     - LED1 blinks for each received character
 *
 *   This example is suitable for learning bare-metal STM32 UART programming
 *   and serves as a foundation for more complex serial communication projects.
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
#include <string.h>
#include "stm32l4s5xx.h"

/* ----------------------------------------------------------------------------
 * Global variables for Live Watch debugging
 * These variables can be monitored in real-time during debugging
 * ----------------------------------------------------------------------------
 */
volatile uint32_t g_chars_received = 0;      // Count of characters received
volatile uint32_t g_chars_transmitted = 0;   // Count of characters transmitted
volatile uint8_t g_last_char_received = 0;   // Last character received
volatile uint8_t g_uart_status = 0;          // UART status flags
volatile uint32_t g_led_blink_count = 0;     // LED blink counter

/* ----------------------------------------------------------------------------
 * UART Configuration Constants
 * ----------------------------------------------------------------------------
 */
#define UART_BAUDRATE           115200UL
#define SYSTEM_CLOCK_HZ         4000000UL    // 4MHz MSI clock (default)

// GPIO pin definitions for UART2
#define UART2_TX_PIN            2    // PA2
#define UART2_RX_PIN            3    // PA3
#define UART2_AF                7    // Alternate Function 7 for UART2

// LED pin definition
#define LED1_PIN                5    // PA5

/**
 * ----------------------------------------------------------------------------
 * @brief   Configure system clock to use default MSI (4MHz).
 * @details For simplicity, we'll use the default MSI clock. In a real
 *          application, you might want to configure PLL for higher speed.
 * ----------------------------------------------------------------------------
 */
void system_clock_init(void)
{
    // The STM32L4 starts with MSI at 4MHz by default
    // For this UART example, we'll keep it simple and use the default clock

    // Update the global SystemCoreClock variable
    SystemCoreClockUpdate();
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize GPIO pins for UART2 (PA2=TX, PA3=RX) and LED (PA5).
 * @details Configures PA2 and PA3 as alternate function for UART2,
 *          and PA5 as output for LED feedback.
 * ----------------------------------------------------------------------------
 */
void gpio_init(void)
{
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // Configure PA2 (UART2_TX) as alternate function
    GPIOA->MODER &= ~(0x3 << (UART2_TX_PIN * 2));     // Clear mode bits
    GPIOA->MODER |= (0x2 << (UART2_TX_PIN * 2));      // Set alternate function mode
    GPIOA->OSPEEDR |= (0x3 << (UART2_TX_PIN * 2));    // High speed
    GPIOA->PUPDR &= ~(0x3 << (UART2_TX_PIN * 2));     // No pull-up/pull-down

    // Configure PA3 (UART2_RX) as alternate function
    GPIOA->MODER &= ~(0x3 << (UART2_RX_PIN * 2));     // Clear mode bits
    GPIOA->MODER |= (0x2 << (UART2_RX_PIN * 2));      // Set alternate function mode
    GPIOA->OSPEEDR |= (0x3 << (UART2_RX_PIN * 2));    // High speed
    GPIOA->PUPDR |= (0x1 << (UART2_RX_PIN * 2));      // Pull-up for RX

    // Set alternate function for PA2 and PA3 (AF7 for UART2)
    GPIOA->AFR[0] &= ~(0xF << (UART2_TX_PIN * 4));    // Clear AF bits for PA2
    GPIOA->AFR[0] |= (UART2_AF << (UART2_TX_PIN * 4)); // Set AF7 for PA2
    GPIOA->AFR[0] &= ~(0xF << (UART2_RX_PIN * 4));    // Clear AF bits for PA3
    GPIOA->AFR[0] |= (UART2_AF << (UART2_RX_PIN * 4)); // Set AF7 for PA3

    // Configure PA5 (LED1) as output
    GPIOA->MODER &= ~(0x3 << (LED1_PIN * 2));         // Clear mode bits
    GPIOA->MODER |= (0x1 << (LED1_PIN * 2));          // Set output mode
    GPIOA->OSPEEDR |= (0x3 << (LED1_PIN * 2));        // High speed

    // Turn off LED initially
    GPIOA->ODR &= ~(1 << LED1_PIN);
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize UART2 for 115200 baud, 8N1.
 * @details Configures UART2 registers for transmission and reception.
 * ----------------------------------------------------------------------------
 */
void uart2_init(void)
{
    // Enable UART2 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    // Disable UART2 during configuration
    USART2->CR1 &= ~USART_CR1_UE;

    // Configure baud rate
    // UART_DIV = f_CK / (16 * baud_rate)
    // For 4MHz clock and 115200 baud: 4000000 / (16 * 115200) = ~2.17
    // BRR = 2 + 0.17 * 16 = 2 + 2.72 ≈ 35 (0x23)
    // More precise: 4000000 / 115200 = 34.72, so BRR = 35
    uint32_t uart_div = (SYSTEM_CLOCK_HZ + (UART_BAUDRATE / 2)) / UART_BAUDRATE;
    USART2->BRR = uart_div;

    // Configure UART: 8 data bits, no parity, 1 stop bit
    USART2->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);    // 8 data bits
    USART2->CR1 &= ~USART_CR1_PCE;                     // No parity
    USART2->CR2 &= ~USART_CR2_STOP;                    // 1 stop bit

    // Enable transmitter and receiver
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

    // Enable UART2
    USART2->CR1 |= USART_CR1_UE;

    // Wait for UART to be ready
    while (!(USART2->ISR & USART_ISR_TEACK));  // Wait for transmitter enable
    while (!(USART2->ISR & USART_ISR_REACK));  // Wait for receiver enable
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Transmit a single character via UART2.
 * @param   ch Character to transmit
 * ----------------------------------------------------------------------------
 */
void uart2_transmit_char(uint8_t ch)
{
    // Wait for transmit data register to be empty
    while (!(USART2->ISR & USART_ISR_TXE_TXFNF));

    // Write character to transmit data register
    USART2->TDR = ch;

    // Update global counter for Live Watch
    g_chars_transmitted++;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Receive a single character via UART2.
 * @return  Received character
 * ----------------------------------------------------------------------------
 */
uint8_t uart2_receive_char(void)
{
    // Wait for receive data register to have data
    while (!(USART2->ISR & USART_ISR_RXNE_RXFNE));

    // Read character from receive data register
    uint8_t ch = USART2->RDR;

    // Update global variables for Live Watch
    g_chars_received++;
    g_last_char_received = ch;

    return ch;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Check if a character is available to receive.
 * @return  1 if character available, 0 otherwise
 * ----------------------------------------------------------------------------
 */
uint8_t uart2_data_available(void)
{
    return (USART2->ISR & USART_ISR_RXNE_RXFNE) ? 1 : 0;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Transmit a null-terminated string via UART2.
 * @param   str Pointer to string to transmit
 * ----------------------------------------------------------------------------
 */
void uart2_transmit_string(const char* str)
{
    while (*str)
    {
        uart2_transmit_char(*str);
        str++;
    }
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Toggle LED1 for visual feedback.
 * ----------------------------------------------------------------------------
 */
void led1_toggle(void)
{
    GPIOA->ODR ^= (1 << LED1_PIN);
    g_led_blink_count++;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Simple delay function (blocking).
 * @param   count Delay count (approximately microseconds at 4MHz)
 * ----------------------------------------------------------------------------
 */
void delay_us(uint32_t count)
{
    for (volatile uint32_t i = 0; i < count; i++)
    {
        __asm("nop");
    }
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Main function for UART Demo on STM32L4S5VI.
 * @details Initializes system clock, GPIO, and UART2, then enters an echo loop.
 *          Receives characters via UART and echoes them back, with LED feedback.
 *
 * @return  This function never returns (infinite loop).
 *
 * @pre     Connect USB-to-Serial adapter to PA2(TX)/PA3(RX) and open terminal at 115200 baud.
 * @post    Characters typed in terminal are echoed back, LED1 blinks on each character.
 * ----------------------------------------------------------------------------
 */
int main(void)
{
    // Initialize system
    system_clock_init();
    gpio_init();
    uart2_init();

    // Send startup message
    uart2_transmit_string("STM32L4S5VI UART Demo\r\n");
    uart2_transmit_string("Type characters to see them echoed back\r\n");
    uart2_transmit_string("LED1 will blink for each received character\r\n");
    uart2_transmit_string("Ready> ");

    // Main loop - UART echo with LED feedback
    while (1)
    {
        // Check if data is available
        if (uart2_data_available())
        {
            // Receive character
            uint8_t received_char = uart2_receive_char();

            // Echo the character back
            uart2_transmit_char(received_char);

            // Toggle LED for visual feedback
            led1_toggle();

            // Update UART status for Live Watch
            g_uart_status = USART2->ISR;

            // If Enter key (CR) was pressed, send line feed and prompt
            if (received_char == '\r')
            {
                uart2_transmit_string("\nReady> ");
            }
        }

        // Small delay to prevent busy waiting
        delay_us(100);
    }

    return 0;  // Never reached
}
