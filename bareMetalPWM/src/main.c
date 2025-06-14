/*
 * ----------------------------------------------------------------------------
 * Project:   Bare-Metal PWM LED Example for STM32L4S5VI (B-L4S5I-IOT01A)
 * Author:    Dimitris Savvaris
 * License:   MIT License (see below)
 * ----------------------------------------------------------------------------
 * Description:
 *   This program demonstrates how to generate a PWM signal on pin PA0 (ARD-D1) using TIM2,
 *   with the ability to change either the pulse width or the frequency by pressing
 *   the user button (PC13). The code uses direct register access, without any HAL
 *   or CMSIS drivers, to configure the system clock, GPIO, and timer peripherals.
 *
 *   Features:
 *     - PA0 (ARD-D1) outputs a PWM signal (TIM2_CH1, AF1).
 *     - PC13 is configured as an input with pull-up for the user button.
 *     - On each button press, the PWM frequency or pulse width cycles through preset values.
 *     - The timer is clocked at 10 kHz (after prescaler), and the PWM period and duty cycle
 *       are set by adjusting ARR and CCR1 registers.
 *
 *   This example is suitable for learning bare-metal STM32 programming and basic PWM control.
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

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize user button (PC13) as input with pull-up.
 * @details Enables GPIOC clock, sets PC13 as input, and enables pull-up.
 * ----------------------------------------------------------------------------
 */
void init_button_pc13(void) {
    /* Enable GPIOC clock */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    /* Set PC13 as input */
    GPIOC->MODER &= ~(0x3 << (13 * 2));
    /* Enable pull-up on PC13 */
    GPIOC->PUPDR &= ~(0x3 << (13 * 2));
    GPIOC->PUPDR |=  (0x1 << (13 * 2));
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Detect falling edge on user button (PC13).
 * @retval  1 if button was just pressed, 0 otherwise.
 * ----------------------------------------------------------------------------
 */
int is_button_pressed(void) {
    static int prev_state = 1;
    int current_state = (GPIOC->IDR & (1 << 13)) ? 1 : 0;

    if (prev_state == 1 && current_state == 0) {
        prev_state = 0;
        return 1; // Falling edge detected
    } else if (current_state == 1) {
        prev_state = 1;
    }
    return 0;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Configure system clock to 80 MHz using HSI and PLL.
 * @details Sets up the PLL and voltage scaling for high performance.
 * ----------------------------------------------------------------------------
 */
void SystemClock_Config(void) {
    /* Enable HSI (16 MHz internal) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* Configure power regulator voltage scaling for high frequency (scale 1) */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
    PWR->CR1 &= ~PWR_CR1_VOS;
    PWR->CR1 |= PWR_CR1_VOS_0; // VOS = 1 (high performance)

    /* Configure PLL: source = HSI, PLLM = 1, PLLN = 20, PLLR = 2 */
    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;
    RCC->PLLCFGR |= (1 << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (20 << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLR_Pos);
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* Set Flash latency for 80 MHz */
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS;

    /* Select PLL as system clock */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize PA0 as TIM2_CH1 PWM output (AF1).
 * @details Enables GPIOA and TIM2 clocks, sets PA0 to AF1 for TIM2_CH1.
 * ----------------------------------------------------------------------------
 */
void init_gpio_pa0_pwm(void) {
    /* Enable GPIOA and TIM2 peripheral clocks */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    /* Set PA0 to Alternate Function mode (AF1 for TIM2_CH1) */
    GPIOA->MODER &= ~(0b11 << (0 * 2));
    GPIOA->MODER |=  (0b10 << (0 * 2));
    GPIOA->AFR[0] &= ~(0xF << (0 * 4));
    GPIOA->AFR[0] |=  (0x1 << (0 * 4));
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Initialize TIM2 for PWM: 20 Hz, 1 ms pulse (2% duty).
 * @details Sets prescaler, ARR, CCR1, and configures PWM mode.
 * ----------------------------------------------------------------------------
 */
void init_tim2_pwm_20hz_1ms_pulse(void) {
    /* Set prescaler to divide 80 MHz to 10 kHz (1 tick = 0.1 ms) */
    TIM2->PSC = 7999;

    /* Set ARR for 50 ms period: 10 kHz / 20 Hz = 500 ticks */
    TIM2->ARR = 499;

    /* Set pulse width = 1 ms = 10 ticks (2% duty cycle) */
    TIM2->CCR1 = 10;

    /* Set PWM Mode 1 on CH1: active until match */
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    /* Enable output for channel 1 */
    TIM2->CCER |= TIM_CCER_CC1E;

    /* Enable auto-reload preload */
    TIM2->CR1 |= TIM_CR1_ARPE;

    /* Enable counter */
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Update PWM pulse width (CCR1).
 * @param   ticks Pulse width in timer ticks.
 * ----------------------------------------------------------------------------
 */
void update_pulse_width(uint16_t ticks) {
    TIM2->CCR1 = ticks;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Update PWM frequency and keep duty at 10%.
 * @param   freq_hz Desired frequency in Hz.
 * ----------------------------------------------------------------------------
 */
void update_pwm_frequency(uint32_t freq_hz) {
    uint32_t timer_clk = 10000; // After PSC = 7999
    uint32_t arr = (timer_clk / freq_hz) - 1;

    TIM2->ARR = arr;
    TIM2->CCR1 = (arr + 1) / 10; // 10% duty

    /* Force update to load new ARR/CCR1 */
    TIM2->EGR |= TIM_EGR_UG;
}

/**
 * ----------------------------------------------------------------------------
 * @brief   Main program loop.
 * @details Initializes peripherals and cycles PWM settings on button press.
 * ----------------------------------------------------------------------------
 */
int main(void) {
    SystemClock_Config();
    SystemCoreClockUpdate();

    init_gpio_pa0_pwm();
    init_tim2_pwm_20hz_1ms_pulse();
    init_button_pc13();

#if PULSE_WIDTH
    /* Pulse widths in ticks (1 tick = 0.1 ms) */
    uint16_t pulse_widths[] = {10, 50, 100, 250, 500, 1000, 2000, 5000, 10000, 10};
    int pulse_index = 0;
#endif    

    /* Frequency steps in Hz */
    uint32_t frequencies[] = {1, 2, 5, 10, 20, 50, 100, 1};
    uint8_t freq_index = 0;

    while (1) {
        if (is_button_pressed()) {
#if PULSE_WIDTH
            pulse_index = (pulse_index + 1) % 10;
            update_pulse_width(pulse_widths[pulse_index]);
#else    
            freq_index = (freq_index + 1) % 8;
            update_pwm_frequency(frequencies[freq_index]);
#endif
        }
    }
}