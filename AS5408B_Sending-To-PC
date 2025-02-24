/*
Author : Tarush Sonawala
Date : 24/02/25
Desc : This is a 14-bit version of the code that takes data from as5048b encoder and sends it to pc using serial port.
*/



#include "stm32g4xx.h"  // Device header for the STM32G4 series
#include <stdio.h>

#define SYSCLK     16000000U  // Assumed system clock frequency (Hz)
#define BAUDRATE   115200U

// Simple delay loop (adjust if needed)
void delay(volatile uint32_t count) {
    while(count--) { __NOP(); }
}

// USART2 initialization (PA2 = TX)
// Configures USART2 for 8 data bits, no parity, 1 stop bit.
void uart_init(void) {
    // Enable clock for USART2 (on APB1ENR1)
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    // Disable USART before configuration.
    USART2->CR1 &= ~USART_CR1_UE;

    // Set baud rate (using SYSCLK directly)
    USART2->BRR = SYSCLK / BAUDRATE;

    // Enable transmitter (8N1 configuration by default)
    USART2->CR1 |= USART_CR1_TE;

    // Enable USART2.
    USART2->CR1 |= USART_CR1_UE;

    // Wait until USART is ready.
    while(!(USART2->ISR & USART_ISR_TEACK));
}

// Transmit one character via USART2
void uart_send_char(char c) {
    while(!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = c;
}

// Transmit a null-terminated string via USART2
void uart_send_string(const char *str) {
    while(*str) {
        uart_send_char(*str++);
    }
}

// Timer3 configuration in PWM input mode for reading the AS5408 sensor PWM signal.
// This configuration uses channel 1 (rising edge) and channel 2 (falling edge) both mapped to TI1.
void timer_pwm_input_init(void) {
    // Enable clock for TIM3 (on APB1ENR1)
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    // Reset timer configuration
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->SMCR = 0;
    TIM3->DIER = 0;

    // --- Configure input capture channels ---
    // Channel 1: input on TI1 (rising edge) – set CC1S = 01.
    // Channel 2: input on TI1 (falling edge) – set CC2S = 10.
    TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1);

    // --- Configure capture polarity and enable channels ---
    // Channel 1: rising edge capture.
    TIM3->CCER &= ~TIM_CCER_CC1P;  // rising edge
    TIM3->CCER |= TIM_CCER_CC1E;   // enable channel 1

    // Channel 2: falling edge capture.
    TIM3->CCER |= TIM_CCER_CC2P;   // falling edge
    TIM3->CCER |= TIM_CCER_CC2E;   // enable channel 2

    // --- Configure timer slave mode (reset mode) ---
    // Reset mode: counter resets on every rising edge (TI1).
    TIM3->SMCR &= ~TIM_SMCR_SMS;
    TIM3->SMCR |= TIM_SMCR_SMS_2;   // SMS = 100 (Reset Mode)

    // Select trigger source as TI1FP1.
    TIM3->SMCR &= ~TIM_SMCR_TS;
    TIM3->SMCR |= (0x5 << TIM_SMCR_TS_Pos); // 0x5 corresponds to TI1FP1

    // Set prescaler to 0 (timer runs at SYSCLK rate)
    TIM3->PSC = 0;

    // Set Auto-Reload Register to maximum so period can be captured.
    TIM3->ARR = 0xFFFF;

    // Enable TIM3 counter.
    TIM3->CR1 |= TIM_CR1_CEN;
}

int main(void) {
    // --- Enable clocks for GPIOA ---
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // --- Configure GPIO pins ---
    // PA2 for USART2 TX: Alternate function mode, AF7.
    GPIOA->MODER &= ~(0x3 << (2 * 2));   // Clear mode bits for PA2
    GPIOA->MODER |= (0x2 << (2 * 2));      // Set alternate function mode
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));    // Clear alternate function bits for PA2
    GPIOA->AFR[0] |= (7 << (2 * 4));       // AF7 for USART2

    // PA6 for TIM3_CH1: Alternate function mode.
    // IMPORTANT: Based on your encoder files, PA6 should use AF2 (not AF1) for TIM3.
    GPIOA->MODER &= ~(0x3 << (6 * 2));     // Clear mode bits for PA6
    GPIOA->MODER |= (0x2 << (6 * 2));        // Set alternate function mode
    GPIOA->AFR[0] &= ~(0xF << (6 * 4));      // Clear alternate function bits for PA6
    GPIOA->AFR[0] |= (2 << (6 * 4));         // AF2 for TIM3_CH1

    // --- Initialize peripherals ---
    uart_init();
    timer_pwm_input_init();

    // Debug startup message.
    uart_send_string("Starting encoder capture...\r\n");

    char buffer[64];
    uint32_t period, high_time, encoder_value;

    float duty_cycle, angle;

    while (1) {
        // Wait for a capture event with a timeout.
        uint32_t timeout = 1000000;
        while (!(TIM3->SR & TIM_SR_CC1IF) && timeout) {
            timeout--;
        }

        if (timeout == 0) {
            // No capture event detected—print a debug message.
            uart_send_string("No capture event.\r\n");
        } else {
            // A capture event occurred; clear the flag and read values.
            TIM3->SR &= ~TIM_SR_CC1IF;
            period   = TIM3->CCR1;  // Captured period (time between rising edges)
            high_time = TIM3->CCR2;  // Captured high time (pulse width)

            if (period != 0) {
                // Calculate raw 14-bit value (0 to 16383) based on the PWM duty cycle.
                encoder_value = (high_time * 16383UL) / period;
            } else {
                encoder_value = 0;
            }

            sprintf(buffer, "Encoder Value: %lu\r\n", encoder_value);
            uart_send_string(buffer);
        }

        // Delay before checking again.
        delay(100000);
    }

    return 0;
}
