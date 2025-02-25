#include "stm32g4xx.h"  // Device header for STM32G4 series
#include <stdio.h>

#define SYSCLK     16000000U  // Assumed system clock frequency (Hz)
#define BAUDRATE   115200U

// Simple delay loop
void delay(volatile uint32_t count) {
    while(count--) { __NOP(); }
}

/* USART2 functions for serial output (PA2 = TX) */
void uart_init(void) {
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    USART2->CR1 &= ~USART_CR1_UE;  // Disable USART2
    USART2->BRR = SYSCLK / BAUDRATE; // Set baud rate
    USART2->CR1 |= USART_CR1_TE;      // Enable transmitter
    USART2->CR1 |= USART_CR1_UE;      // Enable USART2
    while(!(USART2->ISR & USART_ISR_TEACK));
}

void uart_send_char(char c) {
    while(!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = c;
}

void uart_send_string(const char *str) {
    while(*str) {
        uart_send_char(*str++);
    }
}

/*
   timer_pwm_input_init_custom() configures a given timer in PWM input (reset) mode.
   It assumes the PWM signal is connected to channel 1 (TI1) for both rising and falling edges.
*/
void timer_pwm_input_init_custom(TIM_TypeDef* TIMx) {
    // Enable timer clock based on instance.
    if (TIMx == TIM1) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    } else if (TIMx == TIM2) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    } else if (TIMx == TIM3) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    } else if (TIMx == TIM4) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
    } else if (TIMx == TIM5) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
    } else if (TIMx == TIM15) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    } else if (TIMx == TIM8) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM8EN; // Enable TIM8 clock
    }

    // Reset timer registers
    TIMx->CR1 = 0;
    TIMx->CR2 = 0;
    TIMx->SMCR = 0;
    TIMx->DIER = 0;

    // Configure input capture channels:
    // Channel 1: input on TI1 (rising edge) – set CC1S = 01.
    // Channel 2: input on TI1 (falling edge) – set CC2S = 10.
    TIMx->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIMx->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1);

    // Enable capture on both channels and set polarity.
    // Channel 1: rising edge.
    TIMx->CCER &= ~TIM_CCER_CC1P;
    TIMx->CCER |= TIM_CCER_CC1E;
    // Channel 2: falling edge.
    TIMx->CCER |= TIM_CCER_CC2P;
    TIMx->CCER |= TIM_CCER_CC2E;

    // Configure slave mode in Reset Mode (counter resets on every TI1 rising edge).
    TIMx->SMCR &= ~TIM_SMCR_SMS;
    TIMx->SMCR |= TIM_SMCR_SMS_2;  // SMS = 100: Reset mode
    TIMx->SMCR &= ~TIM_SMCR_TS;
    TIMx->SMCR |= (0x5 << TIM_SMCR_TS_Pos); // Trigger selection: TI1FP1

    TIMx->PSC = 0;      // No prescaler
    TIMx->ARR = 0xFFFF; // Maximum auto-reload value
    TIMx->CR1 |= TIM_CR1_CEN;  // Start timer
}

/*
   gpio_init_encoder() configures a given GPIO pin for alternate function mode.
   The 'af' parameter must match the alternate function mapping for the timer channel.
*/
void gpio_init_encoder(GPIO_TypeDef* GPIOx, uint32_t pin, uint32_t af) {
    // Enable clock for GPIO port (for GPIOA, GPIOB, or GPIOC).
    if (GPIOx == GPIOA) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    } else if (GPIOx == GPIOB) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    } else if (GPIOx == GPIOC) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    }

    // Set pin to Alternate Function mode.
    GPIOx->MODER &= ~(0x3 << (pin * 2));
    GPIOx->MODER |= (0x2 << (pin * 2));

    // Set alternate function.
    uint32_t afr_index = pin / 8;
    uint32_t afr_shift = (pin % 8) * 4;
    GPIOx->AFR[afr_index] &= ~(0xF << afr_shift);
    GPIOx->AFR[afr_index] |= (af << afr_shift);
}

/* Structure to hold encoder measurement data */
typedef struct {
    TIM_TypeDef* timer;
    uint32_t period;
    uint32_t high_time;
    float angle;
} EncoderData;

int main(void) {
    // --- Initialize USART2 for serial output (PA2 as TX, AF7) ---
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Configure PA2 for USART2 TX.
    GPIOA->MODER &= ~(0x3 << (2 * 2));
    GPIOA->MODER |=  (0x2 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4));
    GPIOA->AFR[0] |=  (7 << (2 * 4));
    uart_init();
    uart_send_string("USART2 is Initialized\r\n");

    // --- Initialize GPIO pins and timers for 6 encoders ---
    // Encoder1: TIM1 on PC0 (AF2)
    gpio_init_encoder(GPIOC, 0, 2);  // PC0, AF2 for TIM1_CH1
    timer_pwm_input_init_custom(TIM1);

    // Encoder2: TIM2 on PA0 (AF1)
    gpio_init_encoder(GPIOA, 0, 1);  // PA0, AF1 for TIM2_CH1
    timer_pwm_input_init_custom(TIM2);

    // Encoder3: TIM3 on PA6 (AF2)
    gpio_init_encoder(GPIOA, 6, 2);  // PA6, AF2 for TIM3_CH1
    timer_pwm_input_init_custom(TIM3);

    // Encoder4: TIM4 on PB6 (AF2)
    gpio_init_encoder(GPIOB, 6, 2);  // PB6, AF2 for TIM4_CH1
    timer_pwm_input_init_custom(TIM4);

    // Encoder5: TIM5 on PB2 (AF2)
    gpio_init_encoder(GPIOB, 2, 2);  // PB2, AF2 for TIM5_CH1
    timer_pwm_input_init_custom(TIM5);

    // Encoder6: TIM8 on PC6 (AF4)
    gpio_init_encoder(GPIOC, 6, 4);  // PC6, AF4 for TIM8_CH1
    timer_pwm_input_init_custom(TIM8);

    uart_send_string("Starting encoder capture...\r\n");

    // Create an array to hold measurements for each encoder.
    EncoderData encoders[6] = {
        { TIM1, 0, 0, 0.0f },
        { TIM2, 0, 0, 0.0f },
        { TIM3, 0, 0, 0.0f },
        { TIM4, 0, 0, 0.0f },
        { TIM5, 0, 0, 0.0f },
        { TIM8, 0, 0, 0.0f }
    };

    char buffer[128];
    uint8_t i;

    while (1) {
        // For each encoder, if a capture event occurred (CC1 flag), update measurements.
        for (i = 0; i < 6; i++) {
            if (encoders[i].timer->SR & TIM_SR_CC1IF) {
                encoders[i].timer->SR &= ~TIM_SR_CC1IF;
                encoders[i].period = encoders[i].timer->CCR1;  // Time between rising edges
                encoders[i].high_time = encoders[i].timer->CCR2; // Pulse width
                if (encoders[i].period != 0) {
                    float duty = (float)encoders[i].high_time / (float)encoders[i].period;
                    encoders[i].angle = duty * 360.0f;
                } else {
                    encoders[i].angle = 0.0f;
                }
            }
        }

        // Format and send all encoder angles via USART2.
        sprintf(buffer, "E1: %3d, E2: %3d, E3: %3d, E4: %3d, E5: %3d, E6: %3d\r\n",
                (int)encoders[0].angle, (int)encoders[1].angle, (int)encoders[2].angle,
                (int)encoders[3].angle, (int)encoders[4].angle, (int)encoders[5].angle);
        uart_send_string(buffer);

        delay(100000);
    }

    return 0;
}
