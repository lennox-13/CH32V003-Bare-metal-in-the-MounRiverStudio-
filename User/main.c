#include "ch32v00x.h"

/*
 * PA1 Button 1 (IPU)
 * PA2 Button 2 (IPU)
 * PC1 LED 1    (C_C)
 * PC2 LED 2    (C_C)
 * PC4 LED PWM  (C_C)
 * GND
 * VDD 3.3V
 */

#define PWM_STEPS 220 // Number of steps for the LED pulse effect

uint16_t pwm_values[PWM_STEPS];

void Generate_PWM_Values() {
    uint16_t step_size = 1023 / (PWM_STEPS / 2);
    for (uint16_t i = 0; i < PWM_STEPS / 2; i++) {
        pwm_values[i] = i * step_size;                 // DEC LED PWM brightness
        pwm_values[PWM_STEPS - i - 1] = pwm_values[i]; // INC LED PWM brightness
    }
}

void GPIO_init() {
    // GPIOA Setup
    RCC->APB2PCENR |= (1 << 2) | (1 << 0);      // Enable clocks for GPIOA and AFIO
    AFIO->PCFR1   &= ~(1 << 15);                // PA12_RM bit must be cleared when PA1 and PA2 are used as normal GPIO pins

    GPIOA->CFGLR &= ~(0xFF << 4);               // Reset configuration for pins PA1 and PA2
    GPIOA->CFGLR |=  (0x88 << 4);               // Set mode to Input-PullUp
    GPIOA->OUTDR |=  (3 << 1);                  // Activate pull-up resistors for PA1 and PA2

    // GPIOC Setup
    RCC->APB2PCENR |=(1 << 4);                  // Enable clock for GPIOC
    GPIOC->CFGLR &= ~(0xF << 16);               // Clear previous settings
    GPIOC->CFGLR |=  (0xB << 16);               // Alternate function PC4 push-pull 10MHz

    GPIOC->CFGLR &= ~( (0xF << 8)| (0xF << 4)); // Reset configuration for pins PC1, PC2
    GPIOC->CFGLR |=    (0x1 << 8)| (0x1 << 4);  // PC1, PC2 GPIO_Speed_10MHz; OUTPUT
}

void TIM1_PWMOut() {
    RCC->APB2PCENR |= (1 << 11);                // Enable clock for TIM1
    TIM1->PSC   = 480 - 1;                      // 48 MHz
    TIM1->ATRLR = 1000 - 1;                     // 1000 Hz / 10 = 100 Hz

    TIM1->CHCTLR2 &= ~(7 << 12);                // Clear OC4M bits
    TIM1->CHCTLR2 |= (7 << 12);                 // PWM mode 2
    TIM1->CHCTLR2 |= (1 << 11);                 // Enable preload mode
    TIM1->CCER    |= (1 << 12);                 // CC4E = 1 (Enable TIM1 CH4)
    TIM1->BDTR    |= (1 << 15);                 // Enable main output (MOE)
    TIM1->CTLR1   |= (1 << 7);                  // ARPE (Auto-reload preload enable)
    TIM1->CTLR1   |= (1 << 0);                  // Start the TIM1 timer

    TIM1->DMACFGR = ((PWM_STEPS > 32 ? 31 :
                     PWM_STEPS - 1) << 8) | (0x3C); // DMA configuration for CH4CVR (DBL max = 31)
    TIM1->DMAINTENR |= (1 << 12);                   // Enable DMA for CC4
}

void DMA_Config(void) {
    RCC->AHBPCENR |= (1 << 0);                      // Enable clock for DMA1

    DMA1_Channel4->PADDR = (uint32_t)&TIM1->CH4CVR; // Peripheral address (TIM1 CH4CVR register) for DMA
    DMA1_Channel4->MADDR = (uint32_t)pwm_values;    // Memory address where PWM values are stored
    DMA1_Channel4->CNTR  = PWM_STEPS;               // SNumber of data items to transfer (PWM_STEPS)

    DMA1_Channel4->CFGR = (1 << 7)  |               // Memory increment enable (MINC)
                          (1 << 5)  |               // Circular mode enable (CIRC)
                          (1 << 8)  |               // Peripheral data size = 16-bit (PSIZE[0:1])
                          (1 << 10) |               // Memory data size = 16-bit (MSIZE[0:1])
                          (1 << 4)  |               // Data transfer direction: Memory to Peripheral DIR = 1
                          (0 << 12);                // Priority level = Low (PL[0:0])

    DMA1_Channel4->CFGR |= (1 << 0);                // Enable DMA
}

int main(void) {

    GPIO_init();
    TIM1_PWMOut();
    DMA_Config();
    Generate_PWM_Values();

    while (1) {
    (GPIOA->INDR & (1 << 1)) ? (GPIOC->BSHR |= (1 << 17)) : (GPIOC->BSHR |= (1 << 1)); // PA1 = On LED C1
    (GPIOA->INDR & (1 << 2)) ? (GPIOC->BSHR |= (1 << 18)) : (GPIOC->BSHR |= (1 << 2)); // PA2 = On LED C2
        }
}
