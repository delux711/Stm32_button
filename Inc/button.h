#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Initialization
void BUTTON_Init(void);
void BUTTON_delay_ms(uint32_t ms);

// IRQ handlers implemented in button.c
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_H
