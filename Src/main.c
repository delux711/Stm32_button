#include <stm32f10x.h>
#include "system_stm32f10x.h"

extern void buttons_hw_init(void);
extern void led_pc13_init(void);

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    led_pc13_init();
    buttons_hw_init();
    SysTick_Config(SystemCoreClock / 1000);

    for(;;) {
        __WFI();         // EXTI zobudí CPU
        GPIOC->BSRR = (GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    }
    return 0;
}
