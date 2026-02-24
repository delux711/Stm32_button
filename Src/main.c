#include <stm32f10x.h>
#include "system_stm32f10x.h"
#include "button.h"

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    BUTTON_Init();
    SysTick_Config(SystemCoreClock / 1000);

    for(;;) {
        __WFI();         // EXTI zobudí CPU
        GPIOC->BSRR = (GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    }
    return 0;
}
