#include <stm32f10x.h>
#include "system_stm32f10x.h"

extern void buttons_hw_init(void);
extern void led_pc13_init(void);
extern void buttons_exti_init(void);

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    led_pc13_init();
    buttons_hw_init();
    buttons_exti_init();
    SysTick_Config(SystemCoreClock / 1000);

    for(;;) {
        __WFI();         // EXTI zobudí CPU
    }
    return 0;
}
