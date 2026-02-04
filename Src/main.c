#include <stm32f10x.h>
#include "system_stm32f10x.h"

extern void buttonInit(void);

int main(void) {
    SystemInit();
    SystemCoreClockUpdate();

    buttonInit();

    for(;;) {
    }
    return 0;
}
