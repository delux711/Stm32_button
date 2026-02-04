#include <stdint.h>
#include <stm32f10x.h>


#define DEBOUNCE_MS        20
#define LONG_PRESS_MS     800
#define MULTICLICK_MS     400   // max pauza medzi klikmi

typedef enum {
    BTN_IDLE = 0u,
    BTN_DEBOUNCE_PRESS,
    BTN_PRESSED,
    BTN_DEBOUNCE_RELEASE
} btn_state_t;

typedef struct {
    btn_state_t state;
    uint32_t timer;
    uint32_t press_time;
    uint8_t click_count;
    uint8_t long_sent;
} button_t;

static volatile button_t btn = { 0 };
volatile uint32_t sys_ms = 0;

static void on_long_press(void);
static void on_single_click(void);
static void on_double_click(void);
static void on_triple_click(void);
static void delay_ms(uint32_t ms);

static void button_process(volatile button_t *b, uint8_t raw)
{
    switch (b->state)
    {
        case BTN_IDLE:
            if (raw == 0) {
                b->state = BTN_DEBOUNCE_PRESS;
                b->timer = DEBOUNCE_MS;
            }
            break;

        case BTN_DEBOUNCE_PRESS:
            if (raw == 0) {
                if (--b->timer == 0) {
                    b->state = BTN_PRESSED;
                    b->press_time = 0;
                    b->long_sent = 0;
                }
            } else {
                b->state = BTN_IDLE;
            }
            break;

        case BTN_PRESSED:
            b->press_time++;

            if (!b->long_sent && b->press_time >= LONG_PRESS_MS) {
                b->long_sent = 1;
                on_long_press();
            }

            if (raw == 1) {
                b->state = BTN_DEBOUNCE_RELEASE;
                b->timer = DEBOUNCE_MS;
            }
            break;

        case BTN_DEBOUNCE_RELEASE:
            if (raw == 1) {
                if (--b->timer == 0) {
                    b->state = BTN_IDLE;

                    if (!b->long_sent) {
                        b->click_count++;
                        b->timer = MULTICLICK_MS;
                    }
                }
            } else {
                b->state = BTN_PRESSED;
            }
            break;
    }
}

static void button_multiclick_process(volatile button_t *b)
{
    if (b->click_count > 0 && b->state == BTN_IDLE) {
        if (--b->timer == 0) {
            switch (b->click_count) {
            case 1: on_single_click(); break;
            case 2: on_double_click(); break;
            case 3: on_triple_click(); break;
            default: break;
            }
            b->click_count = 0;
        }
    }
}

static void led_pc13_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    GPIOC->CRH &= ~(0xF << 20);
    GPIOC->CRH |=  (0x2 << 20);   // output, 2 MHz, push-pull

    GPIOC->BSRR = (1 << 13);     // LED OFF (active-low)
}

void led_pc13_blink(uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        GPIOC->BRR  = (1 << 13); // LED ON
        delay_ms(200);
        GPIOC->BSRR = (1 << 13); // LED OFF
        delay_ms(200);
    }
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = sys_ms;
    while ((sys_ms - start) < ms) {
        __WFI(); // šetrí CPU
    }
}

static volatile uint8_t papuValue = 0u;
static void on_long_press(void) {
    papuValue = 4;
    led_pc13_blink(papuValue);
}
static void on_single_click(void) {
    papuValue = 1;
    led_pc13_blink(papuValue);
}
static void on_double_click(void) {
    led_pc13_blink(papuValue);
}
static void on_triple_click(void) {
    led_pc13_blink(papuValue);
}

void SysTick_Handler(void)
{
    (void)papuValue;
    sys_ms++;
    uint8_t raw = (GPIOB->IDR & (1 << 12)) ? 1 : 0;
    button_process(&btn, raw);
    button_multiclick_process(&btn);
}

void buttonInit(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    // Vymaž konfiguráciu PB12 (bity 19:16)
    GPIOB->CRH &= ~(0xF << 16);
    // CNF12 = 10, MODE12 = 00
    GPIOB->CRH |=  (0x8 << 16);
    // Pull-up (ODR bit = 1)
    GPIOB->ODR |= (1 << 12);
    led_pc13_init();
    SysTick_Config(SystemCoreClock / 1000); // 1 ms
}
