#include <stdint.h>
#include <stm32f10x.h>


#define DEBOUNCE_MS        20
#define LONG_PRESS_MS     800
#define MULTICLICK_MS     400   // max pauza medzi klikmi

#define BUTTON_DEF(port, pin, exti, single_cb, double_cb, triple_cb, long_cb) \
    { port, (1 << pin), exti, 0, 0, 0, 0, 0, 0, single_cb, double_cb, triple_cb, long_cb }

typedef void (*button_cb_t)(void);
typedef enum {
    BTN_IDLE = 0u,
    BTN_DEBOUNCE_PRESS,
    BTN_PRESSED,
    BTN_DEBOUNCE_RELEASE
} btn_state_t;

typedef struct {
    /* HW */
    GPIO_TypeDef *port;
    uint16_t    pin_mask;
    uint8_t     exti_line;   // 0..15

    /* SW */
    btn_state_t state;
    uint32_t    timer;
    uint32_t    press_time;
    uint8_t     click_count;
    uint8_t     long_sent;
    uint8_t     active;      // 0 = EXTI mode, 1 = polling

    /* callbacks */
    button_cb_t on_single;
    button_cb_t on_double;
    button_cb_t on_triple;
    button_cb_t on_long;
} button_t;

volatile uint32_t sys_ms = 0;
static void delay_ms(uint32_t ms);
static void EXTI_globalHandler(void);


static void btn0_single(void);
static void btn0_double(void);
static void btn0_triple(void);
static void btn0_long(void);

static volatile button_t buttons[] = {
    // Tlačidlo 0 – PB12
    BUTTON_DEF(GPIOB, 12, 12, btn0_single, btn0_double, btn0_triple, btn0_long),
    BUTTON_DEF(GPIOC, 15, 15, btn0_long  , btn0_triple, btn0_double, btn0_single),
    BUTTON_DEF(GPIOA,  1,  1,          0U, btn0_double, btn0_triple, 0U),
    // Tlačidlo 1 – PB9
    // BUTTON_DEF(GPIOB,  9,  9, btn1_single, NULL, NULL, btn1_long)
};
#define BUTTON_COUNT (sizeof(buttons) / sizeof(buttons[0]))

static volatile uint8_t papuValue = 0u;
static void btn0_single(void) {
    papuValue = 1;
    GPIOC->BSRR = (GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    GPIOB->BSRR = (GPIOB->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    // led_pc13_blink(papuValue);
}
static void btn0_double(void) {
    papuValue = 2;
    GPIOC->BSRR = (GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    GPIOB->BSRR = (GPIOB->ODR & GPIO_ODR_ODR14) ? GPIO_BSRR_BR14 : GPIO_BSRR_BS14;
    // led_pc13_blink(papuValue);
}
static void btn0_triple(void) {
    papuValue = 3;
    GPIOC->BSRR = (GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    GPIOB->BSRR = (GPIOB->ODR & GPIO_ODR_ODR15) ? GPIO_BSRR_BR15 : GPIO_BSRR_BS15;
    // led_pc13_blink(papuValue);
}
static void btn0_long(void) {
    papuValue = 4;
    GPIOC->BSRR = (GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
    GPIOA->BSRR = (GPIOA->ODR & GPIO_ODR_ODR8) ? GPIO_BSRR_BR8 : GPIO_BSRR_BS8;
    // led_pc13_blink(papuValue);
}


static inline uint8_t button_raw(const volatile button_t *b)
{
    return (b->port->IDR & b->pin_mask) ? 1 : 0;
}

static void button_process(volatile button_t *b)
{
    uint8_t raw = button_raw(b);

    switch (b->state) {

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
            if (b->on_long) b->on_long();
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

        if (b->timer > 0 && --b->timer == 0) {

            switch (b->click_count) {
            case 1: if (b->on_single) b->on_single(); break;
            case 2: if (b->on_double) b->on_double(); break;
            case 3: if (b->on_triple) b->on_triple(); break;
            }

            b->click_count = 0;

            /* návrat do EXTI režimu */
            b->active = 0;
            EXTI->IMR |= (1 << b->exti_line);
        }
    }
}


void led_pc13_init(void) {
    uint32_t reg;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    reg = GPIOC->CRH;
    reg &= ~((uint32_t)GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    reg |= GPIO_CRH_MODE13; // 11: Output mode, max speed 50 MHz
    GPIOC->CRH = reg;

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    reg = GPIOB->CRH;
    reg &= ~((uint32_t)GPIO_CRH_CNF13 | GPIO_CRH_MODE13 | GPIO_CRH_CNF14 | GPIO_CRH_MODE14 | GPIO_CRH_CNF15 | GPIO_CRH_MODE15);
    reg |= GPIO_CRH_MODE13 | GPIO_CRH_MODE14 | GPIO_CRH_MODE15; // 11: Output mode, max speed 50 MHz
    GPIOB->CRH = reg;

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    reg = GPIOA->CRH;
    reg &= ~((uint32_t)GPIO_CRH_CNF8 | GPIO_CRH_MODE8);
    reg |= GPIO_CRH_MODE8; // 11: Output mode, max speed 50 MHz
    GPIOA->CRH = reg;
}

static void led_pc13_toggle(void) {
    // bit banding pre PC13 ODR registra je adresa 0x422201B4 = 0x42000000 + (0x1100c*32)+(0xD*4)
    // adresa bitu BS13 pre PORTC je: 0x42220234 = 0x42000000 + (0x11010*32)+(13*4)
    // adresa bitu BR13 pre PORTC je: 0x42220274 = 0x42000000 + (0x11010*32)+(29*4)
    GPIOC->BSRR = (GPIOC->ODR & GPIO_ODR_ODR13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
}

void led_pc13_blink(uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        GPIOC->BRR  = (1 << 13); // LED ON
        delay_ms(200);
        GPIOC->BSRR = (1 << 13); // LED OFF
        delay_ms(200);
    }
}

static void delay_ms(uint32_t ms) {
    uint32_t start = sys_ms;
    while ((sys_ms - start) < ms) {
        // __WFI(); // šetrí CPU
    }
}

static void buttons_exti_init(void)
{
    // Povoliť AFIO
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    for (uint32_t i = 0; i < BUTTON_COUNT; i++) {
        button_t *b = (button_t *)&buttons[i];

        // Mapovanie EXTI
        uint32_t line = b->exti_line;
        uint32_t port_source = 0;

        if (b->port == GPIOA) port_source = 0;
        else if (b->port == GPIOB) port_source = 1;
        else if (b->port == GPIOC) port_source = 2;
        else if (b->port == GPIOD) port_source = 3;

        if (line <= 3)      AFIO->EXTICR[0] |= port_source << (line * 4);
        else if (line <= 7) AFIO->EXTICR[1] |= port_source << ((line - 4) * 4);
        else if (line <= 11) AFIO->EXTICR[2] |= port_source << ((line - 8) * 4);
        else if (line <= 15) AFIO->EXTICR[3] |= port_source << ((line - 12) * 4);

        // Nastavenie EXTI
        EXTI->IMR  |= (1 << line);  // unmask
        EXTI->EMR  &= ~(1 << line); // len interrupt
        EXTI->RTSR |= (1 << line);  // zmena na nábežnú hranu
        EXTI->FTSR |= (1 << line);  // aj zostupnú hranu (záleží od logiky tlačidla)
        EXTI->PR   = (1 << line);   // clear pending

        // Povolenie NVIC pre EXTI 10-15 (ak máš viac tlačidiel)
        if(b->exti_line <= 4u) {
            NVIC_EnableIRQ(EXTI0_IRQn + b->exti_line);
        } else if(b->exti_line <= 9u) {
            NVIC_EnableIRQ(EXTI9_5_IRQn);
        } else {
            NVIC_EnableIRQ(EXTI15_10_IRQn);
        }
    }
}

void buttons_hw_init(void)
{
    for (uint32_t i = 0; i < BUTTON_COUNT; i++) {
        button_t *b = (button_t *)&buttons[i];

        // povoliť hodiny pre port
        if (b->port == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
        else if (b->port == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
        else if (b->port == GPIOC) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
        else if (b->port == GPIOD) RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;

        // GPIO pin: vstup pull-up / pull-down
        if (b->pin_mask < (1 << 8)) {
            b->port->CRL &= ~(0xF << (__builtin_ctz(b->pin_mask) * 4)); // vymaž
            b->port->CRL |=  (0x8 << (__builtin_ctz(b->pin_mask) * 4)); // CNF=10, MODE=00
        } else {
            uint32_t pin = __builtin_ctz(b->pin_mask) - 8;
            b->port->CRH &= ~(0xF << (pin * 4));
            b->port->CRH |=  (0x8 << (pin * 4));
        }

        // pull-up
        b->port->BSRR = b->pin_mask;   // nastav high
    }
    buttons_exti_init();
}

void EXTI0_IRQHandler(void) {
    EXTI_globalHandler();
}
void EXTI1_IRQHandler(void) {
    EXTI_globalHandler();
}
void EXTI2_IRQHandler(void) {
    EXTI_globalHandler();
}
void EXTI3_IRQHandler(void) {
    EXTI_globalHandler();
}
void EXTI4_IRQHandler(void) {
    EXTI_globalHandler();
}
void EXTI9_5_IRQHandler(void) {
    EXTI_globalHandler();
}
void EXTI15_10_IRQHandler(void)
{
    EXTI_globalHandler();
}

static void EXTI_globalHandler(void) {
    uint32_t pending = EXTI->PR & EXTI->IMR;

    for (uint32_t i = 0; i < BUTTON_COUNT; i++) {
        uint32_t mask = (1 << buttons[i].exti_line);
        if (pending & mask) {
            EXTI->PR = mask;                 // clear pending
            EXTI->IMR &= ~mask;              // vypnúť EXTI

            buttons[i].active = 1;           // prejsť do polling
        }
    }
}

void SysTick_Handler(void)
{
    (void)papuValue;
    sys_ms++;
    for (uint32_t i = 0; i < BUTTON_COUNT; i++) {
        if (buttons[i].active) {
            button_process(&buttons[i]);
            button_multiclick_process(&buttons[i]);
        }
    }
}
