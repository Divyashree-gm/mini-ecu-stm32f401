/**
 * ============================================================================
 * MINI-ECU : Bare-metal Embedded Drive Control System
 * Target : STM32F401 (default pin map = NUCLEO-F401RE), HSI 16 MHz
 * Style : direct register access only - no HAL, no RTOS, no Arduino
 *
 * PIN MAP
 * LED Orange PB12 | Green PB13 | Blue PB14 | Red PB15
 * Touch START PA1 (active HIGH, pull-down)
 * E-STOP PC13 (EXTI13, falling edge, active LOW)
 * Pot PA0 (ADC1_IN0) | PWM PA6 (TIM3_CH1, AF2, 1 kHz)
 * Dir pins PB0, PB1 | UART USART2 PA2 TX / PA3 RX, 115200 8N1
 * ============================================================================
 */
#include <stdint.h>

#define REG(a) (*(volatile uint32_t *)(a))

/* ---------------------------- 1. USER SETTINGS --------------------------- */
#define LED_PORT GPIOB_BASE
#define LED_ORANGE 12
#define LED_GREEN 13
#define LED_BLUE 14
#define LED_RED 15

#define TOUCH_PORT GPIOA_BASE
#define TOUCH_PIN 1
#define TOUCH_ACTIVE 1 /* 1 = reads HIGH when touched */

#define ESTOP_PORT GPIOC_BASE
#define ESTOP_PIN 13
#define ESTOP_ACTIVE 0 /* 0 = reads LOW when pressed */

#define MOT_PORT GPIOB_BASE
#define MOT_IN1 0
#define MOT_IN2 1

#define PWM_ARR 999 /* 16 MHz /16 /1000 = 1 kHz */
#define ADC_DEAD_LO 40 /* below this -> 0 % */
#define ADC_DEAD_HI 40 /* within this of 4095 -> 100 % */
#define TOUCH_DEBOUNCE_MS 200
#define CMD_IDLE_TIMEOUT_MS 1500 /* accept command w/o Enter after this*/

/* ---------------------------- 2. REGISTER MAP ---------------------------- */
#define RCC_BASE 0x40023800UL
#define RCC_AHB1ENR REG(RCC_BASE + 0x30)
#define RCC_APB1ENR REG(RCC_BASE + 0x40)
#define RCC_APB2ENR REG(RCC_BASE + 0x44)

#define GPIOA_BASE 0x40020000UL
#define GPIOB_BASE 0x40020400UL
#define GPIOC_BASE 0x40020800UL
#define GPIO_MODER(p) REG((p) + 0x00)
#define GPIO_PUPDR(p) REG((p) + 0x0C)
#define GPIO_IDR(p) REG((p) + 0x10)
#define GPIO_BSRR(p) REG((p) + 0x18)

#define TIM3_BASE 0x40000400UL
#define TIM3_CR1 REG(TIM3_BASE + 0x00)
#define TIM3_EGR REG(TIM3_BASE + 0x14)
#define TIM3_CCMR1 REG(TIM3_BASE + 0x18)
#define TIM3_CCER REG(TIM3_BASE + 0x20)
#define TIM3_PSC REG(TIM3_BASE + 0x28)
#define TIM3_ARR REG(TIM3_BASE + 0x2C)
#define TIM3_CCR1 REG(TIM3_BASE + 0x34)

#define USART2_BASE 0x40004400UL
#define USART2_SR REG(USART2_BASE + 0x00)
#define USART2_DR REG(USART2_BASE + 0x04)
#define USART2_BRR REG(USART2_BASE + 0x08)
#define USART2_CR1 REG(USART2_BASE + 0x0C)

#define ADC1_BASE 0x40012000UL
#define ADC1_SR REG(ADC1_BASE + 0x00)
#define ADC1_CR2 REG(ADC1_BASE + 0x08)
#define ADC1_SMPR2 REG(ADC1_BASE + 0x10)
#define ADC1_SQR1 REG(ADC1_BASE + 0x2C)
#define ADC1_SQR3 REG(ADC1_BASE + 0x34)
#define ADC1_DR REG(ADC1_BASE + 0x4C)

#define SYSCFG_EXTICR4 REG(0x40013800UL + 0x14)
#define EXTI_IMR REG(0x40013C00UL + 0x00)
#define EXTI_FTSR REG(0x40013C00UL + 0x0C)
#define EXTI_PR REG(0x40013C00UL + 0x14)

#define NVIC_ISER1 REG(0xE000E104UL) /* IRQ 32..63 */
#define SYST_CSR REG(0xE000E010UL)
#define SYST_RVR REG(0xE000E014UL)
#define SYST_CVR REG(0xE000E018UL)

#define IRQ_OFF() __asm volatile ("cpsid i" ::: "memory")
#define IRQ_ON() __asm volatile ("cpsie i" ::: "memory")

/* ---------------------------- 3. GLOBAL STATE ---------------------------- */
typedef enum { ST_IDLE = 0, ST_READY, ST_RUNNING, ST_ESTOP } state_t;

static volatile state_t state = ST_IDLE;
static volatile uint8_t dir_rev = 0; /* 0 = forward, 1 = reverse */
static volatile uint32_t ms = 0; /* SysTick millisecond counter */
static uint32_t cur_adc = 0, cur_pwm = 0;

#define RXN 64
static volatile uint8_t rx_buf[RXN];
static volatile uint8_t rx_head = 0, rx_tail = 0;

/* ---------------------------- 4. LOW-LEVEL HELPERS ----------------------- */
static void pin_mode(uint32_t port, uint32_t pin, uint32_t mode) /*0in 1out 2af 3an*/
{ GPIO_MODER(port) = (GPIO_MODER(port) & ~(3u << (pin * 2))) | (mode << (pin * 2)); }

static void pin_pull(uint32_t port, uint32_t pin, uint32_t pull) /*0none 1up 2down*/
{ GPIO_PUPDR(port) = (GPIO_PUPDR(port) & ~(3u << (pin * 2))) | (pull << (pin * 2)); }

static void pin_af(uint32_t port, uint32_t pin, uint32_t af)
{
    uint32_t off = (pin < 8) ? 0x20 : 0x24, sh = (pin & 7) * 4;
    REG(port + off) = (REG(port + off) & ~(0xFu << sh)) | (af << sh);
}

static int estop_pressed(void)
{ return (((GPIO_IDR(ESTOP_PORT) >> ESTOP_PIN) & 1u) == ESTOP_ACTIVE); }

/* ---------------------------- 5. UART OUTPUT ----------------------------- */
static void uputc(char c) { while (!(USART2_SR & (1u << 7))); USART2_DR = (uint32_t)c; }
static void uputs(const char *s) { while (*s) uputc(*s++); }
static void uputu(uint32_t v)
{
    char b[11]; int i = 0;
    if (!v) { uputc('0'); return; }
    while (v) { b[i++] = (char)('0' + v % 10); v /= 10; }
    while (i) uputc(b[--i]);
}
static void log_msg(const char *m) /* "[ 1234 ms] message" */
{ uputs("["); uputu(ms); uputs(" ms] "); uputs(m); uputs("\r\n"); }

static const char *state_name(state_t s)
{
    switch (s) {
    case ST_IDLE: return "IDLE";
    case ST_READY: return "READY";
    case ST_RUNNING: return "RUNNING";
    default: return "EMERGENCY";
    }
}

/* ---------------------------- 6. OUTPUTS: LEDS + MOTOR ------------------- */
static void leds(uint8_t o, uint8_t g, uint8_t b, uint8_t r)
{ /* one BSRR write -> all four LEDs change together */
    uint32_t v = 0;
    v |= o ? (1u << LED_ORANGE) : (1u << (LED_ORANGE + 16));
    v |= g ? (1u << LED_GREEN) : (1u << (LED_GREEN + 16));
    v |= b ? (1u << LED_BLUE) : (1u << (LED_BLUE + 16));
    v |= r ? (1u << LED_RED) : (1u << (LED_RED + 16));
    GPIO_BSRR(LED_PORT) = v;
}

static void motor_stop_hw(void)
{
    TIM3_CCR1 = 0; /* PWM = 0 */
    GPIO_BSRR(MOT_PORT) = (1u << (MOT_IN1 + 16)) | (1u << (MOT_IN2 + 16));
}

static void motor_drive(uint32_t pct, uint8_t rev)
{
    if (pct == 0) { motor_stop_hw(); return; }
    if (rev) GPIO_BSRR(MOT_PORT) = (1u << (MOT_IN1 + 16)) | (1u << MOT_IN2);
    else GPIO_BSRR(MOT_PORT) = (1u << MOT_IN1) | (1u << (MOT_IN2 + 16));
    TIM3_CCR1 = pct * (PWM_ARR + 1) / 100;
}

/* The ONLY place LEDs are set from state -> LEDs always match the FSM. */
static void set_state(state_t s)
{
    if (s != ST_RUNNING) motor_stop_hw(); /* safe by default*/
    state = s;
    switch (s) {
    case ST_IDLE: leds(1, 0, 0, 0); break;
    case ST_READY: leds(0, 1, 0, 0); break;
    case ST_RUNNING: leds(0, 0, 1, 0); break;
    default: leds(0, 0, 0, 1); break;
    }
}

/* Main-loop transitions, done atomically so they can't race the E-STOP ISR */
static int go(state_t s)
{
    int ok = 0;
    IRQ_OFF();
    if (s == ST_IDLE && state == ST_ESTOP) { /* RESET path */
        if (!estop_pressed()) { set_state(ST_IDLE); ok = 1; }
    } else if (state != ST_ESTOP) { set_state(s); ok = 1; }
    IRQ_ON();
    return ok;
}

static int try_run(uint32_t pct, uint8_t rev)
{
    int ok = 0;
    IRQ_OFF();
    if (state == ST_READY || state == ST_RUNNING) {
        dir_rev = rev; set_state(ST_RUNNING); motor_drive(pct, rev); ok = 1;
    }
    IRQ_ON();
    return ok;
}

static void live_pwm_update(uint32_t pct)
{
    IRQ_OFF();
    if (state == ST_RUNNING) motor_drive(pct, dir_rev);
    IRQ_ON();
}

/* ---------------------------- 7. ADC -> PWM % ---------------------------- */
static uint32_t adc_once(void)
{
    ADC1_CR2 |= (1u << 30); /* SWSTART */
    while (!(ADC1_SR & (1u << 1))); /* wait EOC */
    return ADC1_DR & 0xFFFu;
}

static void sample_pot(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) sum += adc_once(); /* 8-sample avg */
    cur_adc = sum >> 3;
    if (cur_adc <= ADC_DEAD_LO) cur_pwm = 0;
    else if (cur_adc >= 4095 - ADC_DEAD_HI) cur_pwm = 100;
    else cur_pwm = (cur_adc - ADC_DEAD_LO) * 100 / (4095 - ADC_DEAD_LO - ADC_DEAD_HI);
}

/* ---------------------------- 8. INTERRUPT HANDLERS ---------------------- */
void SysTick_Handler(void) { ms++; }

void USART2_IRQHandler(void)
{
    if (USART2_SR & (1u << 5)) { /* RXNE */
        uint8_t c = (uint8_t)USART2_DR;
        uint8_t n = (uint8_t)((rx_head + 1) & (RXN - 1));
        if (n != rx_tail) { rx_buf[rx_head] = c; rx_head = n; }
    }
}

/* EMERGENCY STOP: kill PWM first, then change state. */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_PR & (1u << ESTOP_PIN)) {
        EXTI_PR = (1u << ESTOP_PIN); /* clear pending */
        motor_stop_hw(); /* PWM = 0, motor OFF */
        set_state(ST_ESTOP); /* red LED only */
    }
}

/* ---------------------------- 9. COMMANDS -------------------------------- */
static int streq(const char *a, const char *b)
{ while (*a && *a == *b) { a++; b++; } return *a == *b; }

static void print_status(void)
{
    uputs("--------------------------------\r\n");
    uputs(" MINI ECU STATUS\r\n");
    uputs("--------------------------------\r\n");
    uputs("STATE : "); uputs(state_name(state)); uputs("\r\n");
    uputs("SYSTEM : ");
    uputs((state == ST_READY || state == ST_RUNNING) ? "ENABLED" : "DISABLED");
    uputs("\r\nMOTOR : "); uputs(state == ST_RUNNING ? "ON" : "OFF");
    uputs("\r\nDIRECTION : "); uputs(dir_rev ? "REVERSE" : "FORWARD");
    uputs("\r\nPWM : ");
    uputu(state == ST_RUNNING ? cur_pwm : 0); uputs("%");
    uputs(" (pot commands "); uputu(cur_pwm); uputs("%)");
    uputs("\r\nADC VALUE : "); uputu(cur_adc);
    uputs("\r\nEMERGENCY : "); uputs(state == ST_ESTOP ? "YES" : "NO");
    uputs("\r\nUPTIME : "); uputu(ms / 1000); uputs(" s\r\n");
    uputs("--------------------------------\r\n");
}

static void handle_cmd(const char *c)
{
    if (!c[0]) return;

    if (streq(c, "F") || streq(c, "R")) {
        uint8_t rev = (c[0] == 'R');
        sample_pot();
        if (state == ST_IDLE) log_msg("IGNORED: system not enabled - touch START first");
        else if (state == ST_ESTOP) log_msg("IGNORED: EMERGENCY active - send RESET");
        else if (cur_pwm == 0) log_msg("PWM = 0% -> motor stays OFF (raise potentiometer)");
        else if (try_run(cur_pwm, rev)) {
            log_msg(rev ? "RUNNING (reverse)" : "RUNNING (forward)");
        }
    }
    else if (streq(c, "S")) {
        if (state == ST_READY || state == ST_RUNNING) {
            if (go(ST_READY)) log_msg("STOP -> READY (PWM=0)");
        } else if (state == ST_ESTOP) log_msg("IGNORED: EMERGENCY active - send RESET");
        else log_msg("IGNORED: system not enabled");
    }
    else if (streq(c, "STATUS")) { print_status(); }
    else if (streq(c, "RESET")) {
        if (state != ST_ESTOP) log_msg("No emergency active");
        else if (estop_pressed()) log_msg("Release the E-STOP button first");
        else if (go(ST_IDLE)) log_msg("EMERGENCY cleared -> IDLE (touch START to enable)");
    }
    else if (streq(c, "HELP")) {
        uputs("Commands: F (forward) R (reverse) S (stop) STATUS RESET HELP\r\n");
    }
    else { log_msg("Unknown command - type HELP"); }
}

/* ---------------------------- 10. INIT ----------------------------------- */
static void init_all(void)
{
    /* clocks: GPIOA/B/C, TIM3, USART2, ADC1, SYSCFG */
    RCC_AHB1ENR |= (1u << 0) | (1u << 1) | (1u << 2);
    RCC_APB1ENR |= (1u << 1) | (1u << 17);
    RCC_APB2ENR |= (1u << 8) | (1u << 14);
    (void)RCC_AHB1ENR;

    /* GPIO */
    pin_mode(LED_PORT, LED_ORANGE, 1); pin_mode(LED_PORT, LED_GREEN, 1);
    pin_mode(LED_PORT, LED_BLUE, 1); pin_mode(LED_PORT, LED_RED, 1);
    pin_mode(MOT_PORT, MOT_IN1, 1); pin_mode(MOT_PORT, MOT_IN2, 1);
    pin_mode(TOUCH_PORT, TOUCH_PIN, 0);
    pin_pull(TOUCH_PORT, TOUCH_PIN, TOUCH_ACTIVE ? 2 : 1);
    pin_mode(ESTOP_PORT, ESTOP_PIN, 0);
    pin_pull(ESTOP_PORT, ESTOP_PIN, ESTOP_ACTIVE ? 2 : 1);
    pin_mode(GPIOA_BASE, 0, 3); /* PA0 analog */
    pin_mode(GPIOA_BASE, 6, 2); pin_af(GPIOA_BASE, 6, 2); /* PA6 TIM3_CH1 */
    pin_mode(GPIOA_BASE, 2, 2); pin_af(GPIOA_BASE, 2, 7); /* PA2 USART2_TX */
    pin_mode(GPIOA_BASE, 3, 2); pin_af(GPIOA_BASE, 3, 7); /* PA3 USART2_RX */

    /* TIM3 CH1 PWM, 1 kHz */
    TIM3_PSC = 15; TIM3_ARR = PWM_ARR; TIM3_CCR1 = 0;
    TIM3_CCMR1 = (6u << 4) | (1u << 3); /* PWM mode 1, preload */
    TIM3_CCER = 1u; /* CH1 output enable */
    TIM3_EGR = 1u; /* load registers */
    TIM3_CR1 = (1u << 7) | 1u; /* ARPE + enable */

    /* ADC1 channel 0, 480-cycle sample time */
    ADC1_SMPR2 = 7u; ADC1_SQR3 = 0; ADC1_SQR1 = 0;
    ADC1_CR2 = 1u; /* ADON */
    for (volatile int i = 0; i < 1000; i++); /* stabilisation */

    /* USART2 115200 @16 MHz, RX interrupt */
    USART2_BRR = (8u << 4) | 11u;
    USART2_CR1 = (1u << 13) | (1u << 3) | (1u << 2) | (1u << 5);

    /* EXTI13 on port C, falling edge (E-STOP) */
    SYSCFG_EXTICR4 = (SYSCFG_EXTICR4 & ~(0xFu << 4)) | (2u << 4);
    EXTI_FTSR |= (1u << ESTOP_PIN);
    EXTI_PR = (1u << ESTOP_PIN);
    EXTI_IMR |= (1u << ESTOP_PIN);

    /* NVIC: USART2 (IRQ38), EXTI15_10 (IRQ40) ; SysTick 1 ms */
    NVIC_ISER1 |= (1u << (38 - 32)) | (1u << (40 - 32));
    SYST_RVR = 16000 - 1; SYST_CVR = 0; SYST_CSR = 7;
}

/* ---------------------------- 11. MAIN ----------------------------------- */
int main(void)
{
    char line[32];
    uint8_t len = 0, touch_prev = 0;
    uint32_t last_adc = 0, last_rx = 0, last_touch = 0;

    init_all();
    sample_pot();
    set_state(estop_pressed() ? ST_ESTOP : ST_IDLE);

    uputs("\r\n=== MINI-ECU (STM32F401, bare-metal) ===\r\n");
    uputs("Commands: F R S STATUS RESET HELP (press Enter)\r\n");
    log_msg(state == ST_ESTOP ? "BOOT: E-STOP held -> EMERGENCY"
                              : "BOOT: IDLE (orange) - touch START to enable");

    for (;;) {
        uint32_t now = ms;

        /* periodic pot sampling (20 ms); PWM follows the pot while RUNNING */
        if (now - last_adc >= 20) {
            last_adc = now;
            sample_pot();
            if (state == ST_RUNNING) {
                if (cur_pwm == 0) { if (go(ST_READY)) log_msg("PWM = 0% -> READY"); }
                else live_pwm_update(cur_pwm);
            }
        }

        /* touch START: rising edge + debounce */
        uint8_t t = (uint8_t)(((GPIO_IDR(TOUCH_PORT) >> TOUCH_PIN) & 1u) == TOUCH_ACTIVE);
        if (t && !touch_prev && (now - last_touch >= TOUCH_DEBOUNCE_MS)) {
            last_touch = now;
            if (state == ST_IDLE) { if (go(ST_READY)) log_msg("TOUCH START -> READY (system enabled)"); }
            else if (state == ST_ESTOP) log_msg("IGNORED: EMERGENCY active - send RESET");
        }
        touch_prev = t;

        /* UART line assembly (ring buffer filled by ISR) */
        while (rx_tail != rx_head) {
            char c = (char)rx_buf[rx_tail];
            rx_tail = (uint8_t)((rx_tail + 1) & (RXN - 1));
            last_rx = now;
            if (c == '\r' || c == '\n') {
                if (len) { uputs("\r\n"); line[len] = 0; handle_cmd(line); len = 0; }
            } else if (len < sizeof(line) - 1) {
                if (c >= 'a' && c <= 'z') c = (char)(c - 32); /* case-insensitive */
                line[len++] = c; uputc(c); /* echo */
            }
        }
        if (len && (now - last_rx >= CMD_IDLE_TIMEOUT_MS)) { /* no Enter sent */
            uputs("\r\n"); line[len] = 0; handle_cmd(line); len = 0;
        }
    }
}