#include "stepper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "esp_check.h"

/* ---- Пины и полярность ---- */
#define PIN_STEP        4                   /* PUL-  */
#define PIN_DIR         5                   /* DIR-  */
#define STEP_ACTIVE     0                   /* общий анод: оптрон включён при GPIO = 0 */
#define PULSE_US        5                   /* длина импульса STEP */
#define LIMIT_ACTIVE    0                   /* кнопка НР на GND + pull-up: нажатие замыкает на землю, сработал = LOW */

#define TIMER_HZ        1000000             /* 1 тик = 1 мкс */

static gptimer_handle_t s_timer;

static struct {
    const uint32_t *tbl;                    /* таблица пауз или NULL */
    uint32_t tbl_len;                       /* сколько элементов в tbl */
    uint32_t const_period;                  /* постоянная пауза, если tbl == NULL */
    uint32_t max_steps;                     /* предохранитель: авария, если концевик не сработал раньше */
    int stop_pin;                           /* концевик остановки */
    volatile uint32_t done;                 /* сколько сделано */
    volatile int32_t pos;                   /* позиция */
    int32_t dir;                            /* +1 / -1 */
    volatile stepper_state_t state;
} s = { .state = STEPPER_IDLE };

static void IRAM_ATTR finish(gptimer_handle_t timer, stepper_state_t st)
{
    gptimer_stop(timer);
    s.state = st;
}

/* Пауза перед шагом номер i: из таблицы, а после её конца - последним значением таблицы. */
static inline uint32_t IRAM_ATTR period_for(uint32_t i)
{
    if (!s.tbl) return s.const_period;
    uint32_t idx = (i < s.tbl_len) ? i : (s.tbl_len - 1);
    return s.tbl[idx];
}

/* Прерывание таймера: срабатывает в момент каждого шага. Должно быть коротким. */
static bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *ev, void *arg)
{
    /* Концевик - главный: проверяем ДО шага. Сработал - шаг не делаем, это нормальный конец. */
    if (gpio_get_level(s.stop_pin) == LIMIT_ACTIVE) {
        finish(timer, STEPPER_DONE_LIMIT);
        return false;
    }

    gpio_set_level(PIN_STEP, STEP_ACTIVE);
    esp_rom_delay_us(PULSE_US);
    gpio_set_level(PIN_STEP, !STEP_ACTIVE);

    s.pos += s.dir;
    uint32_t done = ++s.done;

    /* Концевик так и не сработал за отведённый запас шагов - это авария, а не норма. */
    if (done >= s.max_steps) {
        finish(timer, STEPPER_SAFETY_STOP);
        return false;
    }

    uint32_t period = period_for(done);
    gptimer_alarm_config_t next = { .alarm_count = ev->alarm_value + period };
    gptimer_set_alarm_action(timer, &next);
    return false;
}

void stepper_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(PIN_STEP, !STEP_ACTIVE);
    gpio_set_level(PIN_DIR, 1);

    gptimer_config_t tc = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_HZ,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tc, &s_timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(s_timer));
}

static void start(bool forward, const uint32_t *tbl, uint32_t tbl_len,
                   uint32_t const_period, uint32_t max_steps, int stop_pin)
{
    stepper_wait();                                 /* не начинаем поверх идущего движения */

    /* Концевик уже сработал - ехать некуда, движение окончено мгновенно. */
    if (gpio_get_level(stop_pin) == LIMIT_ACTIVE) {
        s.state = STEPPER_DONE_LIMIT;
        s.done = 0;
        return;
    }

    gpio_set_level(PIN_DIR, forward ? 1 : 0);
    esp_rom_delay_us(20);                           /* DIR должен установиться до первого шага */

    s.tbl = tbl;
    s.tbl_len = tbl_len;
    s.const_period = const_period;
    s.max_steps = max_steps;
    s.stop_pin = stop_pin;
    s.done = 0;
    s.dir = forward ? 1 : -1;
    s.state = STEPPER_RUNNING;

    uint32_t first = tbl ? tbl[0] : const_period;
    ESP_ERROR_CHECK(gptimer_set_raw_count(s_timer, 0));
    gptimer_alarm_config_t a = { .alarm_count = first };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_timer, &a));
    ESP_ERROR_CHECK(gptimer_start(s_timer));
}

void stepper_run_table(bool forward, const uint32_t *periods_us, uint32_t table_len,
                        uint32_t max_steps, int stop_pin)
{
    start(forward, periods_us, table_len, 0, max_steps, stop_pin);
}

void stepper_run_const(bool forward, uint32_t period_us, uint32_t max_steps, int stop_pin)
{
    start(forward, NULL, 0, period_us, max_steps, stop_pin);
}

stepper_state_t stepper_wait(void)
{
    while (s.state == STEPPER_RUNNING) {
        vTaskDelay(1);
    }
    return s.state;
}

uint32_t stepper_steps_done(void) { return s.done; }
int32_t stepper_position(void) { return s.pos; }
bool stepper_limit_hit(int pin) { return gpio_get_level(pin) == LIMIT_ACTIVE; }
