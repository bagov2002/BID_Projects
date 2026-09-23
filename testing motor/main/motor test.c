/*
 * NEMA 17 + TB6600 + ESP32-C3
 * Движение START <-> END между двумя концевиками по профилю скорости.
 *
 * Слои проекта:
 *   stepper.c  - генератор шагов (GPTimer, прерывание, счёт шагов, концевики)
 *   profile.c  - расчёт профиля в таблицу пауз (ГОСТ-экспонента или трапеция)
 *   этот файл  - логика: калибровка, выбор профиля, рабочий цикл
 *
 * Порядок работы:
 *   1. Калибровка: едем медленно START -> END, считаем шаги (travel_steps).
 *   2. Рассчитываем таблицу пауз для travel_steps шагов.
 *   3. Цикл: START -> END, пауза, END -> START, пауза.
 *
 * Подключение TB6600 (общий анод): PUL+, DIR+ -> 3.3 В; PUL- -> GPIO4; DIR- -> GPIO5.
 * Концевики: GPIO10 = START, GPIO7 = END, нормально разомкнутые (НР) кнопки:
 * один контакт на GPIO, другой на GND. Внутренняя подтяжка к 3.3 В держит вход
 * высоким, нажатие замыкает его на землю, срабатывание = уровень 0.
 * Обрыв провода при такой схеме выглядит как "не нажато" - в отличие от НЗ-схемы,
 * это не защищает от обрыва, зато проще подключить обычную кнопку/концевик.
 */

#include <stdlib.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "stepper.h"
#include "profile.h"

static const char *TAG = "motor";

/* ---------- Настройки ---------- */

#define PIN_LIM_START       10
#define PIN_LIM_END         7

/* Выбор профиля */
#define PROFILE_GOST        1
#define PROFILE_TRAPEZOID   2
#define PROFILE             PROFILE_GOST

/* Калибровка */
#define CAL_PERIOD_US       1000            /* 1000 шаг/с: медленно и надёжно */
#define MAX_CAL_STEPS       500000UL        /* защита от бесконечной езды */
#define MIN_TRAVEL_STEPS    100             /* меньше = концевики подключены неправильно */
#define MAX_TABLE_STEPS     50000UL         /* 4 байта на шаг: 50000 шагов = 200 КБ ОЗУ */
#define TABLE_SAFETY_MULT   2               /* предохранитель рабочего хода = n * это число */

/* Общие ограничения профиля */
#define MIN_PERIOD_US       50              /* не чаще 20 кГц: оптроны TB6600 медленные */

/* ГОСТ, профиль C2 (таблица C.1). Остальные профили: tau и t_fev из таблицы */
#define GOST_TAU            1.00f           /* с */
#define GOST_T_FEV          8.586f          /* с: время всего хода */
#define GOST_ACCEL          50000.0f        /* шаг/с^2: ограничение ускорения на старте (0 = выкл.) */

/* Трапеция (в шагах): подбирается опытом, см. советы по настройке */
#define TRAP_VMAX           16000.0f        /* шаг/с */
#define TRAP_ACCEL          16000.0f        /* шаг/с^2 */
#define TRAP_VSTART         200.0f          /* шаг/с */

#define PAUSE_MS            500

/* ---------- Калибровка ---------- */

static void fatal(const char *msg)
{
    ESP_LOGE(TAG, "%s", msg);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

/* Ехать назад до концевика START. Если уже там - stepper_run_const вернётся мгновенно. */
static void go_to_start(void)
{
    stepper_run_const(false, CAL_PERIOD_US, MAX_CAL_STEPS, PIN_LIM_START);
    if (stepper_wait() != STEPPER_DONE_LIMIT) fatal("START не найден (проверь концевик и его тип NC)");
}

/* START -> END, считаем шаги */
static uint32_t calibrate(void)
{
    ESP_LOGI(TAG, "Калибровка: ищу START...");
    go_to_start();

    ESP_LOGI(TAG, "Калибровка: START -> END...");
    stepper_run_const(true, CAL_PERIOD_US, MAX_CAL_STEPS, PIN_LIM_END);
    if (stepper_wait() != STEPPER_DONE_LIMIT) fatal("END не найден (проверь концевик и его тип NC)");

    uint32_t steps = stepper_steps_done();
    if (steps < MIN_TRAVEL_STEPS) fatal("Ход слишком мал, проверь концевики");
    if (steps > MAX_TABLE_STEPS) fatal("Ход больше MAX_TABLE_STEPS, не хватит памяти под таблицу");

    ESP_LOGI(TAG, "Ход между концевиками: %u шагов", (unsigned)steps);
    return steps;
}

/* ---------- Движение по профилю ---------- */

/*
 * Едет в заданную сторону, пока не сработает концевик. Таблица задаёт форму
 * скорости на первых n шагах; если концевик к этому моменту ещё не сработал
 * (мотор проскользнул, реальный ход чуть длиннее калибровки и т.п.), движение
 * не останавливается, а продолжается на последней паузе таблицы. Это нормально
 * и ожидаемо, если мотор не пропускает шаги массово. STEPPER_SAFETY_STOP -
 * авария: концевик не сработал даже за n * TABLE_SAFETY_MULT шагов.
 */
static void run_profile(bool forward, const uint32_t *table, uint32_t n)
{
    int stop_pin = forward ? PIN_LIM_END : PIN_LIM_START;

    int64_t t0 = esp_timer_get_time();
    stepper_run_table(forward, table, n, n * TABLE_SAFETY_MULT, stop_pin);
    stepper_state_t st = stepper_wait();
    int64_t us = esp_timer_get_time() - t0;

    if (st != STEPPER_DONE_LIMIT) {
        fatal("Концевик не сработал в пределах предохранителя: мотор теряет шаги "
              "или концевик неисправен. Снизь скорость/ускорение профиля.");
    }

    ESP_LOGI(TAG, "%s: %u шагов за %.3f с (расчётных было %u)",
             forward ? "START -> END" : "END -> START",
             (unsigned)stepper_steps_done(), us / 1e6f, (unsigned)n);
}

void app_main(void)
{
    stepper_init();
    gpio_config_t lim = {
        .pin_bit_mask = (1ULL << PIN_LIM_START) | (1ULL << PIN_LIM_END),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&lim));

    uint32_t n = calibrate();
    go_to_start();

    uint32_t *table = malloc(n * sizeof(uint32_t));
    if (!table) fatal("Не хватило памяти под таблицу профиля");

#if PROFILE == PROFILE_GOST
    profile_build_gost(table, n, GOST_TAU, GOST_T_FEV, MIN_PERIOD_US, GOST_ACCEL);
    ESP_LOGI(TAG, "Профиль ГОСТ: tau=%.2f, T=%.3f с", GOST_TAU, GOST_T_FEV);
#else
    profile_build_trapezoid(table, n, TRAP_VMAX, TRAP_ACCEL, TRAP_VSTART);
    ESP_LOGI(TAG, "Профиль: трапеция");
#endif

    while (1) {
        run_profile(true, table, n);
        vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
        run_profile(false, table, n);
        vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
    }
}
