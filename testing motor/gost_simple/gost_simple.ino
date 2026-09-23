/* =========================================================================
 * Arduino Uno + драйвер шагового двигателя (STEP/DIR/ENA)
 * Движение START <-> END по экспоненциальному профилю ГОСТ Р ИСО 26782-2016 (прил. C).
 *
 * Порядок работы:
 *   1. КАЛИБРОВКА: едем медленно от START до END и считаем шаги -> travelSteps.
 *   2. Возвращаемся на START.
 *   3. РАБОТА: START -> END и END -> START по профилю. Число шагов = travelSteps,
 *      время каждого шага считается по формуле, импульс выдаётся в этот момент.
 *
 * Формула ГОСТ (положение от времени, доля хода 0...1):
 *
 *   x(t) = [1 - exp(-t/TAU)] / D,   где D = 1 - exp(-T/TAU)
 *
 * Нам нужно обратное: в какой момент t должен быть сделан шаг номер i (x = i / N):
 *
 *   t(i) = -TAU * ln(1 - (i/N) * D)
 *
 * Считаем t(i) для каждого шага и ждём этого момента. Никаких периодов и скоростей.
 *
 * Концевики: INPUT_PULLUP, нормально замкнутые на GND (сработал = HIGH).
 * ========================================================================= */

/* ---------- Пины ---------- */
const uint8_t PUL_PIN = 3;
const uint8_t DIR_PIN = 4;
const uint8_t ENA_PIN = 5;
const uint8_t LIM_START_PIN = 6;
const uint8_t LIM_END_PIN   = 7;

const uint8_t DIR_FORWARD  = HIGH;      // START -> END
const uint8_t DIR_BACKWARD = LOW;       // END -> START

/* ---------- Настройки ---------- */

/* Профиль ГОСТ (из таблицы C.1). C2: TAU = 1.00, T = 8.586. Другие значения см. в исходном скетче. */
const float TAU = 1.00;                 // постоянная времени, с
const float T_FEV = 8.586;              // время всего хода, с

/* Калибровка */
const unsigned int CAL_PERIOD_US = 1000;        // период шага при калибровке (1000 = 1000 шаг/с, медленно и надёжно)
const unsigned long MAX_CAL_STEPS = 500000UL;   // защита: не ехать бесконечно, если концевик не срабатывает
const unsigned long MIN_TRAVEL = 100;           // меньше этого - считаем, что концевики подключены неправильно

/* Импульс и ограничение скорости */
const unsigned int PUL_WIDTH_US = 5;            // ширина импульса STEP
const unsigned long MIN_PERIOD_US = 200;        // не чаще 1 шага в 200 мкс (5000 шаг/с): Uno не успевает быстрее.
                                                // Если профиль требует быстрее - шаги идут с этим периодом,
                                                // потом профиль сам догоняет расписание.
const unsigned long PAUSE_MS = 500;             // пауза на концах

/* ---------- Состояние ---------- */
unsigned long travelSteps = 0;          // результат калибровки
float D = 0;                            // 1 - exp(-T/TAU), считаем один раз

/* ---------- Простые функции ---------- */

bool hit(uint8_t pin)
{
  return digitalRead(pin) == HIGH;
}

void setDir(uint8_t dir)
{
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(10);                // DIR должен установиться до первого шага
}

/* Один импульс STEP */
void pulse()
{
  digitalWrite(PUL_PIN, HIGH);
  delayMicroseconds(PUL_WIDTH_US);
  digitalWrite(PUL_PIN, LOW);
}

/* Остановка при ошибке */
void fail(const __FlashStringHelper *msg)
{
  Serial.println(msg);
  digitalWrite(ENA_PIN, HIGH);          // выключить драйвер
  while (1) { }
}

/* ---------- КАЛИБРОВКА ---------- */

/* Медленно едем назад до концевика START */
void goToStart()
{
  setDir(DIR_BACKWARD);
  unsigned long steps = 0;
  while (!hit(LIM_START_PIN))
  {
    pulse();
    delayMicroseconds(CAL_PERIOD_US);
    if (++steps > MAX_CAL_STEPS) fail(F("Ошибка: START не найден"));
  }
}

/* Едем от START к END и считаем шаги */
unsigned long measureTravel()
{
  goToStart();

  setDir(DIR_FORWARD);
  unsigned long steps = 0;
  while (!hit(LIM_END_PIN))
  {
    pulse();
    delayMicroseconds(CAL_PERIOD_US);
    if (++steps > MAX_CAL_STEPS) fail(F("Ошибка: END не найден"));
  }
  return steps;
}

/* ---------- ПРОФИЛЬ ---------- */

/* Момент (в секундах от старта), когда должен быть сделан шаг номер i из travelSteps */
float stepTime(unsigned long i)
{
  float x = (float)i / (float)travelSteps;      // доля пути 0...1
  return -TAU * log(1.0 - x * D);
}

/* Проезд всего хода в заданную сторону. stopPin - концевик, у которого заканчиваем. */
void moveProfile(uint8_t dir, uint8_t stopPin)
{
  setDir(dir);

  unsigned long t0 = micros();
  unsigned long lastUs = 0;             // время предыдущего шага, мкс от t0
  unsigned long i;

  for (i = 1; i <= travelSteps; i++)
  {
    if (hit(stopPin)) break;            // концевик важнее расчёта

    unsigned long targetUs = (unsigned long)(stepTime(i) * 1000000.0);

    if (targetUs < lastUs + MIN_PERIOD_US)          // ограничение максимальной скорости
      targetUs = lastUs + MIN_PERIOD_US;

    while (micros() - t0 < targetUs) { }            // ждём момента шага

    pulse();
    lastUs = targetUs;
  }

  unsigned long totalMs = (micros() - t0) / 1000;
  Serial.print(F("  шагов: "));
  Serial.print(i - 1);
  Serial.print(F(" из "));
  Serial.print(travelSteps);
  Serial.print(F(", время: "));
  Serial.print(totalMs);
  Serial.print(F(" мс (по ГОСТ "));
  Serial.print((unsigned long)(T_FEV * 1000));
  Serial.println(F(" мс)"));
}

/* ---------- SETUP / LOOP ---------- */

void setup()
{
  Serial.begin(115200);

  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(LIM_START_PIN, INPUT_PULLUP);
  pinMode(LIM_END_PIN, INPUT_PULLUP);

  digitalWrite(PUL_PIN, LOW);
  digitalWrite(ENA_PIN, LOW);           // LOW = драйвер включён

  D = 1.0 - exp(-T_FEV / TAU);

  Serial.println(F("Калибровка..."));
  travelSteps = measureTravel();

  Serial.print(F("Ход между концевиками: "));
  Serial.print(travelSteps);
  Serial.println(F(" шагов"));

  if (travelSteps < MIN_TRAVEL) fail(F("Ошибка: ход слишком мал, проверь концевики"));

  goToStart();
  Serial.println(F("Готово, работаем по профилю ГОСТ."));
  delay(PAUSE_MS);
}

void loop()
{
  Serial.println(F("START -> END"));
  moveProfile(DIR_FORWARD, LIM_END_PIN);
  delay(PAUSE_MS);

  Serial.println(F("END -> START"));
  moveProfile(DIR_BACKWARD, LIM_START_PIN);
  delay(PAUSE_MS);
}
