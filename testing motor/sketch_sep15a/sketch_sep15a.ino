/* =========================================================================
 * HY-DIV268N + Arduino Uno
 *
 * Движение между двумя концевиками по экспоненциальному профилю
 * ГОСТ Р ИСО 26782-2016, Приложение C.
 *
 * START -> END -> START -> ...
 *
 * Модель ГОСТ:
 *
 *   V(t) = FEV * [1 - exp(-t / TAU)]
 *
 * Скорость:
 *
 *   Q(t) = dV/dt
 *        = (FEV / TAU) * exp(-t / TAU)
 *
 * Для двигателя профиль нормируется на полный ход.
 *
 * ========================================================================= */


/* ===================================================================== */
/* ПИНЫ */
/* ===================================================================== */

const uint8_t PUL_PIN = 3;
const uint8_t DIR_PIN = 4;
const uint8_t ENA_PIN = 5;

const uint8_t LIM_START_PIN = 6;
const uint8_t LIM_END_PIN   = 7;


/* ===================================================================== */
/* НАСТРОЙКИ */
/* ===================================================================== */

/*
 * Сколько STEP требуется от одного концевика до другого.
 *
 * !!! ЭТО ВАЖНЫЙ ПАРАМЕТР !!!
 *
 * Его нужно установить равным реальному количеству шагов
 * между START и END.
 *
 * Например:
 *
 * 20000 шагов
 * 40000 шагов
 * и т.д.
 */
const unsigned long TRAVEL_STEPS = 20000UL;


/*
 * Минимальная ширина STEP.
 */
const unsigned int PUL_WIDTH_US = 5;


/*
 * Выбранный профиль ГОСТ.
 *
 * Можно менять:
 *
 * 1  = C1
 * 2  = C2
 * ...
 * 13 = C13
 */
const uint8_t PROFILE_NUMBER = 2;


/*
 * Небольшая задержка после достижения концевика.
 */
const unsigned long END_PAUSE_MS = 200;


/*
 * Антидребезг.
 */
#define DEBOUNCE_MS 5UL


/* ===================================================================== */
/* НАПРАВЛЕНИЯ */
/* ===================================================================== */

const uint8_t DIR_FORWARD  = HIGH;
const uint8_t DIR_BACKWARD = LOW;


/* ===================================================================== */
/* ПРОФИЛЬ ГОСТ */
/* ===================================================================== */

struct GostProfile
{
  float exponentialVolume;  // Экспоненциальный объём, л
  float tau;                // Постоянная времени, с
  float tFEV;               // T_FЖЕЛ, с
  unsigned int riseTime_ms; // tR, мс
};


/*
 * Таблица из ГОСТ Р ИСО 26782-2016,
 * таблица C.1.
 */
const GostProfile profiles[] =
{
  /* C1  */
  { 7.120, 1.00,  2.658, 114 },

  /* C2  */
  { 5.179, 1.00,  8.586,  38 },

  /* C3  */
  { 5.119, 1.50, 12.236,  38 },

  /* C4  */
  { 6.652, 1.00,  9.120, 340 },

  /* C5  */
  { 8.027, 0.75,  7.056, 114 },

  /* C6  */
  { 0.609, 0.50,  3.654, 114 },

  /* C7  */
  { 1.769, 2.00,  1.812, 114 },

  /* C8  */
  { 0.535, 1.50,  8.916, 114 },

  /* C9  */
  { 4.095, 1.50, 12.402,  38 },

  /* C10 */
  { 6.438, 0.75, 12.810,  38 },

  /* C11 */
  { 6.639, 0.50,  5.094, 340 },

  /* C12 */
  { 3.480, 1.00, 14.210, 114 },

  /* C13 */
  { 2.406, 2.50,  2.320, 114 }
};


/*
 * Получить выбранный профиль.
 */
const GostProfile& getProfile()
{
  return profiles[PROFILE_NUMBER - 1];
}


/* ===================================================================== */
/* КОНЦЕВИКИ */
/* ===================================================================== */

inline bool rawStartHit()
{
  return digitalRead(LIM_START_PIN) == HIGH;
}


inline bool rawEndHit()
{
  return digitalRead(LIM_END_PIN) == HIGH;
}


static bool dbStartState = false;
static bool dbEndState = false;

static bool startCand = false;
static bool endCand = false;

static unsigned long startCandTime = 0;
static unsigned long endCandTime = 0;


void debounceUpdate()
{
  unsigned long now = millis();

  bool rs = rawStartHit();

  if (rs != startCand)
  {
    startCand = rs;
    startCandTime = now;
  }
  else if ((now - startCandTime) >= DEBOUNCE_MS)
  {
    dbStartState = rs;
  }


  bool re = rawEndHit();

  if (re != endCand)
  {
    endCand = re;
    endCandTime = now;
  }
  else if ((now - endCandTime) >= DEBOUNCE_MS)
  {
    dbEndState = re;
  }
}


inline bool limStartHit()
{
  return dbStartState;
}


inline bool limEndHit()
{
  return dbEndState;
}


/* ===================================================================== */
/* НАПРАВЛЕНИЕ */
/* ===================================================================== */

void setDir(uint8_t dir)
{
  digitalWrite(DIR_PIN, dir);

  /*
   * Небольшое время на установку DIR
   * перед первым STEP.
   */
  delayMicroseconds(10);
}


/* ===================================================================== */
/* ФОРМУЛА ГОСТ */
/* ===================================================================== */

/*
 * Нормированная доля пройденного пути.
 *
 * V(t) = FEV * [1 - exp(-t/tau)]
 *
 * В момент T_FEV:
 *
 * V(T) = FEV * [1 - exp(-T/tau)]
 *
 * Делим одно на другое, чтобы получить
 * от 0 до 1:
 *
 * position =
 *
 * [1 - exp(-t/tau)]
 * -------------------------
 * [1 - exp(-T/tau)]
 */
float gostPosition(float t)
{
  const GostProfile& p = getProfile();

  float T = p.tFEV;
  float tau = p.tau;

  if (t <= 0.0)
    return 0.0;

  if (t >= T)
    return 1.0;

  float denominator = 1.0 - exp(-T / tau);

  if (denominator <= 0.000001)
    return 1.0;

  float numerator = 1.0 - exp(-t / tau);

  float position = numerator / denominator;

  if (position < 0.0)
    position = 0.0;

  if (position > 1.0)
    position = 1.0;

  return position;
}


/*
 * Скорость движения в долях полного хода / секунду.
 *
 * Производная нормированной функции.
 */
float gostSpeed(float t)
{
  const GostProfile& p = getProfile();

  float T = p.tFEV;
  float tau = p.tau;

  if (t < 0.0)
    t = 0.0;

  if (t >= T)
    return 0.0;

  float denominator = 1.0 - exp(-T / tau);

  if (denominator <= 0.000001)
    return 0.0;

  /*
   * dV/dt =
   *
   * exp(-t/tau)
   * ----------------
   * tau * denominator
   */
  float speed =
    exp(-t / tau) /
    (tau * denominator);

  return speed;
}


/*
 * Перевод скорости профиля в STEP/сек.
 */
float gostStepsPerSecond(float t)
{
  float speedFraction = gostSpeed(t);

  return speedFraction * (float)TRAVEL_STEPS;
}


/*
 * Перевод STEP/сек в период STEP в микросекундах.
 */
unsigned long gostPeriodUs(float t)
{
  float stepsPerSecond = gostStepsPerSecond(t);

  if (stepsPerSecond < 1.0)
    stepsPerSecond = 1.0;

  float period = 1000000.0 / stepsPerSecond;

  /*
   * Защита от слишком маленького периода.
   */
  if (period < (float)(PUL_WIDTH_US + 2))
    period = PUL_WIDTH_US + 2;

  return (unsigned long)period;
}


/* ===================================================================== */
/* STEP */
/* ===================================================================== */

void doStep(unsigned long period_us)
{
  digitalWrite(PUL_PIN, HIGH);

  delayMicroseconds(PUL_WIDTH_US);

  digitalWrite(PUL_PIN, LOW);

  if (period_us > PUL_WIDTH_US)
  {
    delayMicroseconds(period_us - PUL_WIDTH_US);
  }
}


/* ===================================================================== */
/* ПЕЧАТЬ ИНФОРМАЦИИ О ПРОФИЛЕ */
/* ===================================================================== */

void printProfile()
{
  const GostProfile& p = getProfile();

  Serial.println();
  Serial.println(F("======================================"));
  Serial.print(F("GOST PROFILE: C"));
  Serial.println(PROFILE_NUMBER);

  Serial.print(F("Exponential volume = "));
  Serial.println(p.exponentialVolume, 3);

  Serial.print(F("TAU = "));
  Serial.println(p.tau, 3);

  Serial.print(F("T_FEV = "));
  Serial.println(p.tFEV, 3);

  Serial.print(F("Rise time tR = "));
  Serial.print(p.riseTime_ms);
  Serial.println(F(" ms"));

  Serial.print(F("Travel steps = "));
  Serial.println(TRAVEL_STEPS);

  Serial.println(F("======================================"));
  Serial.println();
}


/* ===================================================================== */
/* ДВИЖЕНИЕ ПО ПРОФИЛЮ К END */
/* ===================================================================== */

void moveToEndGost()
{
  setDir(DIR_FORWARD);

  unsigned long startTime = micros();

  unsigned long lastPrint = 0;

  while (1)
  {
    debounceUpdate();

    /*
     * Концевик имеет абсолютный приоритет.
     */
    if (limEndHit())
    {
      break;
    }


    unsigned long elapsed_us = micros() - startTime;

    float t = elapsed_us / 1000000.0;


    /*
     * После T_FEV экспоненциальная часть закончена.
     *
     * Если концевик ещё не сработал,
     * продолжаем с минимальным шагом.
     */
    unsigned long period;

    if (t < getProfile().tFEV)
    {
      period = gostPeriodUs(t);
    }
    else
    {
      /*
       * Очень медленное движение после
       * расчётного окончания профиля.
       *
       * Это защита на случай, если
       * TRAVEL_STEPS не совпадает с реальным
       * количеством шагов.
       */
      period = 5000;
    }


    doStep(period);


    /*
     * Отладочная информация примерно раз в 500 мс.
     */
    if (millis() - lastPrint >= 500)
    {
      lastPrint = millis();

      float pos = gostPosition(t);
      float sps = gostStepsPerSecond(t);

      Serial.print(F("t="));
      Serial.print(t, 3);

      Serial.print(F("  pos="));
      Serial.print(pos * 100.0, 1);
      Serial.print(F("%"));

      Serial.print(F("  speed="));
      Serial.print(sps, 1);
      Serial.println(F(" step/s"));
    }
  }
}


/* ===================================================================== */
/* ДВИЖЕНИЕ ПО ПРОФИЛЮ К START */
/* ===================================================================== */

void moveToStartGost()
{
  setDir(DIR_BACKWARD);

  unsigned long startTime = micros();

  unsigned long lastPrint = 0;

  while (1)
  {
    debounceUpdate();

    /*
     * Концевик START имеет абсолютный приоритет.
     */
    if (limStartHit())
    {
      break;
    }


    unsigned long elapsed_us = micros() - startTime;

    float t = elapsed_us / 1000000.0;


    unsigned long period;

    if (t < getProfile().tFEV)
    {
      period = gostPeriodUs(t);
    }
    else
    {
      /*
       * Медленное движение до физического
       * концевика, если расчётное время
       * оказалось меньше реального.
       */
      period = 5000;
    }


    doStep(period);


    /*
     * Отладка.
     */
    if (millis() - lastPrint >= 500)
    {
      lastPrint = millis();

      float pos = gostPosition(t);
      float sps = gostStepsPerSecond(t);

      Serial.print(F("t="));
      Serial.print(t, 3);

      Serial.print(F("  pos="));
      Serial.print(pos * 100.0, 1);
      Serial.print(F("%"));

      Serial.print(F("  speed="));
      Serial.print(sps, 1);
      Serial.println(F(" step/s"));
    }
  }
}


/* ===================================================================== */
/* SETUP */
/* ===================================================================== */

void setup()
{
  Serial.begin(115200);


  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);


  pinMode(LIM_START_PIN, INPUT_PULLUP);
  pinMode(LIM_END_PIN, INPUT_PULLUP);


  digitalWrite(PUL_PIN, LOW);


  /*
   * ENA LOW = драйвер включён.
   */
  digitalWrite(ENA_PIN, LOW);


  Serial.println();
  Serial.println(F("======================================"));
  Serial.println(F(" GOST EXPONENTIAL MOTION PROFILE"));
  Serial.println(F("======================================"));


  printProfile();


  /*
   * Инициализация антидребезга.
   */
  for (uint8_t i = 0; i < 20; i++)
  {
    debounceUpdate();
    delay(1);
  }


  Serial.print(F("START = "));
  Serial.println(digitalRead(LIM_START_PIN));


  Serial.print(F("END = "));
  Serial.println(digitalRead(LIM_END_PIN));


  /*
   * Если не на START —
   * сначала ищем START.
   */
  if (!limStartHit())
  {
    Serial.println(F("Seeking START..."));

    /*
     * Здесь используем старое медленное движение,
     * чтобы безопасно найти START.
     */
    setDir(DIR_BACKWARD);

    while (!limStartHit())
    {
      debounceUpdate();

      if (!limStartHit())
      {
        doStep(5000);
      }
    }
  }


  Serial.println(F("START reached."));
  Serial.println(F("Ready."));
}


/* ===================================================================== */
/* LOOP */
/* ===================================================================== */

void loop()
{
  Serial.println();
  Serial.println(F("START -> END"));

  moveToEndGost();

  Serial.println(F("END reached."));

  delay(END_PAUSE_MS);


  Serial.println();
  Serial.println(F("END -> START"));

  moveToStartGost();

  Serial.println(F("START reached."));

  delay(END_PAUSE_MS);
}
