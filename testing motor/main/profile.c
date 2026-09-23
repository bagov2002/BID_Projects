#include "profile.h"

#include <math.h>

void profile_build_gost(uint32_t *p, uint32_t n, float tau, float t_fev,
                        uint32_t min_period_us, float accel_sps2)
{
    double D = 1.0 - exp(-(double)t_fev / tau);
    uint64_t actual = 0;                            /* время последнего выданного шага, мкс */

    for (uint32_t i = 1; i <= n; i++) {
        double x = (double)i / n;
        uint64_t ideal = (uint64_t)llround(-tau * log(1.0 - x * D) * 1e6);   /* когда шаг i должен быть по формуле */

        uint32_t min_p = min_period_us;
        if (accel_sps2 > 0) {                       /* v не больше sqrt(2*a*i) */
            uint32_t pa = (uint32_t)(1e6f / sqrtf(2.0f * accel_sps2 * i));
            if (pa > min_p) min_p = pa;
        }

        uint32_t period = ideal > actual ? (uint32_t)(ideal - actual) : 0;
        if (period < min_p) period = min_p;         /* если отстаём, расписание догоняется само */

        p[i - 1] = period;
        actual += period;
    }
}

void profile_build_trapezoid(uint32_t *p, uint32_t n, float vmax, float accel, float vstart)
{
    for (uint32_t i = 0; i < n; i++) {
        float v_acc = sqrtf(2.0f * accel * (i + 1));        /* ограничение разгоном */
        float v_dec = sqrtf(2.0f * accel * (n - i));        /* ограничение торможением */
        float v = v_acc < v_dec ? v_acc : v_dec;
        if (v > vmax) v = vmax;
        if (v < vstart) v = vstart;
        p[i] = (uint32_t)(1e6f / v);
    }
}
