
#include <cmsis_os.h>
#include <float.h>
#include <math.h>
#include <stm32f4xx_hal.h>

#include <utils.hpp>

// poly = x
const uint8_t crcTable[256] = {
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96, 0xBA, 0x2D, 0x03, 0x94,
    0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C, 0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE,
    0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75, 0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76,
    0x58, 0xCF, 0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7, 0xEB, 0x7C, 0x52,
    0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D, 0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79,
    0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24, 0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0,
    0x27, 0x09, 0x9E, 0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34, 0x18, 0x8F,
    0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E, 0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D,
    0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7, 0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88,
    0x43, 0xD4, 0xFA, 0x6D, 0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65, 0x49,
    0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF, 0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62,
    0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86, 0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E,
    0xD9, 0x12, 0x85, 0xAB, 0x3C};

uint8_t calcCRC(uint8_t* buffer, uint8_t length) {
    uint8_t temp = *buffer++;
    while (--length) {
        temp = *buffer++ ^ crcTable[temp];
    }

    return crcTable[temp];
}

int SVM(float alpha, float beta, float* tA, float* tB, float* tC) {
    int Sextant;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            // quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2;  // sextant v2-v3
            else
                Sextant = 1;  // sextant v1-v2

        } else {
            // quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3;  // sextant v3-v4
            else
                Sextant = 2;  // sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            // quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5;  // sextant v5-v6
            else
                Sextant = 6;  // sextant v6-v1
        } else {
            // quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4;  // sextant v4-v5
            else
                Sextant = 5;  // sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }

    // if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid =
        *tA >= 0.0f && *tA <= 1.0f && *tB >= 0.0f && *tB <= 1.0f && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid ? 0 : -1;
}

// based on https://math.stackexchange.com/a/1105038/81278
float fast_atan2(float y, float x) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = MACRO_MIN(abs_x, abs_y) / (MACRO_MAX(abs_x, abs_y) + FLT_MIN);
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x)
        r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f)
        r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f)
        r = -r;

    return r;
}

// Evaluate polynomials using Fused Multiply Add intrisic instruction.
// coeffs[0] is highest order, as per numpy.polyfit
// p(x) = coeffs[0] * x^deg + ... + coeffs[deg], for some degree "deg"
float horner_fma(float x, const float* coeffs, size_t count) {
    float result = 0.0f;
    for (size_t idx = 0; idx < count; ++idx)
        result = fmaf(result, x, coeffs[idx]);
    return result;
}

// Modulo (as opposed to remainder), per https://stackoverflow.com/a/19288271
int mod(int dividend, int divisor) {
    int r = dividend % divisor;
    return (r < 0) ? (r + divisor) : r;
}

// @brief: Returns how much time is left until the deadline is reached.
// If the deadline has already passed, the return value is 0 (except if
// the deadline is very far in the past)
uint32_t deadline_to_timeout(uint32_t deadline_ms) {
    uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
    uint32_t timeout_ms = deadline_ms - now_ms;
    return (timeout_ms & 0x80000000) ? 0 : timeout_ms;
}

// @brief: Converts a timeout to a deadline based on the current time.
uint32_t timeout_to_deadline(uint32_t timeout_ms) {
    uint32_t now_ms = (uint32_t)((1000ull * (uint64_t)osKernelSysTick()) / osKernelSysTickFrequency);
    return now_ms + timeout_ms;
}

// @brief: Returns a non-zero value if the specified system time (in ms)
// is in the future or 0 otherwise.
// If the time lies far in the past this may falsely return a non-zero value.
int is_in_the_future(uint32_t time_ms) {
    return deadline_to_timeout(time_ms);
}

// @brief: Returns number of microseconds since system startup
uint32_t micros(void) {
    register uint32_t ms, cycle_cnt;
    do {
        ms = HAL_GetTick();
        cycle_cnt = TIM_TIME_BASE->CNT;
    } while (ms != HAL_GetTick());

    return (ms * 1000) + cycle_cnt;
}

// @brief: Busy wait delay for given amount of microseconds (us)
void delay_us(uint32_t us) {
    uint32_t start = micros();
    while (micros() - start < (uint32_t)us) {
        __ASM("nop");
    }
}

int32_t fsgn(float num) {
    if (num > 0.0f) {
        return 1;
    } else if (num < 0.0f) {
        return -1;
    } else {
        return 0;
    }
}

int32_t saturation(int32_t num, int32_t min, int32_t max) 
{
    if (num > max) {
        return max;
    } else if (num < min) {
        return min;
    } else {
        return num;
    }
}


void initNotchFilter(NotchFilter* filter, float frequency, float sampleRate, float bandwidth) {
    float w0 = 2 * M_PI * frequency / sampleRate;
    filter->w0 = w0;
    filter->r = 1 - bandwidth / 2;
    filter->r = our_arm_sin_f32(w0) / (2*bandwidth);
    float cosw0 = our_arm_cos_f32(w0);
    // filter->b1 = -2 * cosw0;
    // filter->b2 = 1;
    // filter->a1 = -2 * filter->r * cosw0;
    // filter->a2 = filter->r * filter->r;
    // filter->a2 = 1.0f - 2*filter->r ;
    filter->b1 = -1.9996150f;//-1.99975573206256f;
    filter->b2 = 1.0f;
    filter->a1 = -1.9686884f;
    filter->a2 = 0.9690674f;

    filter->x1 = filter->x2 = filter->y1 = filter->y2 = 0;
}

float applyNotchFilter(NotchFilter* filter, float input) {
    float output = input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                   - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}
