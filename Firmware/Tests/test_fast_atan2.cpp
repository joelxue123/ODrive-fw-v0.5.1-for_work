#include "doctest.h"
#include <cmath>
#include <float.h>
#include "../MotorControl/utils.hpp"


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



TEST_SUITE("fast_atan2 function tests") {
    const float epsilon = 1e-4f; // 允许的误差范围

    TEST_CASE("Quadrant tests") {
        CHECK(fast_atan2(1.0f, 1.0f) == doctest::Approx(M_PI / 4).epsilon(epsilon));
        CHECK(fast_atan2(1.0f, -1.0f) == doctest::Approx(3 * M_PI / 4).epsilon(epsilon));
        CHECK(fast_atan2(-1.0f, -1.0f) == doctest::Approx(-3 * M_PI / 4).epsilon(epsilon));
        CHECK(fast_atan2(-1.0f, 1.0f) == doctest::Approx(-M_PI / 4).epsilon(epsilon));
    }

    TEST_CASE("Axis tests") {
        CHECK(fast_atan2(0.0f, 1.0f) == doctest::Approx(0.0f).epsilon(epsilon));
        CHECK(fast_atan2(1.0f, 0.0f) == doctest::Approx(M_PI / 2).epsilon(epsilon));
        CHECK(fast_atan2(0.0f, -1.0f) == doctest::Approx(M_PI).epsilon(epsilon));
        CHECK(fast_atan2(-1.0f, 0.0f) == doctest::Approx(-M_PI / 2).epsilon(epsilon));
    }

    TEST_CASE("Special cases") {
        CHECK(fast_atan2(0.0f, 0.0f) == doctest::Approx(0.0f).epsilon(epsilon));
        CHECK(fast_atan2(FLT_MIN, 1.0f) == doctest::Approx(0.0f).epsilon(epsilon));
        CHECK(fast_atan2(1.0f, FLT_MIN) == doctest::Approx(M_PI / 2).epsilon(epsilon));
    }

    TEST_CASE("Random values") {
        CHECK(fast_atan2(0.5f, 0.866f) == doctest::Approx(0.5236f).epsilon(epsilon));
        CHECK(fast_atan2(-0.707f, 0.707f) == doctest::Approx(-0.7854f).epsilon(epsilon));
        CHECK(fast_atan2(0.866f, -0.5f) == doctest::Approx(2.0944f).epsilon(epsilon));
    }
}
