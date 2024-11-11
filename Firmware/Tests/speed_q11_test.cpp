#include <doctest.h>
#include <cmath>

TEST_CASE("Encoder velocity calculations") {
    SUBCASE("Q11 fixed point conversion") {
        float test_vel = 1.5f;
        int32_t expected_q11 = (int32_t)(test_vel * 2048.0f);
        CHECK(expected_q11 == 3072);

        CHECK((int32_t)(0.0f * 2048.0f) == 0);
        CHECK((int32_t)(-1.5f * 2048.0f) == -3072);
    }

    SUBCASE("Speed coefficient calculation") {
        float config_speed_base = 12.0f * M_PI;
        float gear_ratio = 10.0f;
        float speed_coeff = 2*M_PI/(config_speed_base)/gear_ratio;
        CHECK(speed_coeff == doctest::Approx(0.0166666667f));

        CHECK(2*M_PI/(config_speed_base)/20.0f == doctest::Approx(0.0083333333f));
    }

    SUBCASE("Complete velocity pipeline") {
        int32_t vel_estimate_q11 = 1024;
        float speed_coeff = 2*M_PI/(12.0f * M_PI)/10.0f;
        int32_t speed_coeff_q11 = (int32_t)(2048.0f * speed_coeff);
        int32_t final_vel = ((vel_estimate_q11 * speed_coeff_q11) >> 11) + 2048;
        
        CHECK(final_vel ==2065);


        int32_t zero_vel = ((0 * speed_coeff_q11) >> 11) + 2048;
        CHECK(zero_vel == 2048);
    }
}
