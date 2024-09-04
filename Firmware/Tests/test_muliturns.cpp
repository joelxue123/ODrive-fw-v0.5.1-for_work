
#include <doctest.h>
#include <algorithm>
#include <cstring>




    // Mock encoder class
class MockEncoder {
    public:
        static constexpr float HALF_CPR = 2048.0f;
        int GearboxOutputEncoder_turns_ = 0;
        float GearboxOutputEncoder_count_in_cpr_ = 0;
        float GearboxOutputEncoder_counts = 0;
        float Gearoffset = 100.0f;
        bool  first_init = true;
        float gear_delta_enc =0;

        void updatePosition(float second_pos_abs) {
            float gear_single_turn_abs = second_pos_abs;

            while(gear_single_turn_abs > HALF_CPR) {
                gear_single_turn_abs -= 2 * HALF_CPR;
            } 
            while(gear_single_turn_abs < -HALF_CPR) {
                gear_single_turn_abs += 2 * HALF_CPR;
            }
            gear_single_turn_abs = gear_single_turn_abs - Gearoffset;
            while(gear_single_turn_abs > HALF_CPR) {
                gear_single_turn_abs -= 2 * HALF_CPR;
            }
            while(gear_single_turn_abs < -HALF_CPR) {
                gear_single_turn_abs += 2 * HALF_CPR;
            }

            if(first_init == true)
            {
                GearboxOutputEncoder_count_in_cpr_ = gear_single_turn_abs;
                first_init = false;
            }

            gear_delta_enc = gear_single_turn_abs - GearboxOutputEncoder_count_in_cpr_;
            GearboxOutputEncoder_count_in_cpr_ = gear_single_turn_abs;

            if (gear_delta_enc > HALF_CPR) {
                GearboxOutputEncoder_turns_ -= 1;
            } else if (gear_delta_enc < -HALF_CPR) {
                GearboxOutputEncoder_turns_ += 1;
            }

            GearboxOutputEncoder_counts = GearboxOutputEncoder_turns_ * (2 * HALF_CPR) + gear_single_turn_abs;
        }
};

MockEncoder encoder;


TEST_SUITE("muliturns Functions") {

    TEST_CASE("CrossPositiveHalfCPR") {
  
        encoder.updatePosition(2200);
        CHECK(encoder.gear_delta_enc == 0);
        CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == -1996);
        encoder.updatePosition(2300);
        CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == -1896);
        CHECK(encoder.gear_delta_enc == 100);
        CHECK(encoder.GearboxOutputEncoder_counts == -1896);
        CHECK(encoder.GearboxOutputEncoder_turns_ == 0);
        encoder.updatePosition(4000);
        CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == -196);
        CHECK(encoder.gear_delta_enc == 1700);
        CHECK(encoder.GearboxOutputEncoder_counts == -196);
        CHECK(encoder.GearboxOutputEncoder_turns_ == 0);

        encoder.updatePosition(200);
        CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == 100);
        CHECK(encoder.gear_delta_enc == 296);
        CHECK(encoder.GearboxOutputEncoder_counts == 100);
        CHECK(encoder.GearboxOutputEncoder_turns_ == 0);

        encoder.updatePosition(2000);
        CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == 1900);
        CHECK(encoder.gear_delta_enc == 1800);
        CHECK(encoder.GearboxOutputEncoder_counts == 1900);
        CHECK(encoder.GearboxOutputEncoder_turns_ == 0);

        encoder.updatePosition(3000);
        CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == -1196);
        CHECK(encoder.gear_delta_enc == -3096);
        CHECK(encoder.GearboxOutputEncoder_counts == 2900);
        CHECK(encoder.GearboxOutputEncoder_turns_ == 1);

    }

    TEST_CASE("MultipleFullRotations") {
    encoder = MockEncoder(); // Reset encoder
    for (int i = 0; i < 5; i++) {
        encoder.updatePosition(i * 2047);
    }
    CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == -104);
    CHECK(encoder.GearboxOutputEncoder_counts == 8088);
    CHECK(encoder.GearboxOutputEncoder_turns_ == 2);
}

TEST_CASE("MultipleFullRotations-negtiave") {
    encoder = MockEncoder(); // Reset encoder
    
    encoder.updatePosition(2047);
    encoder.updatePosition(0);
    encoder.updatePosition(2049);
    encoder.updatePosition(2);
    CHECK(encoder.GearboxOutputEncoder_count_in_cpr_ == -98);
    CHECK(encoder.GearboxOutputEncoder_counts == -4194);
    CHECK(encoder.GearboxOutputEncoder_turns_ == -1);
}

}