#include <doctest.h>
#include <algorithm>
#include <cstring>
#include <cmath>

#define NUM_LINEARITY_SEG 4
float L_Slop_Array_N_[4] = {0,1,2,3};
float L_Slop_Array_P_[4] = {0,1,2,3};
float get_torque_constant(float torque_setpoint_notch_filterd_)
{
        float torque_constant = 0;
        float torque_setpoint_abs = fabsf(torque_setpoint_notch_filterd_);
        unsigned int  idex = (unsigned int)((torque_setpoint_abs*1)); 
        if( idex > (NUM_LINEARITY_SEG -2) )
        {
            idex = NUM_LINEARITY_SEG -1;
            torque_constant = L_Slop_Array_N_[idex];
        }
        else
        {
            if(torque_setpoint_notch_filterd_ > 0.0f)
            {
                torque_constant = L_Slop_Array_P_[idex]*( 1.0f - torque_setpoint_abs+ (unsigned int)torque_setpoint_abs ) + L_Slop_Array_P_[idex+1]*( torque_setpoint_abs- (unsigned int)torque_setpoint_abs);
            }
            else
            {
                torque_constant = L_Slop_Array_N_[idex]*( 1.0f - torque_setpoint_abs+ (unsigned int)torque_setpoint_abs ) + L_Slop_Array_N_[idex+1]*( torque_setpoint_abs- (unsigned int)torque_setpoint_abs);
            }
        }
        return torque_constant;
}

TEST_SUITE("Test positive values within range") {
    TEST_CASE("Test positive values within range") {

        CHECK(get_torque_constant(0.5f) == (0.5f));
        CHECK(get_torque_constant(1.5f) == (1.5f));
        CHECK(get_torque_constant(2.5f) == (2.5f));
         CHECK(get_torque_constant(-0.5f) == doctest::Approx(0.5f));
        CHECK(get_torque_constant(-1.5f) == doctest::Approx(1.5f));
        CHECK(get_torque_constant(-2.5f) == doctest::Approx(2.5f));
        CHECK(get_torque_constant(0.0f) == doctest::Approx(0.0f));
        CHECK(get_torque_constant(1.0f) == doctest::Approx(1.0f));
        CHECK(get_torque_constant(2.0f) == doctest::Approx(2.0f));
        CHECK(get_torque_constant(3.0f) == doctest::Approx(3.0f));
        CHECK(get_torque_constant(3.5f) == doctest::Approx(3.0f));
        CHECK(get_torque_constant(4.0f) == doctest::Approx(3.0f));
        CHECK(get_torque_constant(-3.5f) == doctest::Approx(3.0f));
        CHECK(get_torque_constant(-4.0f) == doctest::Approx(3.0f));
        CHECK(get_torque_constant(-5.0f) == doctest::Approx(3.0f));
        CHECK(get_torque_constant(5.0f) == doctest::Approx(3.0f));
    }
}