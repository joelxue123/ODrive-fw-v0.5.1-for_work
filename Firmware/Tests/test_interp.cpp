#include <doctest.h>
#include <algorithm>
#include <cstring>
#include <cmath>

#define NUM_LINEARITY_SEG 4
float L_Slop_Array_N_[4] = {1,2,3,4};
float L_Slop_Array_P_[4] = {1,2,3,4};
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
                torque_constant = L_Slop_Array_P_[idex]*( 1.0f - torque_setpoint_notch_filterd_+ (unsigned int)torque_setpoint_notch_filterd_ ) + L_Slop_Array_P_[idex+1]*( torque_setpoint_notch_filterd_- (unsigned int)torque_setpoint_abs);
            }
            else
            {
                torque_constant = L_Slop_Array_N_[idex]*( 1.0f - torque_setpoint_notch_filterd_+ (unsigned int)torque_setpoint_notch_filterd_ ) + L_Slop_Array_N_[idex+1]*( torque_setpoint_notch_filterd_- (unsigned int)torque_setpoint_abs);
            }
        }
        return torque_constant;
}