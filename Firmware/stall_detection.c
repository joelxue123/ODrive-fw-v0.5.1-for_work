void Encoder::update_stall_detection() {
    // Stall detection parameters (these could be added to the config struct later)
    static float stall_time_counter = 0.0f; // [s] Counter to track stall time

    // Check if the motor is commanded to move (you might need to adjust this condition based on your control logic)
    bool is_motor_commanded = axis_->controller_.vel_setpoint_ != 0.0f;

    if (is_motor_commanded) {
        if (std::abs(vel_estimate_) < config_.stall_vel_threshold && std::abs(axis_->motor_.current_measured_) > config_.stall_current_threshold) {
            stall_time_counter += current_meas_period; // Assuming current_meas_period is available
            if (stall_time_counter >= config_.stall_time_window) {
                set_error(ERROR_MOTOR_STALLED);
                // Optionally disable the motor here
                axis_->motor_.enable_ = false;
                stall_time_counter = 0.0f; // Reset the counter
            }
        } else {
            stall_time_counter = 0.0f; // Reset the counter if the motor is moving
        }
    } else {
        stall_time_counter = 0.0f; // Reset the counter if the motor is not commanded
    }
}


bool Motor::check_phase_loss(void) {
    PhaseMonitor& pm = phase_monitor_;
    
    // Get actual values
    float va = v_alpha_;  // Applied voltage
    float vb = v_beta_;
    float ia = current_meas_.phB;
    float ib = current_meas_.phC;
    float ic = -(ia + ib);
    float omega = vel_estimate_;
    float theta = phase_;
    
    // Calculate expected voltage from motor equations
    // V = R*I + L*dI/dt + Ke*omega*sin(theta)
    pm.va_expected = pm.Rs * ia + 
                     pm.Ls * dia_dt + 
                     pm.Ke * omega * sinf(theta);
                     
    pm.vb_expected = pm.Rs * ib + 
                     pm.Ls * dib_dt + 
                     pm.Ke * omega * sinf(theta - 2.0f*M_PI/3.0f);
                     
    pm.vc_expected = pm.Rs * ic + 
                     pm.Ls * dic_dt + 
                     pm.Ke * omega * sinf(theta + 2.0f*M_PI/3.0f);
    
    // Check voltage-current relationships
    if (fabsf(va) > pm.MIN_VOLTAGE) {
        float va_error = fabsf(va - pm.va_expected) / fabsf(va);
        float vb_error = fabsf(vb - pm.vb_expected) / fabsf(vb);
        float vc_error = fabsf(vc - pm.vc_expected) / fabsf(vc);
        
        if (va_error > 0.3f || vb_error > 0.3f || vc_error > 0.3f) {
            pm.fault_count++;
        }
    }
    
    return pm.fault_count <= pm.FAULT_THRESHOLD;
}


struct PhaseMonitor {
    // Simple current-based detection
    const float MIN_CURRENT = 0.1f;
    const float IMBALANCE_THRESHOLD = 0.3f;
    const int SAMPLE_COUNT = 100;
    
    // State variables
    float ia_sum, ib_sum, ic_sum;
    int count;
    bool fault;
};

bool check_phase_loss(void) {
    PhaseMonitor* pm = &phase_monitor;
    
    // Get phase currents
    float ia = -current_meas_.phB - current_meas_.phC;
    float ib = current_meas_.phB;
    float ic = current_meas_.phC;
    
    // Accumulate current magnitudes
    pm->ia_sum += fabsf(ia);
    pm->ib_sum += fabsf(ib);
    pm->ic_sum += fabsf(ic);
    
    if (++pm->count >= pm->SAMPLE_COUNT) {
        // Calculate averages
        float ia_avg = pm->ia_sum / pm->SAMPLE_COUNT;
        float ib_avg = pm->ib_sum / pm->SAMPLE_COUNT;
        float ic_avg = pm->ic_sum / pm->SAMPLE_COUNT;
        float mean = (ia_avg + ib_avg + ic_avg) / 3.0f;
        
        // Check imbalance
        if (mean > pm->MIN_CURRENT) {
            pm->fault = (fabsf(ia_avg - mean) > mean * pm->IMBALANCE_THRESHOLD) ||
                       (fabsf(ib_avg - mean) > mean * pm->IMBALANCE_THRESHOLD) ||
                       (fabsf(ic_avg - mean) > mean * pm->IMBALANCE_THRESHOLD);
        }
        
        // Reset accumulators
        pm->ia_sum = pm->ib_sum = pm->ic_sum = 0;
        pm->count = 0;
    }
    
    return !pm->fault;
}




struct MotorProtection {
    // Protection modes
    bool thermal_enable;
    bool current_enable;
    bool i2t_enable;
    
    // Thermal parameters
    float temp_limit;
    float current_limit;
    
    // I2t parameters
    float i2t_integral;
    float i2t_threshold;
    const float tau = 1.0f;
    const float dt = 0.001f;
    uint32_t fault_counter;
};

class Motor {
    MotorProtection protection_;
    bool check_protection(void);
};


bool Motor::check_protection(void) {
    if (!protection_.i2t_enable) return true;
    
    // Calculate I2t with fixed point approximation
    float current = get_current_magnitude();
    int32_t i2 = (int32_t)(current * current * 65536.0f);  // Q16.16
    int32_t decay = 65536 - (int32_t)((protection_.dt / protection_.tau) * 65536.0f);
    
    protection_.i2t_integral = 
        (protection_.i2t_integral * decay) / 65536.0f + 
        (current * current * protection_.dt);
        
    // Apply limits
    protection_.i2t_integral = fmaxf(0.0f, protection_.i2t_integral);
    
    if (protection_.i2t_integral > protection_.i2t_threshold) {
        protection_.fault_counter++;
        return false;
    }
    
    return true;
}



bool Motor::check_protection(void) {
    if (!protection_.i2t_enable) return true;
    
    // Calculate I2t with fixed point approximation
    float current = get_current_magnitude();

    int32_t decay = 0.99f;
    
    protection_.i2t_integral = 
        decay * protection_.i2t_integral + 
        (current * current * protection_.dt);
        
    
    if (protection_.i2t_integral > protection_.i2t_threshold) {
        protection_.fault_counter++;
        return false;
    }
    
    return true;
}
