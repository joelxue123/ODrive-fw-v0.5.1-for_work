
#include <algorithm>

#include "drv8301.h"
#include "odrive_main.h"




/**
 * @brief This control law adjusts the output voltage such that a predefined
 * current is tracked. A hardcoded integrator gain is used for this.
 * 
 * TODO: this might as well be implemented using the FieldOrientedController.
 */
struct ResistanceMeasurementControlLaw : AlphaBetaFrameController {
    void reset() final {
        test_voltage_ = 0.0f;
        test_mod_ = std::nullopt;
    }

    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final {

        if (Ialpha_beta.has_value()) {
            actual_current_ = Ialpha_beta->first;
            test_voltage_ += (kI * current_meas_period) * (target_current_ - actual_current_);
            I_beta_ += (kIBetaFilt * current_meas_period) * (Ialpha_beta->second - I_beta_);
        } else {
            actual_current_ = 0.0f;
            test_voltage_ = 0.0f;
        }
    
        if (std::abs(test_voltage_) > max_voltage_) {
            test_voltage_ = NAN;
           // return Motor::ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        } else if (!vbus_voltage.has_value()) {
          //  return Motor::ERROR_UNKNOWN_VBUS_VOLTAGE;
        } else {
            float vfactor = 1.0f / ((2.0f / 3.0f) * *vbus_voltage);
            test_mod_ = test_voltage_ * vfactor;
           // return Motor::ERROR_NONE;
        }
    }

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp,
            std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final {
        if (!test_mod_.has_value()) {
          //  return Motor::ERROR_CONTROLLER_INITIALIZING;
        } else {
            *mod_alpha_beta = {*test_mod_, 0.0f};
            *ibus = *test_mod_ * actual_current_;
            return Motor::ERROR_NONE;
        }
    }

    float get_resistance() {
        return test_voltage_ / target_current_;
    }

    float get_Ibeta() {
        return I_beta_;
    }

    const float kI = 1.0f; // [(V/s)/A]
    const float kIBetaFilt = 80.0f;
    float max_voltage_ = 0.0f;
    float actual_current_ = 0.0f;
    float target_current_ = 0.0f;
    float test_voltage_ = 0.0f;
    float I_beta_ = 0.0f; // [A] low pass filtered Ibeta response
    std::optional<float> test_mod_ = NAN;
};




/**
 * @brief This control law toggles rapidly between positive and negative output
 * voltage. By measuring how large the current ripples are, the phase inductance
 * can be determined.
 * 
 * TODO: this method assumes a certain synchronization between current measurement and output application
 */
struct InductanceMeasurementControlLaw : AlphaBetaFrameController {
    void reset() final {
        attached_ = false;
    }

    ODriveIntf::MotorIntf::Error on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<float2D> Ialpha_beta,
            uint32_t input_timestamp) final
    {
        if (!Ialpha_beta.has_value()) {
            return {ODriveIntf::MotorIntf::Error::ERROR_UNKNOWN_CURRENT_MEASUREMENT};
        }

        float Ialpha = Ialpha_beta->first;

        if (attached_) {
            float sign = test_voltage_ >= 0.0f ? 1.0f : -1.0f;
            deltaI_ += -sign * (Ialpha - last_Ialpha_);
        } else {
            start_timestamp_ = input_timestamp;
            attached_ = true;
        }

        last_Ialpha_ = Ialpha;
        last_input_timestamp_ = input_timestamp;

        return ODriveIntf::MotorIntf::Error::ERROR_NONE;
    }

    ODriveIntf::MotorIntf::Error get_alpha_beta_output(
            uint32_t output_timestamp, std::optional<float2D>* mod_alpha_beta,
            std::optional<float>* ibus) final
    {
        test_voltage_ *= -1.0f;
        float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
        *mod_alpha_beta = {test_voltage_ * vfactor, 0.0f};
        *ibus = 0.0f;
        return Motor::ERROR_NONE;
    }

    float get_inductance() {
        // Note: A more correct formula would also take into account that there is a finite timestep.
        // However, the discretisation in the current control loop inverts the same discrepancy
        float dt = (float)(last_input_timestamp_ - start_timestamp_) / (float)TIM_1_8_CLOCK_HZ; // at 216MHz this overflows after 19 seconds
        return std::abs(test_voltage_) / (deltaI_ / dt);
    }

    // Config
    float test_voltage_ = 0.0f;

    // State
    bool attached_ = false;
    float sign_ = 0;

    // Outputs
    uint32_t start_timestamp_ = 0;
    float last_Ialpha_ = NAN;
    uint32_t last_input_timestamp_ = 0;
    float deltaI_ = 0.0f;
};




Motor::Motor(const MotorHardwareConfig_t& hw_config,
             const GateDriverHardwareConfig_t& gate_driver_config,
             Config_t& config) :
        hw_config_(hw_config),
        gate_driver_config_(gate_driver_config),
        config_(config),
        gate_driver_({
            .spiHandle = gate_driver_config_.spi,
            .EngpioHandle = gate_driver_config_.enable_port,
            .EngpioNumber = gate_driver_config_.enable_pin,
            .nCSgpioHandle = gate_driver_config_.nCS_port,
            .nCSgpioNumber = gate_driver_config_.nCS_pin,
        }) {
    update_current_controller_gains();
}


/**
 * @brief Updates the phase PWM timings unless the motor is disarmed.
 *
 * If the motor is armed, the PWM timings come into effect at the next update
 * event (and are enabled if they weren't already), unless the motor is disarmed
 * prior to that.
 * 
 * @param tentative: If true, the update is not counted as "refresh".
 */
void Motor::apply_pwm_timings(uint16_t timings[3], bool tentative) {
    CRITICAL_SECTION() {
        if (odrv.config_.enable_brake_resistor && !brake_resistor_armed) {
            disarm_with_error(ERROR_BRAKE_RESISTOR_DISARMED);
        }

        TIM_HandleTypeDef* htim = timer_;
        TIM_TypeDef* tim = htim->Instance;
        tim->CCR1 = timings[0];
        tim->CCR2 = timings[1];
        tim->CCR3 = timings[2];
        
        if (!tentative) {
            if (is_armed_) {
                // Set the Automatic Output Enable so that the Master Output Enable
                // bit will be automatically enabled on the next update event.
                tim->BDTR |= TIM_BDTR_AOE;
            }
        }
        
        // If a timer update event occurred just now while we were updating the
        // timings, we can't be sure what values the shadow registers now contain,
        // so we must disarm the motor.
        // (this also protects against the case where the update interrupt has too
        // low priority, but that should not happen)
        //if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE)) {
        //    disarm_with_error(ERROR_CONTROL_DEADLINE_MISSED);
        //}
    }
}

/**
 * @brief Arms the PWM outputs that belong to this motor.
 *
 * Note that this does not activate the PWM outputs immediately, it just sets
 * a flag so they will be enabled later.
 * 
 * The sequence goes like this:
 *  - Motor::arm() sets the is_armed_ flag.
 *  - On the next timer update event Motor::timer_update_cb() gets called in an
 *    interrupt context
 *  - Motor::timer_update_cb() runs specified control law to determine PWM values
 *  - Motor::timer_update_cb() calls Motor::apply_pwm_timings()
 *  - Motor::apply_pwm_timings() sets the output compare registers and the AOE
 *    (automatic output enable) bit.
 *  - On the next update event the timer latches the configured values into the
 *    active shadow register and enables the outputs at the same time.
 * 
 * The sequence can be aborted at any time by calling Motor::disarm().
 *
 * @param control_law: An control law that is called at the frequency of current
 *        measurements. The function must return as quickly as possible
 *        such that the resulting PWM timings are available before the next
 *        timer update event.
 * @returns: True on success, false otherwise
 */
bool Motor::arm(PhaseControlLaw<3>* control_law) {
    axis_->mechanical_brake_.release();

    uint32_t mask = cpu_enter_critical();
    
        control_law_ = control_law;

        // Reset controller states, integrators, setpoints, etc.
        axis_->controller_.reset();
        axis_->acim_estimator_.rotor_flux_ = 0.0f;
        if (control_law_) {
            control_law_->reset();
        }
        reset_current_control();
        if (!odrv.config_.enable_brake_resistor || brake_resistor_armed) {
            armed_state_ = 1;
            is_armed_ = true;
        } else {
            error_ |= Motor::ERROR_BRAKE_RESISTOR_DISARMED;
        }
    cpu_exit_critical(mask);

    return true;
}

bool Motor::disarm()
{
    uint32_t mask = cpu_enter_critical();
    bool was_armed = armed_state_ != Motor::ARMED_STATE_DISARMED;
    armed_state_ = Motor::ARMED_STATE_DISARMED;
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(hw_config_.timer); //进入刹车模式 2024-10-11
    cpu_exit_critical(mask);
    return was_armed;

}

void Motor::reset_current_control() {
    current_control_.v_current_control_integral_d = 0.0f;
    current_control_.v_current_control_integral_q = 0.0f;
    current_control_.acim_rotor_flux = 0.0f;
    current_control_.Ibus = 0.0f;
}

// @brief Tune the current controller based on phase resistance and inductance
// This should be invoked whenever one of these values changes.
// TODO: allow update on user-request or update automatically via hooks
void Motor::update_current_controller_gains() {
    // Calculate current control gains
    current_control_.p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    float plant_pole = config_.phase_resistance / config_.phase_inductance;
    current_control_.i_gain = plant_pole * current_control_.p_gain;
}

// @brief Set up the gate drivers
void Motor::DRV8301_setup() {
    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    // Solve for exact gain, then snap down to have equal or larger range as requested
    // or largest possible range otherwise
    constexpr float kMargin = 0.90f;
    constexpr float kTripMargin = 1.0f; // Trip level is at edge of linear range of amplifer
    constexpr float max_output_swing = 1.35f; // [V] out of amplifier
    float max_unity_gain_current = kMargin * max_output_swing * hw_config_.shunt_conductance; // [A]
    float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]

    // Decoding array for snapping gain
    std::array<std::pair<float, DRV8301_ShuntAmpGain_e>, 4> gain_choices = { 
        std::make_pair(10.0f, DRV8301_ShuntAmpGain_10VpV),
        std::make_pair(20.0f, DRV8301_ShuntAmpGain_20VpV),
        std::make_pair(40.0f, DRV8301_ShuntAmpGain_40VpV),
        std::make_pair(80.0f, DRV8301_ShuntAmpGain_80VpV)
    };

    // We use lower_bound in reverse because it snaps up by default, we want to snap down.
    auto gain_snap_down = std::lower_bound(gain_choices.crbegin(), gain_choices.crend(), requested_gain, 
    [](std::pair<float, DRV8301_ShuntAmpGain_e> pair, float val){
        return pair.first > val;
    });

    // If we snap to outside the array, clip to smallest val
    if(gain_snap_down == gain_choices.crend())
       --gain_snap_down;

    // Values for current controller
    phase_current_rev_gain_ = 1.0f / gain_snap_down->first;
    // Clip all current control to actual usable range
    current_control_.max_allowed_current = max_unity_gain_current * phase_current_rev_gain_;
    // Set trip level
    current_control_.overcurrent_trip_level = (kTripMargin / kMargin) * current_control_.max_allowed_current;

    // We now have the gain settings we want to use, lets set up DRV chip
    DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
    DRV8301_enable(&gate_driver_);
    DRV8301_setupSpi(&gate_driver_, local_regs);

    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    local_regs->Ctrl_Reg_2.GAIN = gain_snap_down->second;

    local_regs->SndCmd = true;
    DRV8301_writeData(&gate_driver_, local_regs);
    local_regs->RcvCmd = true;
    DRV8301_readData(&gate_driver_, local_regs);
}

// @brief Checks if the gate driver is in operational state.
// @returns: true if the gate driver is OK (no fault), false otherwise
bool Motor::check_DRV_fault() {
    //TODO: make this pin configurable per motor ch
    GPIO_PinState nFAULT_state = HAL_GPIO_ReadPin(gate_driver_config_.nFAULT_port, gate_driver_config_.nFAULT_pin);
    if (nFAULT_state == GPIO_PIN_RESET) {
        // Update DRV Fault Code
        gate_driver_exported_.drv_fault = (GateDriverIntf::DrvFault)DRV8301_getFaultType(&gate_driver_);
        // Update/Cache all SPI device registers
        // DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
        // local_regs->RcvCmd = true;
        // DRV8301_readData(&gate_driver_, local_regs);
        return false;
    };
    return true;
}

void Motor::set_error(Motor::Error error){
    error_ |= error;
    axis_->error_ |= Axis::ERROR_MOTOR_FAILED;
    safety_critical_disarm_motor_pwm(*this);
    update_brake_current();
}

bool Motor::do_checks() {
    if (!check_DRV_fault()) {
        set_error(ERROR_DRV_FAULT);
        axis_->axis_state_.erro = Axis::ENCOS_ERRO::ENCOS_ERROR_DRV_FAULT;
        return false;
    }

    return true;
}

float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim;
    // Hardware limit
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        current_lim = std::min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage); //gimbal motor is voltage control
    } else {
        current_lim = std::min(current_lim, axis_->motor_.current_control_.max_allowed_current);
    }

    // Apply axis current limiters
    for (const CurrentLimiter* const limiter : axis_->current_limiters_) {
        current_lim = std::min(current_lim, limiter->get_current_limit(config_.current_lim));
    }

    effective_current_lim_ = current_lim;

    return effective_current_lim_;
}

//return the maximum available torque for the motor.
//Note - for ACIM motors, available torque is allowed to be 0.
float Motor::max_available_torque() {
    if (config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float max_torque = effective_current_lim() * config_.torque_constant * current_control_.acim_rotor_flux;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    }
    else {
        float max_torque = effective_current_lim() * config_.torque_constant;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    }
}

void Motor::log_timing(TimingLog_t log_idx) {
    static const uint16_t clocks_per_cnt = (uint16_t)((float)TIM_1_8_CLOCK_HZ / (float)TIM_APB1_CLOCK_HZ);
    uint16_t timing = clocks_per_cnt * htim13.Instance->CNT; // TODO: Use a hw_config

    if (log_idx < TIMING_LOG_NUM_SLOTS) {
        timing_log_[log_idx] = timing;
    }
}


void Motor::pos_linearity_init(void)
{
/* 	int16_t i = 0;
	for(i=0;i<NUM_LINEARITY_SEG-1;i++)
	{
		L_Slop_Array_[i] = (config_.CURRENT_LINEARITY_[i+1] - config_.CURRENT_LINEARITY_[i]) / (config_.Torque_LINEARITY_[i+1] - config_.Torque_LINEARITY_[i]);
	} */
}

float Motor::current_Correct(int32_t Torque_Org)
{
/* 	float slopTotall = 0;
	int32_t Index_A = 0,Index_B = NUM_LINEARITY_SEG-1;
	int32_t i = 0;
    float current_Corrected = 0;
	if(Torque_Org>=config_.Torque_LINEARITY_[0] && Torque_Org<config_.Torque_LINEARITY_[NUM_LINEARITY_SEG-1])//��Ҫ�ж�������һ��
	{
		while(1)
		{
			if(Index_B-Index_A == 1)
			{
				break;
			}
			i = (Index_A + Index_B)/2;
			if(Torque_Org<=config_.Torque_LINEARITY_[i])
			{
				Index_B = i;
			}
			else
			{
				Index_A = i;
			}
		}
		current_Corrected = ((Torque_Org - config_.Torque_LINEARITY_[Index_A])* L_Slop_Array_[Index_A]) + config_.CURRENT_LINEARITY_[Index_A];	
	}
	else if(Torque_Org<config_.Torque_LINEARITY_[0])
	{
		current_Corrected = Torque_Org-config_.Torque_LINEARITY_[0]; 
				if( current_Corrected < - 16384)
		{
			current_Corrected = -16384;
		}
	}	
	else
	{
			slopTotall = (config_.CURRENT_LINEARITY_[NUM_LINEARITY_SEG-1] - config_.CURRENT_LINEARITY_[0])/(config_.Torque_LINEARITY_[NUM_LINEARITY_SEG-1] - config_.Torque_LINEARITY_[0]);
			current_Corrected = ((Torque_Org - config_.Torque_LINEARITY_[NUM_LINEARITY_SEG-1])* slopTotall) + config_.CURRENT_LINEARITY_[NUM_LINEARITY_SEG-1];
			if( current_Corrected> 32767 )
			{
				current_Corrected = 32767;
			}
		
	} */
    
    return 0;
}




void Motor::setting_motor_current_linearity(uint32_t index, float value)
{
    if( index < NUM_LINEARITY_SEG )
    {
        config_.CURRENT_LINEARITY_[index] = value;
    }
	
}

float Motor::get_motor_current_linearity(uint32_t index)
{
    if( index < NUM_LINEARITY_SEG )
    {
        return config_.CURRENT_LINEARITY_[index];
    }
    else
    {
        return 0;
    }
}

void Motor::setting_motor_torque_linearity(uint32_t index, float value)
{
    if(index < NUM_LINEARITY_SEG )
    {
        config_.Torque_LINEARITY_[index] = value;
    }
	
}

float Motor::get_motor_torque_linearity(uint32_t index)
{
    if( index < NUM_LINEARITY_SEG )
    {
        return config_.Torque_LINEARITY_[index];
    }
    else
    {
        return 0;
    }
}

float Motor::get_positive_torque_slope(uint32_t index)
{
    if(index < NUM_LINEARITY_SEG)
    {
        return L_Slop_Array_P_[index];
    }
    else
    {
        return 0;
    }
}
float Motor::get_negative_torque_slope(uint32_t index)
{
    if(index < NUM_LINEARITY_SEG)
    {
        return L_Slop_Array_N_[index];
    }
    else
    {
        return 0;
    }
}

void  Motor::setting_positive_torque_slope(uint32_t index, float value)
{
    if(index < NUM_LINEARITY_SEG )
    {
        L_Slop_Array_P_[index] = value;
        config_.Torque_LINEARITY_[index] = value;
    }
}
void  Motor::setting_negative_torque_slope(uint32_t index, float value)
{
    if(index < NUM_LINEARITY_SEG )
    {
        L_Slop_Array_N_[index] = value;
        config_.CURRENT_LINEARITY_[index] = value;
    }
}

void  Motor::setting_current2torque_slope(uint32_t index, float value)
{
    if(index < 2*NUM_LINEARITY_SEG )
    {
        config_.CURRENT2TORQUE_COEFF[index] = value;
    }
}


float Motor::getting_current2torque_slope(uint32_t index)
{
    if(index < 2*NUM_LINEARITY_SEG)
    {
        return config_.CURRENT2TORQUE_COEFF[index];
    }
    else
    {
        return 0;
    }
}

float Motor::convert_torque_from_current(float current,float *current2torque_coeff,uint32_t coeff_size,float current_step)
{
    uint32_t idex = (uint32_t)((fabsf(current) *current_step)); 
    float torque_constant = 0;

    if(using_old_torque_constant_ == true)
    {
        return current;
    }
    
    if( idex > (coeff_size -1) )
    {
        idex = coeff_size -1;
    }
    
    torque_constant = current2torque_coeff[2*idex + (current < 0.0f)];
    
    return current * torque_constant;
}



float Motor::phase_current_from_adcval(uint32_t ADCValue, float phase_current_gain_coeff) {
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * phase_current_rev_gain_;
    float current = shunt_volt * hw_config_.shunt_conductance * phase_current_gain_coeff;
    return current;
}

//--------------------------------
// Measurement and calibration
//--------------------------------

// TODO check Ibeta balance to verify good motor connection
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {
    static const float kI = 10.0f;                                 // [(V/s)/A]
    static const int num_test_cycles = (int)(3.0f / CURRENT_MEAS_PERIOD); // Test runs for 3s
    float test_voltage = 0.0f;
    
    size_t i = 0;
    axis_->run_control_loop([&](){
        float Ialpha = -(current_meas_.phB + current_meas_.phC);
        test_voltage += (kI * current_meas_period) * (test_current - Ialpha);
        if (test_voltage > max_voltage || test_voltage < -max_voltage)
            return set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE), false;


        log_timing(TIMING_LOG_MEAS_R);
        // Test voltage along phase A
        if (!enqueue_voltage_timings(test_voltage, 0.0f))
            return false; // error set inside enqueue_voltage_timings

        return ++i < num_test_cycles;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    float R = test_voltage / test_current;
    config_.phase_resistance = R;
    return true; // if we ran to completion that means success
}

bool Motor::measure_phase_inductance(float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;

    size_t t = 0;
    axis_->run_control_loop([&](){
        int i = t & 1;
        Ialphas[i] += -current_meas_.phB - current_meas_.phC;

        log_timing(TIMING_LOG_MEAS_L);
        while( timing_log_[TIMING_LOG_MEAS_L] < 7000)
        {
            log_timing(TIMING_LOG_MEAS_L);
        }
        // Test voltage along phase A
        if (!enqueue_voltage_timings(test_voltages[i], 0.0f))
            return false; // error set inside enqueue_voltage_timings
        log_timing(TIMING_LOG_MEAS_L);

        return ++t < (num_cycles << 1);
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (current_meas_period * (float)num_cycles);
    float L = v_L / dI_by_dt;

    config_.phase_inductance = L;
    // TODO arbitrary values set for now
    if (L < 2e-6f || L > 4000e-6f)
        return set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE), false;
    return true;
}


bool Motor::run_calibration() {
    float R_calib_max_voltage = config_.resistance_calib_max_voltage;
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT
        || config_.motor_type == MOTOR_TYPE_ACIM) {
        if (!measure_phase_resistance(config_.calibration_current, R_calib_max_voltage))
            return false;
        if (!measure_phase_inductance(-R_calib_max_voltage, R_calib_max_voltage))
            return false;
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        // no calibration needed
    } else {
        return false;
    }

    update_current_controller_gains();
    
    is_calibrated_ = true;
    return true;
}

bool Motor::enqueue_modulation_timings(float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    if (SVM(mod_alpha, mod_beta, &tA, &tB, &tC) != 0)
    return set_error(ERROR_MODULATION_MAGNITUDE), false;


if( deadtime_compensation_coff_ < 0.0f)
{
    I_phase_ = wrap_pm_pi(I_phase_);

    float Ialpha = current_meas_.phA;
    float Ibeta = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);
    
    // Park transform
    float c_I = our_arm_cos_f32(I_phase_);
    float s_I = our_arm_sin_f32(I_phase_);
    float Id = c_I * Ialpha + s_I * Ibeta;
    float Iq = c_I * Ibeta - s_I * Ialpha;

    Iq_filter2 += Idq_filter_k2_ * (Iq - Iq_filter2);
    Id_filter2 += Idq_filter_k2_ * (Id - Id_filter2);

    float offset_phase = fast_atan2(Iq_filter2, Id_filter2);
    total_phase_for_abc_sign_calculation_  = I_phase_ + offset_phase + M_PI;
    
    total_phase_for_abc_sign_calculation_ = wrap_pm_pi(total_phase_for_abc_sign_calculation_);
    total_phase_for_abc_sign_calculation_ += M_PI;

    abc_sign_calculation(total_phase_for_abc_sign_calculation_, &sign_a_, &sign_b_, &sign_c_);
    

     Aphase_deadtime_compensation_ = ((int16_t)(deadtime_compensation_coff_* TIM_1_8_DEADTIME_CLOCKS))*sign_a_ ;
     Bphase_deadtime_compensation_ = ((int16_t)(deadtime_compensation_coff_ * TIM_1_8_DEADTIME_CLOCKS))*sign_b_ ;
     Cphase_deadtime_compensation_ = ((int16_t)(deadtime_compensation_coff_ * TIM_1_8_DEADTIME_CLOCKS))*sign_c_ ;
}
else
{
    Aphase_deadtime_compensation_ =0;
    Bphase_deadtime_compensation_ =0;
    Cphase_deadtime_compensation_ =0;
}


    next_timings_[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS) + Aphase_deadtime_compensation_;
    next_timings_[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS) + Bphase_deadtime_compensation_ ;
    next_timings_[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS) + Cphase_deadtime_compensation_  ;
    next_timings_valid_ = true;
    safety_critical_apply_motor_pwm_timings(
                *this, next_timings_
            );
    return true;
}

bool Motor::enqueue_voltage_timings(float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);  // float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;

    log_timing(TIMING_LOG_FOC_VOLTAGE);

    if (!enqueue_modulation_timings(mod_alpha, mod_beta))
        return false;
    return true;
}

// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool Motor::FOC_voltage(float v_d, float v_q, float pwm_phase) {
    float c = our_arm_cos_f32(pwm_phase);
    float s = our_arm_sin_f32(pwm_phase);
    float v_alpha = c*v_d - s*v_q;
    float v_beta = c*v_q + s*v_d;

    return enqueue_voltage_timings(v_alpha, v_beta);
}

void Motor::abc_sign_calculation(float phase , int32_t *a, int32_t *b, int32_t *c)
{
    if(phase > 0 && phase <= M_PI/6){
       *a = 1;
       *b = -1;
       *c = -1;
    }
    else if(phase > M_PI/6 && phase <= 3*M_PI/6){
       *a = 1;
       *b = 1;
       *c = -1;
    }
    else if(phase > 3*M_PI/6 && phase <= 5*M_PI/6){
       *a = -1;
       *b = 1;
       *c = -1;
    }
    else if(phase > 5*M_PI/6 && phase <= 7*M_PI/6){
       *a = -1;
       *b = 1;
       *c = 1;
    }
    else if(phase > 7*M_PI/6 && phase <= 3*M_PI/2){
       *a = -1;
       *b = -1;
       *c = 1;
    }
    else if (phase > 3*M_PI/2 && phase <= 11*M_PI/6){
       *a = 1;
       *b = -1;
       *c = 1;
    }
    else if (phase > 11*M_PI/6 && phase <= 2*M_PI){
       *a = 1;
       *b = -1;
       *c = -1;
    }
    else{
       *a = 0;
       *b = 0;
       *c = 0;
    }
}


bool Motor::FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase) {
    // Syntactic sugar
    CurrentControl_t& ictrl = current_control_;

    // For Reporting
    ictrl.Iq_setpoint = Iq_des;

    // Check for current sense saturation
    if (std::abs(current_meas_.phB) > ictrl.overcurrent_trip_level || std::abs(current_meas_.phC) > ictrl.overcurrent_trip_level) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
        axis_->axis_state_.erro = Axis::ENCOS_ERRO::ENCOS_ERROR_CURRENT_LIMIT_VIOLATION;
        return false;
    }

    // Clarke transform
    // float Ialpha = -current_meas_.phB - current_meas_.phC;
    // float Ibeta = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);
    float Ialpha = current_meas_.phA;
    float Ibeta = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);
  //  float Ibeta = one_by_sqrt3 * (-current_meas_.phA - current_meas_.phC  - current_meas_.phC);

    // Park transform
    float c_I = our_arm_cos_f32(I_phase);
    float s_I = our_arm_sin_f32(I_phase);
    float Id = c_I * Ialpha + s_I * Ibeta;
    float Iq = c_I * Ibeta - s_I * Ialpha;
    ictrl.Iq_measured += ictrl.I_measured_report_filter_k * (Iq - ictrl.Iq_measured);
    ictrl.Id_measured += ictrl.I_measured_report_filter_k * (Id - ictrl.Id_measured);

    Iq_filter += Idq_filter_k_ * (Iq - Iq_filter);
    Id_filter += Idq_filter_k_ * (Id - Id_filter);
    
    float dec_vd=0, dec_vq=0,pm_flux_linkage=0;
    pm_flux_linkage =  0.444444f*config_.torque_constant/ (config_.pole_pairs);
    dec_vd = Iq_filter * m_speed_est_fast * config_.phase_inductance;
    dec_vq = Id_filter * m_speed_est_fast * config_.phase_inductance;
    dec_bemf_ = m_speed_est_fast * pm_flux_linkage;

    // Check for violation of current limit
    float I_trip = effective_current_lim() + config_.current_lim_margin;
    if (SQ(Id) + SQ(Iq) > SQ(I_trip)) {
        set_error(ERROR_CURRENT_LIMIT_VIOLATION);
        return false;
    }

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = ictrl.v_current_control_integral_d + Ierr_d * ictrl.p_gain;
    float Vq = ictrl.v_current_control_integral_q + Ierr_q * ictrl.p_gain;

    Vd -=  dec_vd;
    Vq +=  dec_vq + dec_bemf_;  


    ictrl.final_v_d = Vd;
    ictrl.final_v_q = Vq;

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        ictrl.v_current_control_integral_d *= 0.99f;
        ictrl.v_current_control_integral_q *= 0.99f;
    } else {
        ictrl.v_current_control_integral_d += Ierr_d * (ictrl.i_gain * current_meas_period);
        ictrl.v_current_control_integral_q += Ierr_q * (ictrl.i_gain * current_meas_period);
    }

    // Compute estimated bus current
    ictrl.Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    float mod_alpha = c_p * mod_d - s_p * mod_q;
    float mod_beta = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl.final_v_alpha = mod_to_V * mod_alpha;
    ictrl.final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    if (!enqueue_modulation_timings(mod_alpha, mod_beta))
        return false; // error set inside enqueue_modulation_timings
    
    log_timing(TIMING_LOG_FOC_CURRENT);
    return true;
}





// torque_setpoint [Nm]
// phase [rad electrical]
// phase_vel [rad/s electrical]
bool Motor::update(float torque_setpoint, float phase, float phase_vel) {
    float current_setpoint = 0.0f;
    float torque_constant = 0.12f;
    phase *= config_.direction;
    phase_vel *= config_.direction;
    m_speed_est_fast =  phase_vel; 

    // if (config_.motor_type == MOTOR_TYPE_ACIM) {
    //     current_setpoint = torque_setpoint / (config_.torque_constant * fmax(current_control_.acim_rotor_flux, config_.acim_gain_min_flux));
    // }
    // else {
    //     current_setpoint = torque_setpoint / config_.torque_constant;
    // }
    
    if( notch_filter_enable_ )
    {
        torque_setpoint_filterd_ += 0.015f * (torque_setpoint - torque_setpoint_filterd_);
        torque_setpoint_notch_filterd_= applyNotchFilter(&notch_filter_, torque_setpoint_filterd_);
    }
    else
    {
        torque_setpoint_notch_filterd_ = torque_setpoint;
    }
    

    if( using_old_torque_constant_ ==  true)
    {
        current_setpoint = torque_setpoint_notch_filterd_ / (config_.torque_constant );
    }
    else
    {
        float torque_setpoint_abs = fabsf(torque_setpoint_notch_filterd_);
        uint32_t idex = (uint32_t)((torque_setpoint_abs*CALIBRATION_INCREMENT)); 
        const float* torque_constant_array  = torque_setpoint_notch_filterd_ > 0.0f ? L_Slop_Array_P_ : L_Slop_Array_N_;
        if( idex > (NUM_LINEARITY_SEG -2) )
        {
            idex = NUM_LINEARITY_SEG -1;
            torque_constant = torque_constant_array [idex];
        }
        else
        {
            torque_constant = torque_constant_array [idex]*( 1.0f - torque_setpoint_abs+ (uint32_t)torque_setpoint_abs ) + torque_constant_array [idex+1]*( torque_setpoint_abs- (uint32_t)torque_setpoint_abs);
        }

        current_setpoint = torque_setpoint_notch_filterd_ / torque_constant;


    }
    current_setpoint *= config_.direction;

    // TODO: 2-norm vs independent clamping (current could be sqrt(2) bigger)
    float ilim = effective_current_lim();
    float id = std::clamp(current_control_.Id_setpoint, -ilim, ilim);
    float iq = std::clamp(current_setpoint, -ilim, ilim);

    if (config_.motor_type == MOTOR_TYPE_ACIM) {
        // Note that the effect of the current commands on the real currents is actually 1.5 PWM cycles later
        // However the rotor time constant is (usually) so slow that it doesn't matter
        // So we elect to write it as if the effect is immediate, to have cleaner code

        if (config_.acim_autoflux_enable) {
            float abs_iq = fabsf(iq);
            float gain = abs_iq > id ? config_.acim_autoflux_attack_gain : config_.acim_autoflux_decay_gain;
            id += gain * (abs_iq - id) * current_meas_period;
            id = std::clamp(id, config_.acim_autoflux_min_Id, ilim);
            current_control_.Id_setpoint = id;
        }

        // acim_rotor_flux is normalized to units of [A] tracking Id; rotor inductance is unspecified
        float dflux_by_dt = config_.acim_slip_velocity * (id - current_control_.acim_rotor_flux);
        current_control_.acim_rotor_flux += dflux_by_dt * current_meas_period;
        float slip_velocity = config_.acim_slip_velocity * (iq / current_control_.acim_rotor_flux);
        // Check for issues with small denominator. Polarity of check to catch NaN too
        bool acceptable_vel = fabsf(slip_velocity) <= 0.1f * (float)current_meas_hz;
        if (!acceptable_vel)
            slip_velocity = 0.0f;
        phase_vel += slip_velocity;
        // reporting only:
        current_control_.async_phase_vel = slip_velocity;

        current_control_.async_phase_offset += slip_velocity * current_meas_period;
        current_control_.async_phase_offset = wrap_pm_pi(current_control_.async_phase_offset);
        phase += current_control_.async_phase_offset;
        phase = wrap_pm_pi(phase);
    }

    float pwm_phase = phase + 1.5f * current_meas_period * phase_vel;
    pwm_phase = wrap_pm_pi(pwm_phase);
    I_phase_ = pwm_phase;
    // Clarke transform


    bool res = true;
    // Execute current command
    switch(config_.motor_type){
        case MOTOR_TYPE_HIGH_CURRENT: res =  FOC_current(id, iq, phase, pwm_phase); break;
        case MOTOR_TYPE_ACIM: res = FOC_current(id, iq, phase, pwm_phase); break;
        case MOTOR_TYPE_GIMBAL: res =FOC_voltage(id, iq, pwm_phase); break;
        default: set_error(ERROR_NOT_IMPLEMENTED_MOTOR_TYPE); return false; break;
    }
    return res;
}


/**
 * @brief Called when the underlying hardware timer triggers an update event.
 */
void Motor::dc_calib_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current) {
    const float dc_calib_period = static_cast<float>(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;
    TaskTimerContext tmr{axis_->task_times_.dc_calib};

    if (current.has_value()) {
        const float calib_filter_k = std::min(dc_calib_period / config_.dc_calib_tau, 1.0f);
        DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
        DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
        DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
        dc_calib_running_since_ += dc_calib_period;
    } else {
        DC_calib_.phA = 0.0f;
        DC_calib_.phB = 0.0f;
        DC_calib_.phC = 0.0f;
        dc_calib_running_since_ = 0.0f;
    }
}

/**
 * @brief Called when the underlying hardware timer triggers an update event.
 */
void Motor::current_meas_cb(uint32_t timestamp, std::optional<Iph_ABC_t> current) {
    // TODO: this is platform specific
    //const float current_meas_period = static_cast<float>(2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1)) / TIM_1_8_CLOCK_HZ;
    TaskTimerContext tmr{axis_->task_times_.current_sense};

    n_evt_current_measurement_++;

    bool dc_calib_valid = (dc_calib_running_since_ >= config_.dc_calib_tau * 7.5f)
                       && (abs(DC_calib_.phA) < max_dc_calib_)
                       && (abs(DC_calib_.phB) < max_dc_calib_)
                       && (abs(DC_calib_.phC) < max_dc_calib_);

    if (armed_state_ == 1 || armed_state_ == 2) {
        current_meas_ = {0.0f, 0.0f, 0.0f};
        armed_state_ += 1;
    } else if (current.has_value() && dc_calib_valid) {
        current_meas_ = {
            current->phA - DC_calib_.phA,
            current->phB - DC_calib_.phB,
            current->phC - DC_calib_.phC
        };
    } else {
        current_meas_ = std::nullopt;
    }

    // Run system-level checks (e.g. overvoltage/undervoltage condition)
    // The motor might be disarmed in this function. In this case the
    // handler will continue to run until the end but it won't have an
    // effect on the PWM.
    odrv.do_fast_checks();

    if (current_meas_.has_value()) {
        // Check for violation of current limit
        // If Ia + Ib + Ic == 0 holds then we have:
        // Inorm^2 = Id^2 + Iq^2 = Ialpha^2 + Ibeta^2 = 2/3 * (Ia^2 + Ib^2 + Ic^2)
        float Itrip = effective_current_lim_ + config_.current_lim_margin;
        float Inorm_sq = 2.0f / 3.0f * (SQ(current_meas_->phA)
                                      + SQ(current_meas_->phB)
                                      + SQ(current_meas_->phC));

        // Hack: we disable the current check during motor calibration because
        // it tends to briefly overshoot when the motor moves to align flux with I_alpha
        if (Inorm_sq > SQ(Itrip)) {
            disarm_with_error(ERROR_CURRENT_LIMIT_VIOLATION);
        }
    } else if (is_armed_) {
        // Since we can't check current limits, be safe for now and disarm.
        // Theoretically we could continue to operate if there is no active
        // current limit.
        disarm_with_error(ERROR_UNKNOWN_CURRENT_MEASUREMENT);
    }

    if (control_law_) {
        Error err = control_law_->on_measurement(vbus_voltage,
                            current_meas_.has_value() ?
                                std::make_optional(std::array<float, 3>{current_meas_->phA, current_meas_->phB, current_meas_->phC})
                                : std::nullopt,
                            timestamp);
        if (err != ERROR_NONE) {
            disarm_with_error(err);
        }
    }
}



void Motor::pwm_update_cb(uint32_t output_timestamp) {
    TaskTimerContext tmr{axis_->task_times_.pwm_update};
    n_evt_pwm_update_++;

    Error control_law_status = ERROR_CONTROLLER_FAILED;
    float pwm_timings[3] = {NAN, NAN, NAN};
    std::optional<float> i_bus;

    if (control_law_) {
        control_law_status = control_law_->get_output(
            output_timestamp, pwm_timings, &i_bus);
    }

    // Apply control law to calculate PWM duty cycles
    if (is_armed_ && control_law_status == ERROR_NONE) {
        uint16_t next_timings[] = {
            (uint16_t)(pwm_timings[0] * (float)TIM_1_8_PERIOD_CLOCKS),
            (uint16_t)(pwm_timings[1] * (float)TIM_1_8_PERIOD_CLOCKS),
            (uint16_t)(pwm_timings[2] * (float)TIM_1_8_PERIOD_CLOCKS)
        };
        apply_pwm_timings(next_timings, false);
    } else if (is_armed_) {
        if (!(timer_->Instance->BDTR & TIM_BDTR_MOE) && (control_law_status == ERROR_CONTROLLER_INITIALIZING)) {
            // If the PWM output is armed in software but not yet in
            // hardware we tolerate the "initializing" error.
            i_bus = 0.0f;
        } else {
            set_error(control_law_status);
        }
    }

    if (!is_armed_) {
        // If something above failed, reset I_bus to 0A.
        i_bus = 0.0f;
    } else if (is_armed_ && !i_bus.has_value()) {
        // If the motor is armed then i_bus must be known
        set_error(ERROR_UNKNOWN_CURRENT_MEASUREMENT);
        i_bus = 0.0f;
    }

    I_bus_ = *i_bus;

    if (*i_bus < config_.I_bus_hard_min || *i_bus > config_.I_bus_hard_max) {
        set_error(ERROR_I_BUS_OUT_OF_RANGE);
    }


}