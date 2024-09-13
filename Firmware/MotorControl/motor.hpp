#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include "drv8301.h"

class Motor : public ODriveIntf::MotorIntf {
public:
    struct Iph_BC_t {
        float phA;
        float phB;
        float phC;
    };

    struct CurrentControl_t{
        float p_gain; // [V/A]
        float i_gain; // [V/As]
        float v_current_control_integral_d; // [V]
        float v_current_control_integral_q; // [V]
        float Ibus; // DC bus current [A]
        // Voltage applied at end of cycle:
        float final_v_alpha; // [V]
        float final_v_beta; // [V]

        float final_v_d; // [V]
        float final_v_q; // [V]

        float Id_setpoint; // [A]
        float Iq_setpoint; // [A]
        float Iq_measured; // [A]
        float Id_measured; // [A]
        float I_measured_report_filter_k;
        float max_allowed_current; // [A]
        float overcurrent_trip_level; // [A]
        float acim_rotor_flux; // [A]
        float async_phase_vel; // [rad/s electrical]
        float async_phase_offset; // [rad electrical]
    };
    static constexpr int32_t NUM_LINEARITY_SEG = 20;
    // NOTE: for gimbal motors, all units of Nm are instead V.
    // example: vel_gain is [V/(turn/s)] instead of [Nm/(turn/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t {
        bool pre_calibrated = false; // can be set to true to indicate that all values here are valid
        int32_t pole_pairs = 21;
        int32_t gear_ratio_ = 16;
        float motor_torque_base = 60.0f; // [Nm]
        float calibration_current = 10.0f;    // [A]
        float resistance_calib_max_voltage = 2.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        float phase_inductance = 0.000055f;        // to be set by measure_phase_inductance
        float phase_resistance = 0.07f;        // to be set by measure_phase_resistance
        float torque_constant = 0.087f;         // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor
        int32_t direction = 1;                // 1 or -1 (0 = unspecified)
        MotorType motor_type = MOTOR_TYPE_HIGH_CURRENT;
        // Read out max_allowed_current to see max supported value for current_lim.
        // float current_lim = 70.0f; //[A]
        float current_lim = 10.0f;          //[A]
        float current_lim_margin = 8.0f;    // Maximum violation of current_lim
        float torque_lim = std::numeric_limits<float>::infinity();           //[Nm]. 
        // Value used to compute shunt amplifier gains
        float requested_current_range = 60.0f; // [A]
        float current_control_bandwidth = 5000.0f;  // [rad/s]
        float inverter_temp_limit_lower = 100;
        float inverter_temp_limit_upper = 120;
        float acim_slip_velocity = 14.706f; // [rad/s electrical] = 1/rotor_tau
        float acim_gain_min_flux = 10; // [A]
        float acim_autoflux_min_Id = 10; // [A]
        bool acim_autoflux_enable = false;
        float acim_autoflux_attack_gain = 10.0f;
        float acim_autoflux_decay_gain = 1.0f;

        float Torque_LINEARITY_[NUM_LINEARITY_SEG];
        float CURRENT_LINEARITY_[NUM_LINEARITY_SEG];


        // custom property setters
        Motor* parent = nullptr;
        void set_pre_calibrated(bool value) {
            pre_calibrated = value;
            parent->is_calibrated_ = parent->is_calibrated_ || parent->config_.pre_calibrated;
        }
        void set_phase_inductance(float value) { phase_inductance = value; parent->update_current_controller_gains(); }
        void set_phase_resistance(float value) { phase_resistance = value; parent->update_current_controller_gains(); }
        void set_current_control_bandwidth(float value) { current_control_bandwidth = value; parent->update_current_controller_gains(); }
    };

    Motor(const MotorHardwareConfig_t& hw_config,
         const GateDriverHardwareConfig_t& gate_driver_config,
         Config_t& config);

    bool arm();
    void disarm();
    void setup() {
        int32_t index = 0;
        DRV8301_setup();
        for( index = 0;index < NUM_LINEARITY_SEG;index++)
        {
              L_Slop_Array_[index] = config_.Torque_LINEARITY_[index];
        }
        initNotchFilter(&notch_filter_, notch_filter_frequency_, notch_filter_sample_rate_, notch_filter_bandwidth_);


    }
    void reset_current_control();

    void update_current_controller_gains();
    void DRV8301_setup();
    bool check_DRV_fault();
    void set_error(Error error);
    bool do_checks();
    float effective_current_lim();
    float max_available_torque();
    void log_timing(TimingLog_t log_idx);
    float phase_current_from_adcval(uint32_t ADCValue, float phase_current_gain_coeff);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float voltage_low, float voltage_high);
    bool run_calibration();
    bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
    bool enqueue_voltage_timings(float v_alpha, float v_beta);
    bool FOC_voltage(float v_d, float v_q, float pwm_phase);
    bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase);
    bool update(float current_setpoint, float phase, float phase_vel);
    void pos_linearity_ini(void);
    float current_Correct(int32_t Torque_Org);
    void abc_sign_calculation(float phase , int32_t *a, int32_t *b, int32_t *c);
    const MotorHardwareConfig_t& hw_config_;
    const GateDriverHardwareConfig_t gate_driver_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

//private:

    DRV8301_Obj gate_driver_; // initialized in constructor
    uint16_t next_timings_[3] = {
        TIM_1_8_PERIOD_CLOCKS / 2,
        TIM_1_8_PERIOD_CLOCKS / 2,
        TIM_1_8_PERIOD_CLOCKS / 2
    };
    bool next_timings_valid_ = false;
    uint16_t last_cpu_time_ = 0;
    int timing_log_index_ = 0;
    struct {
        uint16_t& operator[](size_t idx) { return content[idx]; }
        uint16_t& get(size_t idx) { return content[idx]; }
        uint16_t content[TIMING_LOG_NUM_SLOTS];
    } timing_log_;

    // variables exposed on protocol
    Error error_ = ERROR_NONE;
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    ArmedState armed_state_ = ARMED_STATE_DISARMED; 
    bool is_calibrated_ = config_.pre_calibrated;
    Iph_BC_t current_meas_ = {0.0f,0.0f, 0.0f};
    Iph_BC_t DC_calib_ = {0.0f,0.0f, 0.0f};
    float phase_current_rev_gain_ = 0.0f; // Reverse gain for ADC to Amps (to be set by DRV8301_setup)
    CurrentControl_t current_control_ = {
        .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
        .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
        .v_current_control_integral_d = 0.0f,
        .v_current_control_integral_q = 0.0f,
        .Ibus = 0.0f,
        .final_v_alpha = 0.0f,
        .final_v_beta = 0.0f,
        .final_v_d = 0.0f,
        .final_v_q = 0.0f,
        .Id_setpoint = 0.0f,
        .Iq_setpoint = 0.0f,
        .Iq_measured = 0.0f,
        .Id_measured = 0.0f,
        .I_measured_report_filter_k = 1.0f,
        .max_allowed_current = 0.0f,
        .overcurrent_trip_level = 0.0f,
        .acim_rotor_flux = 0.0f,
        .async_phase_vel = 0.0f,
        .async_phase_offset = 0.0f,
    };
    struct : GateDriverIntf {
        DrvFault drv_fault = DRV_FAULT_NO_FAULT;
    } gate_driver_exported_;
    DRV_SPI_8301_Vars_t gate_driver_regs_; //Local view of DRV registers (initialized by DRV8301_setup)
    float effective_current_lim_ = 10.0f;
    bool capturing_ = false;
    int32_t oscilloscope_div = 0;
    float  m_speed_est_fast =0;
    float Iq_filter = 0;
    float Id_filter = 0;
    float Iq_filter2 = 0;
    float Id_filter2 = 0;
    float Idq_filter_k_ = 0.4f;
    float Idq_filter_k2_ = 0.01f;
    bool using_old_torque_constant_ = true;
    float L_Slop_Array_[NUM_LINEARITY_SEG] = {1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f,1.f};
    int32_t sign_a_, sign_b_, sign_c_;
    float total_phase_for_abc_sign_calculation_;
    float I_phase_;
    float I_phase2_;
    float deadtime_compensation_coff_ = 0.0f;
    int16_t Aphase_deadtime_compensation_ = 0;
    int16_t Bphase_deadtime_compensation_ = 0;
    int16_t Cphase_deadtime_compensation_ = 0;
    float torque_setpoint_filterd_ = 0;
    float torque_setpoint_notch_filterd_ = 0;

    float notch_filter_sample_rate_ = 20000.f;
    float notch_filter_frequency_ = 50.f;
    float notch_filter_bandwidth_ = 10.f;  

    NotchFilter notch_filter_;

    void setting_motor_current_linearity(uint32_t index, float value);
    void setting_motor_torque_linearity(uint32_t index, float value);
    float get_motor_current_linearity(uint32_t index);
    float get_motor_torque_linearity(uint32_t index);
    void setting_torque_slope(uint32_t index, float value);
    float get_torque_slope(uint32_t index);
    void pos_linearity_init(void);
};

#endif // __MOTOR_HPP
