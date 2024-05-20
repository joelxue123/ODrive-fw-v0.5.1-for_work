#ifndef __INTERFACE_CAN_HPP
#define __INTERFACE_CAN_HPP

#include <cmsis_os.h>
#include <stm32f4xx_hal.h>
#include "fibre/protocol.hpp"
#include "odrive_main.h"
#include "can_helpers.hpp"

#define CAN_CLK_HZ (42000000)
#define CAN_CLK_MHZ (42)

#define CAN_CMD_NOBR 3

struct Cia402_Cmd_TimerList 
{
    bool enable;
    uint32_t timer_cnt;
    uint32_t interval_time;
    can_Message_t txmsg;
    void (*init_buf_f)( uint8_t *buf);
};

enum Epos_word {
    MOTOR_CONTROL_WORD = 0x6040,
    MOTOR_STATUS_WORD = 0x6041,
    MOTOR_OPERATION_MODE = 0x6060,
    MOTOR_ACTUAL_POSITION_VALUE_WORD = 0X6064,
    MOTOR_TARGET_CURRENT_WORD = 0x6071,
    MOTOR_TARGET_POSITION_WORD = 0x607A,
    MOTOR_TARGET_VELOCITY_WORD = 0x60ff,
};
enum Epos_ctrl {

    CW_OPERATION_ENABLED = 0x000F,
    CW_SHUTDOWN = 0x0006,
    CW_SWITCH_ON = 0x0007,
    CW_QUICK_STOP = 0x0002,
    CW_DISABLE_VOLTAGE = 0x0000,
    CW_SWITCH_ON_DISABLED = 0x0080,
} ;

enum Epos_mode {
    
	Interpolated_Position_Mode = 7,
	Homing_Mode = 6,
	Profile_Velocity_Mode = 3,
	Profile_Position_Mode = 1,
	Position_Mode = -1,
	Velocity_Mode = -2,
	Current_Mode = -3,
	Diagnostic_Mode = -4,
	Master_Encoder_Mode = -5,
	Step_Direction_Mode = -6,
    Cycle_Synchronous_Position =0x08,
    Cycle_Synchronous_Velocity =0x09,
    Cycle_Synchronous_Current =0x0a,
};



// Anonymous enum for defining the most common CAN baud rates
enum {
    CAN_BAUD_125K   = 125000,
    CAN_BAUD_250K   = 250000,
    CAN_BAUD_500K   = 500000,
    CAN_BAUD_1000K  = 1000000,
    CAN_BAUD_1M     = 1000000
};

class ODriveCAN : public ODriveIntf::CanIntf {
   public:
    struct Config_t {
        uint32_t baud_rate = CAN_BAUD_1000K;
        Protocol protocol = PROTOCOL_SIMPLE;
    };



    ODriveCAN(ODriveCAN::Config_t &config, CAN_HandleTypeDef *handle);
    struct Cia402_Cmd_TimerList can_cmd_timer_list[CAN_CMD_NOBR];
    // Thread Relevant Data
    osThreadId thread_id_;
    const uint32_t stack_size_ = 1024; // Bytes
    Error error_ = ERROR_NONE;

    volatile bool thread_id_valid_ = false;
    enum Epos_mode  operaton_mode;


    bool start_can_server();
    void can_server_thread();
    void send_heartbeat(Axis *axis);
    void reinit_can();
    int32_t cia_402_send_task(Axis *axis);
    void set_error(Error error);

    void init_postion_can_cmd_timer_list(struct Cia402_Cmd_TimerList * cmd_timer_list, uint32_t size);
    void init_can_cmd_timer_list(struct Cia402_Cmd_TimerList * cmd_timer_list, uint32_t size,enum Epos_word motor_control_word);
    // I/O Functions
    uint32_t available();
    uint32_t write(can_Message_t &txmsg);
    bool read(can_Message_t &rxmsg);

    ODriveCAN::Config_t &config_;

    bool send_task_ready = false;

    const uint32_t atleast_send_interval = 20;
    uint32_t last_transfer_stamp =0;

    volatile int32_t actual_position =0;
private:
    CAN_HandleTypeDef *handle_ = nullptr;

    void set_baud_rate(uint32_t baudRate);


};

#endif  // __INTERFACE_CAN_HPP
