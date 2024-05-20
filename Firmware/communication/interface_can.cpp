#include "interface_can.hpp"

#include "fibre/crc.hpp"
#include "freertos_vars.h"
#include "utils.hpp"

#include <can.h>
#include <cmsis_os.h>
#include <stm32f4xx_hal.h>

// Specific CAN Protocols
#include "can_simple.hpp"

int32_t  epos_Controlword(enum Epos_ctrl ctrl);
int32_t epos_Modes_of_Operation( enum Epos_mode mode);
int32_t epos_Target_velocity( int32_t enc);
int32_t epos_Target_position( int32_t enc);
int32_t epos_read_SDO( int32_t word);
// Safer context handling via maps instead of arrays
// #include <unordered_map>
// std::unordered_map<CAN_HandleTypeDef *, ODriveCAN *> ctxMap;

// Constructor is called by communication.cpp and the handle is assigned appropriately
ODriveCAN::ODriveCAN(ODriveCAN::Config_t &config, CAN_HandleTypeDef *handle)
    : config_{config},
      handle_{handle} {
    // ctxMap[handle_] = this;
}

void ODriveCAN::can_server_thread() {

    epos_Controlword(CW_SWITCH_ON_DISABLED);
    vTaskDelay(100);
    epos_Controlword(CW_DISABLE_VOLTAGE);
    vTaskDelay(100);
    epos_Modes_of_Operation(operaton_mode);
    vTaskDelay(100);
    epos_Modes_of_Operation(operaton_mode);
    vTaskDelay(100);
    epos_read_SDO(MOTOR_ACTUAL_POSITION_VALUE_WORD);
    osSemaphoreWait(sem_can, 0);
    osSemaphoreWait(sem_can, 0);
    {
        can_Message_t rxmsg;
        osSemaphoreWait(sem_can, 10);  // Poll every 10ms regardless of sempahore status
        while (available()) {
            read(rxmsg);
            CANSimple::handle_can_message(rxmsg);
        }
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    epos_Target_position(actual_position);
    vTaskDelay(100);
    epos_Controlword(CW_SHUTDOWN);
    vTaskDelay(100);
    epos_Controlword(CW_SWITCH_ON);
    vTaskDelay(100);
    epos_Controlword(CW_OPERATION_ENABLED);
    vTaskDelay(3000);
    send_task_ready = true;
    

  
    for (;;) {
        uint32_t status = HAL_CAN_GetError(handle_);
        if (status == HAL_CAN_ERROR_NONE) {
            can_Message_t rxmsg;

            osSemaphoreWait(sem_can, 10);  // Poll every 10ms regardless of sempahore status
            while (available()) {
                read(rxmsg);
                switch (config_.protocol) {
                    case PROTOCOL_SIMPLE:
                        CANSimple::handle_can_message(rxmsg);
                        break;
                }
            }
            HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
        } else {
            if (status == HAL_CAN_ERROR_TIMEOUT) {
                HAL_CAN_ResetError(handle_);
                status = HAL_CAN_Start(handle_);
                if (status == HAL_OK)
                    status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
            }
        }
    }
}

static void can_server_thread_wrapper(void *ctx) {
    reinterpret_cast<ODriveCAN *>(ctx)->can_server_thread();
    reinterpret_cast<ODriveCAN *>(ctx)->thread_id_valid_ = false;
}

bool ODriveCAN::start_can_server() {
    HAL_StatusTypeDef status;


    set_baud_rate(config_.baud_rate);

    status = HAL_CAN_Init(handle_);

    CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    status = HAL_CAN_ConfigFilter(handle_, &filter);

    status = HAL_CAN_Start(handle_);
    if (status == HAL_OK)
        status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);

    init_postion_can_cmd_timer_list(can_cmd_timer_list,CAN_CMD_NOBR);

    operaton_mode = Cycle_Synchronous_Position;

    osThreadDef(can_server_thread_def, can_server_thread_wrapper, osPriorityNormal, 0, stack_size_ / sizeof(StackType_t));
    thread_id_ = osThreadCreate(osThread(can_server_thread_def), this);
    thread_id_valid_ = true;

    return status;
}

// Send a CAN message on the bus
uint32_t ODriveCAN::write(can_Message_t &txmsg) {
    if (HAL_CAN_GetError(handle_) == HAL_CAN_ERROR_NONE) {
        CAN_TxHeaderTypeDef header;
        header.StdId = txmsg.id;
        header.ExtId = txmsg.id;
        header.IDE = txmsg.isExt ? CAN_ID_EXT : CAN_ID_STD;
        header.RTR = CAN_RTR_DATA;
        header.DLC = txmsg.len;
        header.TransmitGlobalTime = FunctionalState::DISABLE;

        uint32_t retTxMailbox = 0;
        if (HAL_CAN_GetTxMailboxesFreeLevel(handle_) > 0)
            HAL_CAN_AddTxMessage(handle_, &header, txmsg.buf, &retTxMailbox);

        return retTxMailbox;
    } else {
        return -1;
    }
}

uint32_t ODriveCAN::available() {
    return (HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO0) + HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO1));
}

bool ODriveCAN::read(can_Message_t &rxmsg) {
    CAN_RxHeaderTypeDef header;
    bool validRead = false;
    if (HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO0) > 0) {
        HAL_CAN_GetRxMessage(handle_, CAN_RX_FIFO0, &header, rxmsg.buf);
        validRead = true;
    } else if (HAL_CAN_GetRxFifoFillLevel(handle_, CAN_RX_FIFO1) > 0) {
        HAL_CAN_GetRxMessage(handle_, CAN_RX_FIFO1, &header, rxmsg.buf);
        validRead = true;
    }

    rxmsg.isExt = header.IDE;
    rxmsg.id = rxmsg.isExt ? header.ExtId : header.StdId;  // If it's an extended message, pass the extended ID
    rxmsg.len = header.DLC;
    rxmsg.rtr = header.RTR;

    return validRead;
}

// Set one of only a few common baud rates.  CAN doesn't do arbitrary baud rates well due to the time-quanta issue.
// 21 TQ allows for easy sampling at exactly 80% (recommended by Vector Informatik GmbH for high reliability systems)
// Conveniently, the CAN peripheral's 42MHz clock lets us easily create 21TQs for all common baud rates
void ODriveCAN::set_baud_rate(uint32_t baudRate) {
    switch (baudRate) {
        case CAN_BAUD_125K:
            handle_->Init.Prescaler = 16;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        case CAN_BAUD_250K:
            handle_->Init.Prescaler = 8;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        case CAN_BAUD_500K:
            handle_->Init.Prescaler = 4;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        case CAN_BAUD_1000K:
            handle_->Init.Prescaler = 2;  // 21 TQ's
            config_.baud_rate = baudRate;
            reinit_can();
            break;

        default:
            // baudRate is invalid, so don't accept it.
            break;
    }
}

void ODriveCAN::reinit_can() {
    HAL_CAN_Stop(handle_);
    HAL_CAN_Init(handle_);
    auto status = HAL_CAN_Start(handle_);
    if (status == HAL_OK)
        status = HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void ODriveCAN::set_error(Error error) {
    error_ |= error;
}
// This function is called by each axis.
// It provides an abstraction from the specific CAN protocol in use
void ODriveCAN::send_heartbeat(Axis *axis) {
    // Handle heartbeat message
    if (axis->config_.can_heartbeat_rate_ms > 0) {
        uint32_t now = osKernelSysTick();
        if ((now - axis->last_heartbeat_) >= axis->config_.can_heartbeat_rate_ms) {
            switch (config_.protocol) {
                case PROTOCOL_SIMPLE:
                    CANSimple::send_heartbeat(axis);
                    break;
            }
            axis->last_heartbeat_ = now;
        }
    }
}


#define SAMPLE_FRE 500

struct Sin_t{
    uint32_t sample_rate;
    uint32_t output_fre;
    uint32_t index;
    bool is_sin;
};
static struct Sin_t sin_rule = {

    .sample_rate = SAMPLE_FRE,
    .output_fre = 35,
    .index = 0,
    .is_sin = true,
};


static void generate_sin_data_to_buf(uint8_t *buf)
{

    uint32_t sample_rate = sin_rule.sample_rate ;
    uint32_t output_fre = sin_rule.output_fre;
    float dt = 1.0f / sample_rate;
    float amplitude =0x00000100 ;

    float phase = 2.0f*M_PI* sin_rule.index * output_fre*dt ;
    phase = wrap_pm_pi(phase);

    float  sin_data;

    if( sin_rule.is_sin )
    {
        sin_data  =   our_arm_sin_f32(phase) * amplitude  ;
    }
    else
    {
        sin_data  = ( (phase < 0) ?1:-1 ) * amplitude  ;
    }
   


    int32_t sin_data_int = static_cast<int>(sin_data);


    buf[4] = sin_data_int & 0xff;
    buf[5] = (sin_data_int>>8) & 0xff;
    buf[6] = (sin_data_int>>16) & 0xff;
    buf[7] = (sin_data_int>>24) & 0xff;


    sin_rule.index++;
    if( sin_rule.index > sample_rate/output_fre -1)
    {
        sin_rule.index =0;
    }
}



void ODriveCAN:: init_can_cmd_timer_list(struct Cia402_Cmd_TimerList * cmd_timer_list, uint32_t size,enum Epos_word motor_control_word)
{
    uint32_t cmd_timer_index = 0;

    cmd_timer_list[cmd_timer_index].enable = true;
    cmd_timer_list[cmd_timer_index].interval_time = 20*1000/SAMPLE_FRE; // 200:10ms  20:1ms
    cmd_timer_list[cmd_timer_index].timer_cnt =0;
    cmd_timer_list[cmd_timer_index].txmsg.id = 0x601;
    cmd_timer_list[cmd_timer_index].txmsg.isExt  =0;
    cmd_timer_list[cmd_timer_index].txmsg.len = 8;
    cmd_timer_list[cmd_timer_index].txmsg.buf[0]=0x2b;
    cmd_timer_list[cmd_timer_index].txmsg.buf[1]=motor_control_word & 0xff;
    cmd_timer_list[cmd_timer_index].txmsg.buf[2]=(motor_control_word>>8) & 0xff;
    cmd_timer_list[cmd_timer_index].txmsg.buf[3]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[4]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[5]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[6]=0x02;
    cmd_timer_list[cmd_timer_index].txmsg.buf[7]=0x00;
    cmd_timer_list[cmd_timer_index].init_buf_f = generate_sin_data_to_buf;

    cmd_timer_index++;

    cmd_timer_list[cmd_timer_index].enable = false;
    cmd_timer_list[cmd_timer_index].interval_time = 20;
    cmd_timer_list[cmd_timer_index].timer_cnt =0;
    cmd_timer_list[cmd_timer_index].txmsg.id = 0x601;
    cmd_timer_list[cmd_timer_index].txmsg.isExt  =0;
    cmd_timer_list[cmd_timer_index].txmsg.len = 8;
    cmd_timer_list[cmd_timer_index].txmsg.buf[0]=0x40;
    cmd_timer_list[cmd_timer_index].txmsg.buf[1]=0x69;
    cmd_timer_list[cmd_timer_index].txmsg.buf[2]=0x60;
    cmd_timer_list[cmd_timer_index].txmsg.buf[3]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[4]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[5]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[6]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[7]=0x00;
    cmd_timer_list[cmd_timer_index].init_buf_f = nullptr;

    cmd_timer_index++;

    cmd_timer_list[cmd_timer_index].enable = false;
    cmd_timer_list[cmd_timer_index].interval_time = 10000;
    cmd_timer_list[cmd_timer_index].timer_cnt =0;
    cmd_timer_list[cmd_timer_index].txmsg.id = 0x601;
    cmd_timer_list[cmd_timer_index].txmsg.isExt  =0;
    cmd_timer_list[cmd_timer_index].txmsg.len = 8;
    cmd_timer_list[cmd_timer_index].txmsg.buf[0]=0x23;
    cmd_timer_list[cmd_timer_index].init_buf_f = nullptr;

    cmd_timer_index++;

    if(cmd_timer_index > size)
    {
        while(1);
    }
}

void ODriveCAN:: init_postion_can_cmd_timer_list(struct Cia402_Cmd_TimerList * cmd_timer_list, uint32_t size)
{
    uint32_t cmd_timer_index = 0;

    cmd_timer_list[cmd_timer_index].enable = true;
    cmd_timer_list[cmd_timer_index].interval_time = 20*1000/SAMPLE_FRE;;  // 200:10ms  20:1ms
    cmd_timer_list[cmd_timer_index].timer_cnt =0;
    cmd_timer_list[cmd_timer_index].txmsg.id = 0x601;
    cmd_timer_list[cmd_timer_index].txmsg.isExt  =0;
    cmd_timer_list[cmd_timer_index].txmsg.len = 8;
    cmd_timer_list[cmd_timer_index].txmsg.buf[0]=0x23;
    cmd_timer_list[cmd_timer_index].txmsg.buf[1] = MOTOR_TARGET_POSITION_WORD & 0xff;
    cmd_timer_list[cmd_timer_index].txmsg.buf[2] = (MOTOR_TARGET_POSITION_WORD>>8) & 0xff;
    cmd_timer_list[cmd_timer_index].txmsg.buf[3]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[4]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[5]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[6]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[7]=0x00;
    cmd_timer_list[cmd_timer_index].init_buf_f = generate_sin_data_to_buf;

    cmd_timer_index++;

    cmd_timer_list[cmd_timer_index].enable = false;
    cmd_timer_list[cmd_timer_index].interval_time = 20;
    cmd_timer_list[cmd_timer_index].timer_cnt =0;
    cmd_timer_list[cmd_timer_index].txmsg.id = 0x601;
    cmd_timer_list[cmd_timer_index].txmsg.isExt  =0;
    cmd_timer_list[cmd_timer_index].txmsg.len = 8;
    cmd_timer_list[cmd_timer_index].txmsg.buf[0]=0x40;
    cmd_timer_list[cmd_timer_index].txmsg.buf[1]=0x69;
    cmd_timer_list[cmd_timer_index].txmsg.buf[2]=0x60;
    cmd_timer_list[cmd_timer_index].txmsg.buf[3]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[4]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[5]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[6]=0x00;
    cmd_timer_list[cmd_timer_index].txmsg.buf[7]=0x00;
    cmd_timer_list[cmd_timer_index].init_buf_f = nullptr;

    cmd_timer_index++;

    cmd_timer_list[cmd_timer_index].enable = false;
    cmd_timer_list[cmd_timer_index].interval_time = 10000;
    cmd_timer_list[cmd_timer_index].timer_cnt =0;
    cmd_timer_list[cmd_timer_index].txmsg.id = 0x601;
    cmd_timer_list[cmd_timer_index].txmsg.isExt  =0;
    cmd_timer_list[cmd_timer_index].txmsg.len = 8;
    cmd_timer_list[cmd_timer_index].txmsg.buf[0]=0x23;
    cmd_timer_list[cmd_timer_index].init_buf_f = nullptr;

    cmd_timer_index++;

    if(cmd_timer_index > size)
    {
        while(1);
    }
}



int32_t ODriveCAN::cia_402_send_task(Axis *axis) {

    can_Message_t txmsg;
    int32_t status = 0;
    int8_t timer_item = 0;
    uint32_t high_freq_tick  = axis->loop_counter_;

    uint32_t timer_over = 0;

    if(send_task_ready)
    {
        for ( timer_item = 0 ; timer_item < CAN_CMD_NOBR; timer_item++ )    
        {
            if(can_cmd_timer_list[timer_item].enable == true )
            {
                timer_over = high_freq_tick - can_cmd_timer_list[timer_item].timer_cnt;
                uint32_t last_transfer_elapsetime = high_freq_tick - last_transfer_stamp; //最小时间mak
                if( (timer_over > 0 && timer_over < 0x7fffffff) && (last_transfer_elapsetime > 10 && last_transfer_elapsetime < 0x7fffffff) )
                {
                    can_cmd_timer_list[timer_item].timer_cnt  = high_freq_tick + can_cmd_timer_list[timer_item].interval_time;
                    if( can_cmd_timer_list[timer_item].init_buf_f)
                    {
                        can_cmd_timer_list[timer_item].init_buf_f(can_cmd_timer_list[timer_item].txmsg.buf);
                    }
                    txmsg =  can_cmd_timer_list[timer_item].txmsg;
                    status = CANSimple::cia_402_send_callback(axis,txmsg);
                    last_transfer_stamp = high_freq_tick;
                    break;
                }
            }

        }
    }


    return status;
      
}

int32_t  epos_Controlword(enum Epos_ctrl ctrl)
 {
	can_Message_t txmsg;
    int32_t status = 0;

    txmsg.id = 0x601;
	txmsg.isExt = false;
    txmsg.rtr = false;
    txmsg.len = 8;

    txmsg.buf[0] = 0x2b;
    txmsg.buf[1] = 0x40;
    txmsg.buf[2] = 0x60;
    txmsg.buf[3] = 0x00;
    txmsg.buf[4] = ctrl & 0xff;
    txmsg.buf[5] = (ctrl>>8) & 0xff;
    txmsg.buf[6] = (ctrl>>16) & 0xff;
    txmsg.buf[7] = (ctrl>>24) & 0xff;


	status = CANSimple::cia_402_send_callback(nullptr,txmsg);
    return status;
}

int32_t epos_Modes_of_Operation( enum Epos_mode mode) 
{
	can_Message_t txmsg;
    int32_t status = 0;

    txmsg.id = 0x601;
	txmsg.isExt = false;
    txmsg.rtr = false;
    txmsg.len = 8;

    txmsg.buf[0] = 0x2b;
    txmsg.buf[1] = 0x60;
    txmsg.buf[2] = 0x60;
    txmsg.buf[3] = 0x00;
    txmsg.buf[4] = mode & 0xff;
    txmsg.buf[5] = (mode>>8) & 0xff;
    txmsg.buf[6] = (mode>>16) & 0xff;
    txmsg.buf[7] = (mode>>24) & 0xff;


	status = CANSimple::cia_402_send_callback(nullptr,txmsg);
    return status;
}

int32_t epos_Target_velocity( int32_t enc) 
{
	can_Message_t txmsg;
    int32_t status = 0;

    txmsg.id = 0x601;
	txmsg.isExt = false;
    txmsg.rtr = false;
    txmsg.len = 8;

    txmsg.buf[0] = 0x23;
    txmsg.buf[1] = 0xff;
    txmsg.buf[2] = 0x60;
    txmsg.buf[3] = 0x00;
    txmsg.buf[4] = enc & 0xff;
    txmsg.buf[5] = (enc>>8) & 0xff;
    txmsg.buf[6] = (enc>>16) & 0xff;
    txmsg.buf[7] = (enc>>24) & 0xff;

	status = CANSimple::cia_402_send_callback(nullptr,txmsg);
    return status;
}


int32_t epos_Target_position( int32_t enc) 
{
	can_Message_t txmsg;
    int32_t status = 0;

    txmsg.id = 0x601;
	txmsg.isExt = false;
    txmsg.rtr = false;
    txmsg.len = 8;

    txmsg.buf[0] = 0x23;
    txmsg.buf[1] = MOTOR_TARGET_POSITION_WORD & 0xff;
    txmsg.buf[2] = (MOTOR_TARGET_POSITION_WORD>>8) & 0xff;
    txmsg.buf[3] = 0x00;
    txmsg.buf[4] = enc & 0xff;
    txmsg.buf[5] = (enc>>8) & 0xff;
    txmsg.buf[6] = (enc>>16) & 0xff;
    txmsg.buf[7] = (enc>>24) & 0xff;

	status = CANSimple::cia_402_send_callback(nullptr,txmsg);
    return status;
}


int32_t epos_read_SDO( int32_t word) 
{
	can_Message_t txmsg;
    int32_t status = 0;

    txmsg.id = 0x601;
	txmsg.isExt = false;
    txmsg.rtr = false;
    txmsg.len = 8;

    txmsg.buf[0] = 0x40;
    txmsg.buf[1] = word & 0xff;
    txmsg.buf[2] = (word>>8) & 0xff;
    txmsg.buf[3] = 0x00;
    txmsg.buf[4] = 0x00;
    txmsg.buf[5] = 0x00;
    txmsg.buf[6] = 0x00;
    txmsg.buf[7] = 0x00;

	status = CANSimple::cia_402_send_callback(nullptr,txmsg);
    return status;
}



void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
    // osSemaphoreRelease(sem_can);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan) {}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_ResetError(hcan);
}
