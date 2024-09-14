#include <odrive_main.h>
#include "can_encos.hpp"

typedef struct __attribute__((packed)) {
    uint8_t kp_h  :5;
    uint8_t mode  :3;
    uint8_t kd_h  :1;
    uint8_t kp_l  :7;
    uint8_t kd_l  :8;
    uint8_t pos_h :8;
    uint8_t pos_l :8;
    uint8_t vel_h :8;
    uint8_t tor_h :4;
    uint8_t vel_l :4;
    uint8_t tor_l :8;
} encos_cmd_pvt_t;

typedef struct __attribute__((packed)) {
    uint8_t ack_type :2;
    uint8_t          :3;
    uint8_t mode     :3;
} encos_curr_brake_cmd_t;

void encos_ack_type_1(Axis* &axis)
{
    Axis::axis_state_t state;
    can_Message_t txmsg;

    axis->get_axis_state(&state);

    txmsg.id = axis->config_.can_node_id;
    txmsg.isExt = axis->config_.can_node_id_extended;
    txmsg.len = 8;
    txmsg.buf[0] = state.erro & ((1 << 5) - 1);
    txmsg.buf[0] |= 1 << 5;
    txmsg.buf[1] = state.pos >> 8;
    txmsg.buf[2] = state.pos & 0xFF;
    txmsg.buf[3] = state.vel >> 4;
    txmsg.buf[4] = state.vel << 4;
    txmsg.buf[4] |= state.cur >> 8;
    txmsg.buf[5] = state.cur & 0xFF;
    txmsg.buf[6] = state.motor_temperature;
    txmsg.buf[7] = state.mos_temperature;

    odCAN->write(txmsg);
}

void encos_cmd_handle(Axis* &axis, can_Message_t& msg)
{
    if (axis->config_.can_node_id == msg.id) {
    switch (msg.len) {
        case 3:
            {
                encos_curr_brake_cmd_t *cmd = (encos_curr_brake_cmd_t *)&(msg.buf[0]);
                switch (cmd->mode) {
                case 3:// 手册描述能耗制动，但实际测试用来设置电流
                    axis->set_axis_current((msg.buf[1] << 8) + msg.buf[2]);
                    break;
                }
                
                switch (cmd->ack_type) {
                case 1:
                    encos_ack_type_1(axis);
                    break;
                }
            }
            break;
        case 8:
            switch (msg.buf[0] >> 5) {
            case 0:
                {
                    encos_cmd_pvt_t *cmd = (encos_cmd_pvt_t *)msg.buf;
                    Axis::axis_pvt_parm_t pvt_parm;
                    pvt_parm.kp = (cmd->kp_h << 7) + cmd->kp_l;
                    pvt_parm.kd = (cmd->kd_h << 8) + cmd->kd_l;
                    pvt_parm.pos_setpoint = (cmd->pos_h << 8) + cmd->pos_l;
                    pvt_parm.vel_setpoint = (cmd->vel_h << 4) + cmd->vel_l;
                    pvt_parm.torque_setpoint = (cmd->tor_h << 8) + cmd->tor_l;
                    axis->set_axis_pvt_parm(&pvt_parm);

                    encos_ack_type_1(axis);
                }
                break;
            }
            break;
        }
    } else if (0x7FF == msg.id) {
        switch (msg.len) {
        case 4:
            {
                if (0xFF == msg.buf[0] && 0xFF == msg.buf[1] && 0x00 == msg.buf[2] && 0x82 == msg.buf[3]) {
                    uint32_t id;
                    bool success = axis->get_nodeID(id);
                    can_Message_t txmsg;
                    txmsg.id = 0x7FF;
                    txmsg.isExt = axis->config_.can_node_id_extended;
                    if (success) {
                        txmsg.len = 5;
                        txmsg.buf[0] = 0xFF;
                        txmsg.buf[1] = 0xFF;
                        txmsg.buf[2] = 0x01;
                        txmsg.buf[3] = id >> 8;
                        txmsg.buf[4] = id & 0xFF;
                    } else {
                        txmsg.len = 4;
                        txmsg.buf[0] = 0x80;
                        txmsg.buf[1] = 0x80;
                        txmsg.buf[2] = 0x01;
                        txmsg.buf[3] = 0x80;
                    }
                    odCAN->write(txmsg);
                } else {
                    uint32_t id = (msg.buf[0] << 8) + msg.buf[1];
                    if (axis->config_.can_node_id == id) {
                        if (0 == msg.buf[2] && 3 == msg.buf[3]) {
                            // 设置零位
                            bool success = axis->set_offset();
                            if(success)
                            {
                                odrv.save_configuration();
                            }
                            
                            can_Message_t txmsg;
                            txmsg.id = 0x7FF;
                            txmsg.isExt = axis->config_.can_node_id_extended;
                            txmsg.len = 4;
                            txmsg.buf[0] = axis->config_.can_node_id >> 8;
                            txmsg.buf[1] = axis->config_.can_node_id & 0xFF;
                            txmsg.buf[2] = 0x01;
                            txmsg.buf[3] = success ? 3 : 0;
                            odCAN->write(txmsg);
                        }
                    }
                }
            }
            break;
        case 6:
            {
                if (0x7F == msg.buf[0] && 0x7F == msg.buf[1] && 0 == msg.buf[2] && 5 == msg.buf[3] && 0x7F == msg.buf[4] && 0x7F == msg.buf[5]) {
                    // 重置ID为1
                    if (axis->set_nodeID(1)) {
                        odrv.save_configuration();
                        can_Message_t txmsg;
                        txmsg.id = 0x7FF;
                        txmsg.isExt = axis->config_.can_node_id_extended;
                        txmsg.len = 6;
                        txmsg.buf[0] = 0x7F;
                        txmsg.buf[1] = 0x7F;
                        txmsg.buf[2] = 0x01;
                        txmsg.buf[3] = 0x05;
                        txmsg.buf[4] = 0x7F;
                        txmsg.buf[5] = 0x7F;
                        odCAN->write(txmsg);
                    }
                } else {
                uint32_t id = (msg.buf[0] << 8) + msg.buf[1];
                if (axis->config_.can_node_id == id) {
                    if (0 == msg.buf[2] && 4 == msg.buf[3]) {
                        // 设置ID
                        uint32_t new_id = (msg.buf[4] << 8) + msg.buf[5];
                        bool success = false;
                        if (new_id < 0x7FF && new_id > 0) {
                            success = axis->set_nodeID(new_id);
                            if(success)
                            {
                                odrv.save_configuration();
                            }
                            
                        }
                        can_Message_t txmsg;
                        txmsg.id = 0x7FF;
                        txmsg.isExt = axis->config_.can_node_id_extended;
                        txmsg.len = 4;
                        txmsg.buf[2] = 0x01;
                        if (success) {
                            txmsg.buf[0] = new_id >> 8;
                            txmsg.buf[1] = new_id & 0xFF;
                            txmsg.buf[3] = 4;
                        } else {
                            txmsg.buf[0] = id >> 8;
                            txmsg.buf[1] = id & 0xFF;
                            txmsg.buf[3] = 0;
                        }
                        odCAN->write(txmsg);
                    }
                    }
                }
            }
            break;
        }
    } else if (0x7FE == msg.id && msg.len >= 3) {
        uint32_t id = (msg.buf[0] << 8) + msg.buf[1];
        if (axis->config_.can_node_id == id) {
            uint8_t index = msg.buf[2];
            bool success = false;

            if (0xFF == index) {
                odrv.save_configuration();
                success = true;
            } else if (index < sizeof(axis->config_.ext_cfg) / sizeof(axis->config_.ext_cfg[0])) {
                if (7 == msg.len){
                    uint32_t value = (msg.buf[3] << 24) + (msg.buf[4] << 16) + (msg.buf[5] << 8) + msg.buf[6];
                    axis->config_.ext_cfg[index] = value;
                    success = true;
                    if(axis->ext_config_reg_callback_fun_[index])
                    {
                        axis->ext_config_reg_callback_fun_[index](axis,value);
                    }
                }
            }

            can_Message_t txmsg;
            txmsg.id = 0x7FE;
            txmsg.isExt = axis->config_.can_node_id_extended;
            txmsg.len = 4;
            txmsg.buf[0] = msg.buf[0];
            txmsg.buf[1] = msg.buf[1];
            txmsg.buf[2] = msg.buf[2];
            txmsg.buf[3] = success;
            odCAN->write(txmsg);
        }
    }
}

void CANEncos::handle_can_message(can_Message_t& msg)
{
    Axis* axis = nullptr;

    for (uint8_t i = 0; i < AXIS_COUNT_USED; i++) {
        if (axes[i]->config_.can_node_id_extended != msg.isExt)
            continue;

        if (0x7FF == msg.id) {
            encos_cmd_handle(axes[i], msg);
            continue;
        }

        if (axes[i]->config_.can_node_id == msg.id) {
            if (nullptr == axis) {
                axis = axes[i];
            } else {
                // Duplicate can IDs, don't assign to any axis
                odCAN->set_error(ODriveCAN::ERROR_DUPLICATE_CAN_IDS);
                return;
            }
        }
    }
    
    if (nullptr == axis)
        return;

    encos_cmd_handle(axis, msg);
    axis->watchdog_feed();
    if(axis->config_.can_node_id == msg.id)
    {
        axis->axis_enable_by_encos();
    }
        
}
