#include <ODriveCan.h>


ODriveCanMtr::ODriveCanMtr(int (*send_func) (uint16_t can_id, uint8_t len, uint8_t* data), uint8_t node_id) 
{
    _node_id = node_id;
    can_send_msg = send_func;
    can_read_msg_buffer = can_read;
}

void ODriveCanMtr::begin()
{
    if (_node_id == 0x3f) {
        Serial.println("[ODriveCan - Warning] 0x3f is the broadcast node ID!");
    }
}


/*
    Use when the main code is reading the can messages and passing them to the correct motor

    All can messages use intel (little-endian), i.e. if bytes 0-3 are the data for an unsigned 32 bit integer,
    byte 3 is the most significant byte and 1 is the least significant byte.
*/
int ODriveCanMtr::process_cmd(uint8_t cmd, uint8_t len, uint8_t* data)
{
    int error_state = success;

    switch (cmd) {
        case get_version:
            //Bin, I don't care
            break;
        case heartbeat:
            _mtr_last_hb = millis();
            if (len < 5) {
                //Err
                return invalid_data_length;
            }
            last_mtr_values.axis_error = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.axis_state = data[4];
            //Dont really care about the rest.
            break;
        case get_error:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            last_mtr_values.active_errors = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.disarm_reason = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            break;
        case tx_sdo:
            //Dont care
            break;
        case address:
            //Dont care
            break;
        case get_encoder_estimates:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.pos_estimate = flt_cnv.flt;

            flt_cnv.u32 = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.vel_estimate = flt_cnv.flt;
            break;
        case get_iq:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.iq_setpoint = flt_cnv.flt;

            flt_cnv.u32 = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.iq_measured = flt_cnv.flt;
            break;
        case get_temp:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.fet_temp = flt_cnv.flt;

            flt_cnv.u32 = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.motor_temp = flt_cnv.flt;
            break;
        case get_bus_voltage_current:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.bus_voltage = flt_cnv.flt;

            flt_cnv.u32 = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.bus_current = flt_cnv.flt;
            break;
        case get_torques:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.torque_tar = flt_cnv.flt;

            flt_cnv.u32 = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.torque_estimate = flt_cnv.flt;
            break;
        case get_powers:
            if (len < 8) {
                //err
                return invalid_data_length;
            }
            flt_cnv.u32 = (data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0]);
            last_mtr_values.elec_power = flt_cnv.flt;

            flt_cnv.u32 = (data[7] << 24) + (data[6] << 16) + (data[5] << 8) + (data[4]);
            last_mtr_values.mech_power = flt_cnv.flt;
            break;
        default:
            error_state = not_valid_cmd;
            break;
    }

    return error_state;
}

/*
    Use when only the motors on the bus
    Pass the full can id into the passing
*/
int ODriveCanMtr::process_msg(uint16_t can_id, uint8_t len, uint8_t* data)
{
    //Left most 6 bits are the node id
    if ((can_id >> 5) != _node_id) {
        return not_this_node;
    }

    //Rest (5 bits) are the command ID
    uint8_t cmd = (can_id & 0xFFFFF);
    return process_cmd(cmd, len, data);
}

int ODriveCanMtr::req_info_cmd(cmd_id cmd)
{
    switch (cmd) {
        case get_version:
        case heartbeat:
        case get_error:
        case tx_sdo:
        case address:
        case get_encoder_estimates:
        case get_iq:
        case get_temp:
        case get_bus_voltage_current:
        case get_torques:
        case get_powers:
            can_send_msg((_node_id << 5) + ((uint8_t) cmd), 0, 0);
            break;
        default:
            //Not one of the right IDs
            Serial.println("[ODrive Can] Not an info cmd");
            return not_info_cmd;
            break;

    }
}

/*
    Set motor values
*/
set_axis_state(uint32_t state)
{
    uint8_t data[4];
    data[3] = (state >> 24);
    data[2] = 
}

set_cont_mode(uint32 cont_mode, uint32_t ip_mode)
{

}

set_ip_pos(float ip_pos, int16_t vel, int16_t torque)
{

}

set_input_vel(float vel, float torque)
{

}

set_input_torq(float torque)
{

}

set_lim(float vel_lim, float cur_lim)
{

}

set_traj_vel_limit(float vel_lim)
{

}

set_traj_accel_limits(float accel_limit, float decel_limit)
{

}

set_traj_inertia(float interia)
{

}

reboot_mtr()
{

}

clear_errors(uint8_t error)
{

}

set_absolute_position(float pos)
{

}

set_position_gain(float pos_gain)
{

}

set_velocity_gains(float vel_gain, float vel_integ_gain)
{

}

uint8_t ODriveCanMtr::node_id()
{
    return _node_id;
}

bool ODriveCanMtr::mtr_connected()
{
    return (millis() - _mtr_last_hb > _timeout);
}