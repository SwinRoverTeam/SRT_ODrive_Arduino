#ifndef ODriveCan
#define ODriveCan

#include <Arduino.h>

union float_conv {
    float flt;
    uint32_t u32;
};

enum cmd_id {
    get_version             = 0x000,
    heartbeat               = 0x001,
    estop                   = 0x002,
    get_error               = 0x003,
    rx_sdo                  = 0x004,
    tx_sdo                  = 0x005,
    address                 = 0x006,
    set_axis_state          = 0x007,
    get_encoder_estimates   = 0x009,
    set_controller_mode     = 0x00b,
    set_input_pos           = 0x00c,
    set_input_vel           = 0x00d,
    set_input_torque        = 0x00e,
    set_limit               = 0x00f,
    set_traj_vel_limit      = 0x011,
    set_traj_accel_limits   = 0x012,
    set_traj_intertia       = 0x013,
    get_iq                  = 0x014,
    get_temp                = 0x015,
    reboot                  = 0x016,
    get_bus_voltage_current = 0x017,
    clear_errors            = 0x018,
    set_abs_position        = 0x019,
    set_pos_gain            = 0x01a,
    set_vel_gains           = 0x01b,
    get_torques             = 0x01c,
    get_powers              = 0x1d,
    enter_dfu_mode          = 0x1f
};

enum proccess_return {
    not_this_node = -0x1,
    success = 0x00,
    can_bus_fault = 0x01,
    can_bus_no_response = 0x02,
    not_info_cmd = 0x03,
    not_valid_cmd = 0x04,
    invalid_data_length = 0x05
};

struct mtr_values {
    uint32_t axis_error;
    uint8_t axis_state;
    uint32_t active_errors;
    uint32_t disarm_reason;
    float pos_estimate;
    float vel_estimate;
    float iq_setpoint;
    float iq_measured;
    float fet_temp;
    float motor_temp;
    float bus_voltage;
    float bus_current;
    float torque_tar;
    float torque_estimate;
    float elec_power;
    float mech_power;
};


class ODriveCanMtr
{
    private:
        uint8_t _node_id; //Be careful 0x3f is the broadcast ID!

        //Can message abstraction function
        int (*can_send_msg) (uint16_t can_id, uint8_t len, uint8_t* data);
        uint32_t _mtr_last_hb;
        float_conv flt_cnv;
    public:
        ODriveCanMtr(int (*send_func) (uint16_t can_id, uint8_t len, uint8_t* data), uint8_t node_id);
        void begin();
        //bool send_command(cmd_id cmd, uint8_t len, uint8_t* data);   
        int process_cmd(cmd_id cmd, uint8_t len, uint8_t* data);
        int process_msg(uint16_t can_id, uint8_t len, uint8_t* data);
        int stop();
        bool config_motor();
        int req_info_cmd(cmd_id cmd);

        set_axis_state(uint32_t state);
        set_cont_mode(uint32 cont_mode, uint32_t ip_mode);
        set_ip_pos(float ip_pos, int16_t vel, int16_t torque);
        set_input_vel(float vel, float torque);
        set_input_torq(float torque);
        set_lim(float vel_lim, float cur_lim);
        set_traj_vel_limit(float vel_lim);
        set_traj_accel_limits(float accel_limit, float decel_limit);
        set_traj_inertia(float interia);
        reboot_mtr();
        clear_errors(uint8_t error);
        set_absolute_position(float pos);
        set_position_gain(float pos_gain);
        set_velocity_gains(float vel_gain, float vel_integ_gain);

        ~ODriveCanMtr();

        uint8_t node_id();
        bool mtr_connected();
        mtr_values last_mtr_values;
};

#endif