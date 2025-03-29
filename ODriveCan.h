#ifndef ODriveCan
#define ODriveCan

#include <Arduino.h>

#include <mcp_can.h>
#include <spi.h>


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
    set_trak_intertia       = 0x013,
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
class ODriveCanMtr
{
    private:
        /* data */
        MCP_CAN* _mcp_obj;
        uint8_t _node_id; //Be careful 0x3f is the broadcast ID!
        bool can_attached;
    public:
        ODriveCanMtr(MCP_CAN &can_obj, uint8_t node_id);
        void begin();
        //bool send_command(cmd_id cmd, uint8_t len, uint8_t* data);   
        bool process_cmd(uint8_t cmd, uint8_t len, uint8_t* data);
        bool proccess_msg();
        ~ODriveCanMtr();

        uint8_t node_id();
};

#endif