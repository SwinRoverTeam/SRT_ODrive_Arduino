#include <Arduino.h>
#include "driver/twai.h"
#include "SRT_Odrive.h"
#include "SRT_OpenCan.h"

// -------- CAN config --------
#define CAN_TX_GPIO GPIO_NUM_16
#define CAN_RX_GPIO GPIO_NUM_18
#define CAN_BAUDRATE_500K TWAI_TIMING_CONFIG_500KBITS()

// Fixed profile parameters
const uint32_t FIXED_VEL = 1000;   // 6081h BIGGER NUMBER = SLOWER SPEED
const uint32_t FIXED_ACC = 1000;    // 6083h
const uint32_t FIXED_DEC = 1000;    // 6084h 1 IS VERY FAST DO NOT GO BELOW 300 FOR ANY OF THESE!!!

    bool gim_moved_out_1 = false; 
// -------- Node ID arrays --------
// ODrive: 1+4n, between 1 and 29
//set odrive motors to as many as needed and and there can_id addresses
constexpr uint8_t ODRIVE_NODE_IDS[]   = {1, 5, 9, 13};
//do not touch this function this tests how many of each motor there m4 is of odrive
constexpr size_t  NUM_ODRIVE_MOTORS   = sizeof(ODRIVE_NODE_IDS) / sizeof(ODRIVE_NODE_IDS[0]);

// CANopen: 0..31, but 0 reserved for broadcast
//set lichuan motors to as many as needed and and there can_id addresses
constexpr uint8_t OPENCAN_NODE_IDS[]  = {1, 2, 3, 4, 5};
//do not touch this function this tests how many of each motor there is of lichuan
constexpr size_t  NUM_OPENCAN_MOTORS  = sizeof(OPENCAN_NODE_IDS) / sizeof(OPENCAN_NODE_IDS[0]);

// -------- Shared CAN TX (don’t touch logic) --------
// if you want to read more on this go ahead https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/twai.html#_CPPv418twai_transmitPK18twai_message_t15TickType_t
int send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr) {
    twai_message_t msg{};
    msg.identifier = can_id;
    msg.extd = 0;// 11-bit ID normal frame length for can
    msg.rtr = rtr ? 1 : 0; // remote transmission request this is for requesting data
    msg.self = 0; // self reception request 
    msg.ss = 0; // single shot transmission

    //this makes sure that the length of data being sent is not more than 8 bytes if it is it trims it down to 8 bytes
    if (len > 8) len = 8; 
    msg.data_length_code = len;
    if (!rtr && len && data) memcpy(msg.data, data, len);
// pdMS_TO_TICKS converts milliseconds to RTOS ticks
    return (twai_transmit(&msg, pdMS_TO_TICKS(500)) == ESP_OK) ? 0 : -1;
}

// -------- TWAI init --------
bool init_twai(gpio_num_t tx, gpio_num_t rx) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 20;
    twai_timing_config_t t_config = CAN_BAUDRATE_500K;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }
    Serial.println("TWAI started @ 500 kbit/s");
    return true;
}

// -------- Motor objects (arrays) --------
SRT_OdriveMtr   odrives[NUM_ODRIVE_MOTORS] = {
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[0]), // First motor with can id 1
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[1]), // Second motor with can id 5
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[2]), // Third motor with can id 9
    SRT_OdriveMtr(&send_can_msg, ODRIVE_NODE_IDS[3]), // Fourth motor with can id 13
};

SRT_CanOpenMtr  opencans[NUM_OPENCAN_MOTORS] = {
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[0]),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[1]),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[2]),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[3]),
    SRT_CanOpenMtr(&send_can_msg, OPENCAN_NODE_IDS[4]),
};
void init_odrive_motors() {
    for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
        odrives[i].begin();
        //Populate cases when more motors are added or we want seperate settings for each motor
        switch (i) {
            default:
                odrives[i].set_cont_mode(position_control, trap_traj);
                odrives[i].set_axis_state(closed_loop_control);
                odrives[i].set_lim(10.0f, 20.0f);
                odrives[i].set_traj_vel_limit(5.0f);
                odrives[i].set_traj_accel_limits(5.0f, 5.0f);
                break;
        }
    }
}

void init_opencan_motors() {
    for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
        opencans[i].enable_motor();
        delay(10);
    }
}

// ===== Serial command reading =====
/*
String readSerialLine() {
    String input = ""; //reads sring that is typed
    while (Serial.available() > 0) {
        char c = Serial.read(); //reads serial
        if (c == '\n' || c == '\r') break; //reads untill enter
        input += c;
    }
    input.trim();
    return input; //returns input back to loop
}
*/

// -------- CAN RX: hand frames to all motors, they self-filter by node_id --------
void can_check_recv() {
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(2)) == ESP_OK) {
        if (msg.rtr) continue;

        // ODrive motors
        for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
            odrives[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
        // CANopen motors
        for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
            opencans[i].process_msg(msg.identifier, msg.data_length_code, msg.data);
        }
    }
}

// =========== Serial Commands ============
/*
  Commands:
    help
    list
    m<idx> <position>
 
  Examples:
    m0 10000
    m2 -5000
 */

 /*
void handleSerialCommand(const String &cmd) {
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("help")) { // when help typed it diaplayes help
        Serial.println("Commands:");
        Serial.println("  list");
        Serial.println("  m<idx> <pos>");
        Serial.print  ("Fixed vel/acc/dec: ");
        Serial.print  (FIXED_VEL);
        Serial.print  (", ");
        Serial.print  (FIXED_ACC);
        Serial.print  (", ");
        Serial.println(FIXED_DEC);
        Serial.println("Examples: m0 10000   or   m1 -20000");
        return;
    }

    if (cmd.equalsIgnoreCase("list")) { // when list typed shows list of CAN IDs
        Serial.println("LiChuan nodes:");
        for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
            Serial.print("  index ");
            Serial.print(i);
            Serial.print(" -> CAN ID ");
            Serial.println(OPENCAN_NODE_IDS[i]);
        }
        return;
    }

    // Expect commands like "m0 10000"
    if (cmd.charAt(0) == 'm' || cmd.charAt(0) == 'M') {
        int space1 = cmd.indexOf(' ');
        if (space1 < 0) {
            Serial.println("ERR: usage m<idx> <pos>");
            return;
        }

        String motorStr = cmd.substring(1, space1); //reading from second digit and beyond
        int idx = motorStr.toInt(); //the index of the motor the m1 (the 1)
        if (idx < 0 || idx >= (int)NUM_OPENCAN_MOTORS) { 
            Serial.println("ERR: motor index out of range");
            return;
        }

        String posStr = cmd.substring(space1 + 1); //reading from beyond the first 2 digits
        posStr.trim();
        if (posStr.length() == 0) {
            Serial.println("ERR: missing position");
            return;
        }

        long targetPos = posStr.toInt(); //converts to integer from string

        // Update only target position; acc/dec are fixed
        opencans[idx].move_relative((int32_t)targetPos, FIXED_ACC, FIXED_DEC);

        Serial.print("OK: motor index "); //prints what has been sent in serial for our human brains to understand
        Serial.print(idx);
        Serial.print(" (CAN ID ");
        Serial.print(OPENCAN_NODE_IDS[idx]);
        Serial.print(") -> target ");
        Serial.print(targetPos);
        Serial.print(" (vel ");
        Serial.print(FIXED_VEL);
        Serial.print(", acc ");
        Serial.print(FIXED_ACC);
        Serial.print(", dec ");
        Serial.print(FIXED_DEC);
        Serial.println(")");
        return;
    }

    Serial.println("Unknown command. Type 'help'.");
}
*/

void setup() {
    Serial.begin(115200);
    delay(1000);

    if (!init_twai(CAN_TX_GPIO, CAN_RX_GPIO)) {
        Serial.println("CAN init failed");
        while (1) delay(1000);
    }

    init_odrive_motors();
    init_opencan_motors();

    // Define current position as 0 for motor 0
    odrives[0].set_absolute_position(0.0f);

    // Debug: check axis state/error for motor 0
    Serial.print("Axis state: ");
    Serial.println(odrives[0].last_mtr_values.axis_state);
    Serial.print("Axis error: 0x");
    Serial.println(odrives[0].last_mtr_values.axis_error, HEX);

    Serial.println("All motors initialised");
}
void loop() {
    can_check_recv();// this checks for any incoming can messages and processes them accordingly. Dont remove stinky!
    
    // ================ Opens Serial comms =============
    /*
    if (Serial.available() > 0) { // checks for serial and commands
        String command = readSerialLine();
        handleSerialCommand(command);
    }
    */
    // LIBRARY DOES ALL SERIAL WORK
    if (Serial.available() > 0) {
        String cmd = SRT_CanOpenMtr::readSerialLine();
        SRT_CanOpenMtr::handleSerialCommand(cmd, opencans, NUM_OPENCAN_MOTORS, OPENCAN_NODE_IDS, FIXED_ACC, FIXED_DEC);
    }

    for (size_t i = 0; i < NUM_ODRIVE_MOTORS; ++i) {
        switch (i) {
            case 0:
                if (!gim_moved_out_1) {
                    // Move motor 0 (CAN ID 1) to +0.5 turns
                    odrives[i].set_ip_pos(0.5f, 0, 0);
                    delay(1000);
                    gim_moved_out_1 = true;
                } else {
                    // Move motor 0 back to 0 turns
                    odrives[i].set_ip_pos(0.0f, 0, 0);
                    delay(1000);
                    gim_moved_out_1 = false;
                }
                break;

            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
        }
    }

    // You can leave this loop empty for now
    for (size_t i = 0; i < NUM_OPENCAN_MOTORS; ++i) {
        switch (i) {
            case 0:
            case 1:
            case 2:
            case 3:
                break;
            case 4:
            /* do not know if this works yet
                static bool pos_toggle = false;

                // CANopen node 5: simple back‑and‑forth
                int32_t target = pos_toggle ? 10000 : -10000; // steps / counts (depends on drive)
                    opencans[4].set_profile_position(target, 1000, 500, 500);

                    pos_toggle = !pos_toggle;
                    delay(1000);
            
                // No commands for other ODrives yet
                */
               break; 
        }
    
    }


    delay(5);
}


