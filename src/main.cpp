#include <Arduino.h>
#include "driver/twai.h"

#include "SRT_Odrive.h"
#include "SRT_OpenCan.h"
#include "SRT_CanRouter.h"

// --------- CAN TX abstraction ----------
int send_can_msg(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr) {
    twai_message_t msg = {};
    msg.identifier = can_id;
    msg.extd       = 0;               // 11-bit standard ID
    msg.rtr        = rtr ? 1 : 0;
    msg.self       = 0;
    msg.ss         = 0;

    if (len > 8) len = 8;
    msg.data_length_code = len;

    if (!rtr && len > 0 && data != nullptr) {
        memcpy(msg.data, data, len);
    }

    if (twai_transmit(&msg, pdMS_TO_TICKS(500)) == ESP_OK) {
        return 0;
    }
    return -1;
}

// --------- Global motor objects ---------
// ODrive node 1 (GIM)
SRT_OdriveMtr  odrive_mtr(&send_can_msg, 1);

// Lichuan / CANopen node 1
SRT_CanOpenMtr lichuan_mtr(&send_can_msg, 1);

// --------- TWAI init ----------
bool init_twai() {
    // Adjust GPIOs to your transceiver wiring
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_16, GPIO_NUM_17, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 20;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        return false;
    }

    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }

    Serial.println("TWAI started @ 1Mbps");
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\nSRT Router test start");

    if (!init_twai()) {
        Serial.println("CAN init failed, halting");
        while (1) delay(1000);
    }

    // Init ODrive side
    Serial.println("Init ODrive node 1...");
    odrive_mtr.begin();
    // Changed from init_motor to config_motor
    odrive_mtr.config_motor();
    odrive_mtr.set_lim(20.0f, 5.0f);

    // Init Lichuan side
    Serial.println("Enable Lichuan node 1...");
    lichuan_mtr.enable_motor();

    Serial.println("Setup complete");
}

void loop() {
    // Pump RX and route frames
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(1)) == ESP_OK) {
        if (!msg.rtr) {
            process_can_frame(msg.identifier,
                              msg.data_length_code,
                              msg.data,
                              odrive_mtr,
                              lichuan_mtr);
        }
    }

    // Simple periodic test every 5 seconds
    static unsigned long last_cmd = 0;
    if (millis() - last_cmd > 5000) {
        last_cmd = millis();

        // 1) Set Lichuan (CANopen) position for node 1
        //    Example: +10000 steps relative move; tune accel/decel as needed
        Serial.println("Lichuan: move +10000 steps");
        lichuan_mtr.move_relative(10000, 500, 500);

        // 2) Set ODrive (CANsimple) velocity for node 1
        //    Example: 5.0 rad/s, 0.5 Nm torque limit (tune for your setup)
        Serial.println("ODrive: set velocity 5.0 rad/s");
        odrive_mtr.set_ip_vel(5.0f, 0.5f);
    }

    delay(5);
}