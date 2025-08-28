# 📚 Background: CAN 2.0 and ODrive CAN

This document summarizes how CAN 2.0 works in the context of ODrive, and how ODrive defines CAN IDs and commands.

> The ODrive documentation ships a `.dbc` describing the CAN signals (see `/assets`). A DBC viewer like
> [this one](https://www.csselectronics.com/pages/dbc-editor-can-bus-database) makes it human‑readable. The upstream CAN protocol reference is
> the ODrive manual: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html

---

## Controller Area Network (CAN) — essentials

CAN is a multi‑drop differential bus. For our use case, **CAN 2.0** (Standard/Extended) is sufficient — **FD/XL** are not required by ODrive.

- **Bit rates:** 125k / 250k / 500k / 1000k (500k is common; 1M can be unreliable over ~40m).
- **Priority:** Lower numeric ID wins (dominant bits beat recessive bits on the bus).
- **Physical layer:** Twisted pair; CH/CL as a differential pair. Noise appearing equally on both is rejected.

### Frame overview (Standard CAN, OSI L2)

- **Arbitration:** ID (11‑bit for Standard), IDE (0 for Standard), **RTR** (special bit for remote requests)
- **Control:** DLC (0..8 bytes of data in CAN 2.0)
- **Data:** up to 8 bytes
- **CRC / EoF:** with **ACK** bit driven dominant by receivers that validated the frame

#### RTR (Remote Transmission Request)
A node can request a message by sending the same ID with **RTR=1** and **no data**. The owner of that ID replies with **RTR=0** and data.
If a data frame and an RTR frame with the same ID collide, the **data frame wins** (as desired).

---

## ODrive CAN Mapping

ODrive uses **Standard 11‑bit CAN IDs** split into:
- **Node ID**: upper 6 bits (0..63; `0x3F` = broadcast)
- **Command ID**: lower 5 bits (0..31)

```
  10..5           4..0
[ NodeID ] [ Command ]
```

- Extract Node ID: `node_id = (can_id >> 5);`
- Extract Command: `cmd_id  = (can_id & 0x1F);`

### Command table (subset)

| Command | Hex |
|:--|:--:|
| Get Version | 0x00 |
| Heartbeat | 0x01 |
| Estop | 0x02 |
| Get Error | 0x03 |
| Rx sdo | 0x04 |
| Tx sdo | 0x05 |
| Address | 0x06 |
| Set Axis State | 0x07 |
| **(reserved)** | 0x08 |
| Get Encoder Estimates | 0x09 |
| **(reserved)** | 0x0A |
| Set Controller Mode | 0x0B |
| Set Input Pos | 0x0C |
| Set Input Vel | 0x0D |
| Set Input Torque | 0x0E |
| Set Limits | 0x0F |
| **(reserved)** | 0x10 |
| Set Traj Vel Limit | 0x11 |
| Set Traj Accel Limits | 0x12 |
| Set Traj Inertia | 0x13 |
| Get Iq | 0x14 |
| Get Temperature | 0x15 |
| Reboot | 0x16 |
| Get Bus Voltage Current | 0x17 |
| Clear Errors | 0x18 |
| Set Absolute Position | 0x19 |
| Set Pos Gain | 0x1A |
| Set Vel Gains | 0x1B |
| Get Torques | 0x1C |
| Get Powers | 0x1D |
| **(reserved)** | 0x1E |
| Enter DFU Mode | 0x1F |

### Node & Command extraction

```cpp
uint8_t node_id  = (can_id >> 5);
uint8_t command  = (can_id & 0x1F);
```

See the library header for enums and helpers used by the implementation.

---

## Control Presets

**Position control:**
```cpp
mtr1.set_cont_mode(position_control, vel_ramp);
mtr1.set_axis_state(closed_loop_control);
// use set_ip_pos(...) to command position
```

**Velocity control:**
```cpp
mtr1.set_cont_mode(velocity_control, vel_ramp);
mtr1.set_axis_state(closed_loop_control);
// use set_ip_vel(...) to command velocity (rev/s at encoder)
```

---

## Notes

- Extended (29‑bit) CAN frames can coexist on the bus but ODrive itself uses **Standard 11‑bit** IDs.
- Broadcast Node ID `0x3F` will be accepted by all motors — use with care.
