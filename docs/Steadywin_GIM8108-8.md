# 🌀 Steadywin **GIM8108-8** — Integration Notes (with ODrive)

> Practical setup for the **GIM80 series, model GIM8108-8 (8:1 planetary)** with **ODrive** over CAN.
> This focuses on wiring, parameter conversion (gearbox), and sane defaults for bring‑up.

---

## 1) Key Specs (from vendor sheet)

| Field | Value |
|---|---|
| Driver model (vendor) | SDC104 |
| Nominal voltage | 48 V (range 12–56 V) |
| Rated power | 336 W |
| **Nominal torque (after reduce)** | **7.5 Nm** |
| **Stall torque (after reduce)** | **22 Nm** |
| **Nominal speed (after reduce)** | **110 rpm** |
| Max speed (after reduce) | 320 rpm |
| **Nominal current** | **7 A** |
| **Stall current** | **22 A** |
| Phase resistance | 0.67 Ω |
| Phase inductance | 0.45 mH |
| **Speed constant (after reduce)** | **6.67 rpm/V** |
| **Torque constant (after reduce)** | **≈ 1.0 Nm/A** |
| **Pole pairs** | **21** |
| **Gear ratio** | **8:1** (planetary, steel), backlash ≈ 15 arcmin |
| Encoder on driver | 16-bit (driver-integrated) |
| Second encoder | Yes (model-dependent) |
| Protection | IP54 |

> The torque/speed constants above (1.0 Nm/A and 6.67 rpm/V) are **at the output shaft** after the 8:1 reducer.

---

## 2) Using **ODrive** (motor-side parameters)

ODrive expects **motor-side** constants (before gearbox). Convert from the given **output** specs:

- Gear ratio **g = 8**  
- `Kv_motor = Kv_out * g = 6.67 × 8 = 53.36 rpm/V`
- `Kt_motor = 60 / (2π · Kv_motor) ≈ **0.179 Nm/A**`  (motor shaft)
- Phase resistance **R ≈ 0.67 Ω**, inductance **L ≈ 0.45 mH** (as provided)

> The vendor’s **1.0 Nm/A** aligns with **output** torque constant. Use **0.179 Nm/A** for `motor.torque_constant` in ODrive.

### Suggested ODrive axis config (starting points)

```text
axisX:
  motor:
    pole_pairs: 21
    torque_constant: 0.1790        # Nm/A (motor-side)
    current_lim: 15                        # A, start conservative (raise after thermal checks)
    requested_current_range: 25            # A, to avoid saturation during tuning
    resistance_calib_max_voltage: 8.0      # V, adjust if supply is low
    calibration_current: 8                  # A, match to motor scale

  encoder:
    # Choose ONE of these modes depending on your sensor:
    # mode: ENCODER_MODE_SPI_ABS  # e.g., AS5047/AS5600 (SPI abs)
    # mode: ENCODER_MODE_INCREMENTAL
    # mode: ENCODER_MODE_HALL
    cpr: 16384                    # if incremental 14-bit equiv; set to your hardware
    bandwidth: 1500               # 1000–2000 is typical for gimbals

  controller:
    # Velocity control example
    config:
      control_mode: CONTROL_MODE_VELOCITY_CONTROL
      input_mode:   INPUT_MODE_VEL_RAMP
      vel_ramp_rate: 5.0          # rev/s^2 motor shaft
      pos_gain: 20
      vel_gain: 0.02              # tune on rig
      vel_integrator_gain: 0.05   # add slowly if needed

  # State flow at runtime:
  # requested_state: AXIS_STATE_FULL_CALIBRATION_SEQUENCE
  # ... wait for success ...
  # requested_state: AXIS_STATE_CLOSED_LOOP_CONTROL
```

> **Encoder choice:** The “Resolution of Encoder on Driver = 16-bit” refers to the vendor driver. When using **ODrive directly**, you’ll typically wire **your own encoder** (SPI absolute like AS5047/AS5600, or an incremental A/B/Z). Pick mode + CPR accordingly.

---

## 3) Gearbox math for commands & limits

ODrive control values are **motor-side** (turns/s). If you reason about **output shaft** numbers, convert by **gear ratio g**:

- Velocity: `ω_motor = g × ω_output`
- Position: `θ_motor = g × θ_output`
- Torque:   `τ_motor ≈ τ_output / g` (ignoring gearbox efficiency)

**Example (velocity):** 5 rev/s **output** → `8 × 5 = 40 rev/s` motor input.

**In CANSimple** (`Set Input Vel` / `Set Limits`):
- `vel_limit` and `input_vel` are **turns/s at motor shaft**
- `current_limit` is motor phase current limit (A), not output torque

---

## 4) CAN Bring‑up (with this repo’s library)

**TX callback** signature:
```cpp
int send_can(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);
```

**Construct motor instance:**
```cpp
ODriveCanMtr mtr1(&send_can, 0); // Node 0
```

**Typical flow:**
```cpp
mtr1.set_cont_mode(velocity_control, vel_ramp);
mtr1.set_axis_state(full_calibration_sequence);
// wait for heartbeat → no errors
mtr1.set_axis_state(closed_loop_control);
mtr1.set_lim( (320.0/60.0) * 8, 15.0 );  // vel_limit ≈ max motor rev/s, current_lim 15 A
mtr1.set_ip_vel( (110.0/60.0) * 8, 0.0 ); // command nominal output speed (110 rpm) converted to motor rev/s
```

> For **heartbeat parsing**, feed incoming frames to `process_msg(can_id, len, data)`. Use `mtr_connected()` to gate commands until the drive is alive.

---

## 5) Tuning checklist (GIM80 flavor)

1. **Mechanical first:** rigid mount, no load initially.  
2. **Encoder sanity:** direction correct, CPR confirmed, bandwidth ~1000–2000.  
3. **Calibration:** full cal → check for errors → save config.  
4. **Closed loop at low speed:** start at 0.5–1 rev/s output → watch current & temperature.  
5. **Gains:** increase `vel_gain` until responsive but stable; add small `vel_integrator_gain` to remove steady error.  
6. **Limits:** set `current_lim` and `vel_limit` to realistic values; keep headroom.  
7. **Thermal check:** run for 2–5 minutes at nominal; record temps and current.  

---

## 6) Reference values (fill with **your** measurements)

- Measured pole pairs: **21**
- Measured Kv (motor): **53.36 rpm/V**
- Computed Kt (motor): **0.179 Nm/A**
- Phase R / L: **0.67 Ω / 0.45 mH**
- Output → motor conversion: gear **8:1**
- Preferred control mode: **Velocity / Position** (choose)
- Stable gains: `pos_gain = __`, `vel_gain = __`, `vel_integrator_gain = __`

---

## 7) Gotchas

- Vendor “16‑bit encoder on driver” does **not** mean you have 16‑bit output when bypassing the driver. Pick and wire your own encoder when using ODrive directly.  
- Post‑gear constants (1.0 Nm/A, 6.67 rpm/V) are **not** what ODrive wants; convert to motor-side.  
- If it runs away at closed loop entry: swap any two motor phases **or** invert encoder direction.  
- Long leads? Twist phases; shield encoders; add ferrites near the drive.  

---

*Prepared for GIM8108‑8 (8:1). Adapt numbers if using a different GIM80 variant.*
