# SIH Real PWM Outputs — Lockdown Bypass (`SIH_ACT_OUT`)

This document describes the PX4 changes that allow **Simulation-In-Hardware (SIH)** on a Pixhawk to drive **real MAIN PWM** (servos / ESC) for bench testing, and why that was blocked by default.

**Board used in testing:** Auterion / Pixhawk FMU **v6C** (`px4_fmu-v6c`), airframe **1101 SIH plane AERT**.

**Safety:** Bench only. Propeller removed. Prefer ESC unpowered until motor tests. Never use `SIH_ACT_OUT=1` for outdoor flight with SIH sensors.

---

## 1. Problem (stock PX4)

With `SYS_HITL = 2` (SIH enabled), Commander sets `vehicle_status.hil_state = HIL_STATE_ON`.

In stock PX4, Commander then forces:

```text
actuator_armed.lockdown = true   (whenever hil_state is ON)
```

Effects:

| Layer | Behavior with lockdown |
|--------|-------------------------|
| Mixing / `px4io` | Real MAIN outputs forced to **disarmed** PWM |
| `pwm_out_sim` | Still runs (sim plant / `actuator_outputs_sim`) |
| SIH dynamics | Continues from simulated actuators |
| Physical servos | Stay frozen at disarmed (e.g. 1500 / 1000) |

Symptom on the bench:

```text
arming_lockdown: True
pwm: [1500, 1500, 1500, 1000, ...]   # stuck
```

even though the vehicle is armed and flying the SIH mission.

This is intentional stock safety: SIH must not move real actuators by accident.

---

## 2. Solution overview

Add parameter **`SIH_ACT_OUT`**:

| Value | Meaning |
|------:|---------|
| `0` (default) | Stock behavior: HIL/SIH → lockdown → no real PWM motion |
| `1` | Allow real PWM while HIL/SIH is on (bench) |

Lockdown logic becomes:

```text
hil_lockdown = (hil_state == ON) AND (SIH_ACT_OUT == 0)
actuator_armed.lockdown = hil_lockdown OR throw_launch_in_progress
```

When `SIH_ACT_OUT = 1` and armed:

- `lockdown = False`
- `PWM_MAIN_*` follows Control Allocator like a normal plane
- SIH still uses simulated sensors + `pwm_out_sim` / `actuator_outputs_sim`

---

## 3. Files changed in PX4

### 3.1 `src/lib/systemlib/system_params.c`

New parameter:

```c
/**
 * Allow real PWM outputs during SIH / HITL
 * ...
 * @boolean
 * @reboot_required true
 * @group Simulation In Hardware
 */
PARAM_DEFINE_INT32(SIH_ACT_OUT, 0);
```

Default **0** keeps stock safety for all other airframes / users.

### 3.2 `src/modules/commander/Commander.hpp`

Bind the parameter in Commander’s param list:

```cpp
(ParamInt<px4::params::SIH_ACT_OUT>) _param_sih_act_out
```

### 3.3 `src/modules/commander/Commander.cpp`

**Before (stock):**

```cpp
_actuator_armed.lockdown =
    (_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON)
    || _multicopter_throw_launch.isThrowLaunchInProgress();
```

**After:**

```cpp
const bool hil_lockdown =
    (_vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON)
    && (_param_sih_act_out.get() == 0);

_actuator_armed.lockdown =
    hil_lockdown || _multicopter_throw_launch.isThrowLaunchInProgress();
```

Only this path was changed for real SIH PWM. Kill-switch / manual lockdown / force failsafe are unchanged.

### 3.4 `ROMFS/px4fmu_common/init.d/airframes/1101_rc_plane_sih.hil`

Bench defaults for SIH plane airframe:

```sh
# Real MAIN PWM for bench SIH (Pixhawk IO). Propeller must be removed / ESC unpowered.
param set-default PWM_MAIN_FUNC1 201   # Single Channel Aileron
param set-default PWM_MAIN_FUNC2 202   # Elevator
param set-default PWM_MAIN_FUNC3 203   # Rudder
param set-default PWM_MAIN_FUNC4 101   # Motor 1
param set-default SIH_ACT_OUT 1
```

Notes:

- `HIL_ACT_FUNC1..4` remain the SIH / `pwm_out_sim` mapping (sim plant).
- `PWM_MAIN_FUNC*` is the **physical** MAIN mapping on Pixhawk IO.
- Airframe default is motor on **MAIN 4**. If your airframe wires ESC on **MAIN 3**, override in QGC (see §6).

---

## 4. Boot / data-flow context (v6C + SIH)

Relevant `rcS` behavior when `SYS_HITL > 0`:

1. `sensors start -h` (HITL sensor path)
2. If `SYS_HITL == 2`: `simulator_sih start` (+ simulated baro/mag/GPS/…)
3. `px4io start` (still started — MAIN PWM via IO co-processor)
4. `commander start -h`
5. `pwm_out_sim start -m hil` (sim outputs; **not** `pwm_out`)

```text
  Controllers → Control Allocator
        │
        ├──► pwm_out_sim  → actuator_outputs_sim → SIH plant
        │
        └──► px4io MixingOutput (PWM_MAIN_*)
                 │
                 └──► physical MAIN pins   [only if lockdown == false]
```

`SIH_ACT_OUT` only affects whether Commander sets `actuator_armed.lockdown`. It does not replace SIH sensors with real IMUs.

---

## 5. How to enable / verify

### Flash

```bash
make px4_fmu-v6c_default upload
```

Close QGroundControl first if USB reports `Device or resource busy`.

### Params

```text
param show SIH_ACT_OUT     # expect 1
param show SYS_HITL        # expect 2
```

If needed:

```text
param set SIH_ACT_OUT 1
param save
reboot
```

### Armed checks

```text
listener actuator_armed
```

Expect:

```text
armed: True
lockdown: False
```

```text
px4io status
```

Expect:

```text
arming_fmu_armed: True
arming_lockdown: False
pwm: [...]   # values changing, not stuck at disarmed
```

---

## 6. Bench wiring notes (from hardware bring-up)

These are **configuration / calibration** findings, not further firmware patches.

### 6.1 MAIN function map

Airframe **1101** default:

| MAIN | Function | Typical DIS |
|-----:|----------|-------------|
| 1 | Aileron (201) | 1500 |
| 2 | Elevator (202) | 1500 |
| 3 | Rudder (203) | 1500 |
| 4 | Motor 1 (101) | 1000 |

If hardware is **ESC on MAIN 3, rudder on MAIN 4**, set:

```text
PWM_MAIN_FUNC3 101
PWM_MAIN_FUNC4 203
PWM_MAIN_DIS3 1000
PWM_MAIN_DIS4 1500
PWM_MAIN_MIN3/MAX3 1000/2000
PWM_MAIN_MIN4/MAX4 1000/2000
```

Mismatch (e.g. motor channel with `DIS=1500`) is unsafe and breaks ESC scaling.

### 6.2 ESC deadband (~70%+)

If the motor only spins near 0.8–0.9 throttle while `px4io` PWM rises correctly, the ESC endpoints are wrong. Recalibrate ESC to **1000–2000 µs** (prop off). Changing PX4 min/max alone does not recalibrate the ESC.

### 6.3 `actuator_test` syntax

Wrong (targets servo function ~201):

```text
actuator_test set -m 101 -v 0.3 -t 3
```

Correct for Motor 1:

```text
actuator_test set -m 1 -v 0.3 -t 3
# or
actuator_test set -f 101 -v 0.3 -t 3
```

Servos:

```text
actuator_test set -s 1 -v 1 -t 3   # aileron
```

### 6.4 Small surface motion in Mission

With SIH tracking a ~40° bank (setpoint ≈ actual), `actuator_servos` often stays at a few percent. Holding bank does not need large aileron. Full travel appears in Manual / `actuator_test`. That is expected closed-loop behavior for the SIH plant, not a lockdown failure.

---

## 7. Build / flash reminder

```bash
make px4_fmu-v6c_default
make px4_fmu-v6c_default upload
```

Firmware artifact:

```text
build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

`SIH_ACT_OUT` is marked `@reboot_required true` — reboot after changing it.

---

## 8. Summary

| Item | Detail |
|------|--------|
| Root cause | HIL/SIH sets `actuator_armed.lockdown`, freezing real PWM |
| Fix | `SIH_ACT_OUT`: lockdown only if HIL **and** param == 0 |
| Default | `0` (safe) |
| SIH plane 1101 | Defaults `SIH_ACT_OUT=1` + `PWM_MAIN_FUNC*` |
| Touched modules | `system_params.c`, `Commander.cpp` / `.hpp`, `1101_rc_plane_sih.hil` |
| Not changed | SIH physics plant, TECS, Control Allocator (for this lockdown feature) |

Related HySoar SIH plant work (AdvancedLiftDrag, SITL airframe 10041, etc.) is separate from this lockdown patch; see other reports under `docs/hysoar/` / `reports/` if needed.

---

## Appendix — Phoenix 2400 plant + larger surface commands

Approximate Phoenix 2400 sizing was applied to SIH AdvancedLiftDrag / airframes (with control-moment scale **0.4** so Mission commands larger throws):

| Spec | Value used in SIH |
|------|-------------------|
| Wingspan | 2.40 m |
| Overall length | 1.132 m (CP / scale reference) |
| Flying weight | 1.16 kg (`SIH_MASS`) |
| AR (kept) | 6.5 |
| Wing area | 0.886 m² (= b²/AR) |
| MAC | 0.369 m |
| `cell/cem/cen_ctrl` | Gazebo baselines × **`SIH_CTRL_EFF`** (default **0.12**) |

**Why ×0.4 alone looked unchanged:** Phoenix area×span is ~4× `advanced_plane`, so net control moments stayed similar. Use `SIH_CTRL_EFF` (tunable without editing coeffs). Rebuild/flash required once for the new param; then:

```text
param set SIH_CTRL_EFF 0.12   # try 0.08 if still tiny during roll-in
param save
```

**Important:** While *holding* ~40° bank, aileron stays small even with weak aero (attitude error ≈ 0). Watch surfaces during **roll entry**, or use Manual / `actuator_test`.

Inertia (from advanced_plane × mass×span² scale): `SIH_IXX=0.597`, `SIH_IYY=0.441`, `SIH_IZZ=0.446`.

Files: `advanced_liftdrag.hpp`, `sih.hpp` (`ADV_PLANE_*`), `sih_params.c` (`SIH_CTRL_EFF`), `10041_sihsim_airplane`, `1101_rc_plane_sih.hil`.
