# PX4 v1.16.1 — Autonomous Soaring Implementation
## Complete Change Documentation (Final Version)

**Repository:** `/home/radhouene/Pictures/PX4-Autopilot`  
**Base version:** PX4 v1.16.1  
**Features:** Engine-off gliding (AUTO_MISSION) + thermal loiter (AUTO_LOITER)  
**Control authority:** ROS2 companion via XRCE-DDS **and** PX4 CLI  

---

## Architecture Overview

```
ROS2 Companion (XRCE-DDS)                  PX4 CLI
        │                                       │
        │ AutosoaringControl uORB               │ commander mode soar:glide
        │ /fmu/in/autosoaring_control           │ commander mode soar:thermal
        ▼                                       ▼ commander mode soar:off
FixedwingPositionControl::Run()        VEHICLE_CMD_CUSTOM_0
        │  (DDS poll + staleness watchdog)        │
        └──────────────┬────────────────────────┘
                       │ _autosoaring_control flags
                       ▼
        FixedwingPositionControl::control_auto()
        │  ┌────────────────────────────────┐
        │  │  Altitude safety (DDS only)    │
        │  │  FW_ALT_MIN floor + latch      │
        │  │  FW_ALT_MAX ceiling + hyst     │
        │  └────────────────────────────────┘
        │
        ├── effective_glide=true  → DO_SET_MODE(AUTO_MISSION) + TECS glide
        │
        └── effective_thermal=true → DO_REPOSITION(lat,lon,radius) → AUTO_LOITER + TECS glide
                                      (pinned centre — never moves after first activation)
                       │
                       ▼
              TECS (gliding_mode_enabled=true)
              • spe_weighting=0, ske_weighting=2  (pitch = speed-on-elevator)
              • throttle = throttle_min (hard 0 via actuator gate)
              • throttle integrator decays with tau = FW_GLIDE_I_DECAY
              • best-glide airspeed override = FW_GLIDE_AIRSPD
              • altitude_rate_sp clipped to ≤ 0 (no climb commands in glide)
              • fast_descend disabled (conflicts with speed-on-elevator)
```

---

## Files Modified / Created

| File | Change Type |
|------|-------------|
| `CMakeLists.txt` | Modified |
| `ROMFS/.../4008_gz_advanced_plane` | Modified |
| `msg/CMakeLists.txt` | Modified |
| `msg/AutosoaringControl.msg` | **Created** |
| `src/lib/tecs/TECS.hpp` | Modified |
| `src/lib/tecs/TECS.cpp` | Modified |
| `src/modules/fw_pos_control/FixedwingPositionControl.hpp` | Modified |
| `src/modules/fw_pos_control/FixedwingPositionControl.cpp` | Modified |
| `src/modules/fw_pos_control/fw_path_navigation_params.c` | Modified |
| `src/modules/navigator/navigator_main.cpp` | Modified |
| `src/modules/commander/Commander.cpp` | Modified |
| `src/modules/uxrce_dds_client/dds_topics.yaml` | Modified |

---

## 1. `CMakeLists.txt` (root)

**Why:** `std::optional` used in Gazebo soaring plugin helpers requires C++17.

```diff
- set(CMAKE_CXX_STANDARD 14)
+ set(CMAKE_CXX_STANDARD 17) # Required by std::optional and structured bindings used in Gazebo soaring plugins
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

---

## 2. `ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_advanced_plane`

**Why:** PX4 default `FW_THR_MIN = 0.05` prevents true engine-off gliding. TECS clamps throttle to this floor, so the UAV always has residual thrust even with gliding mode enabled.

```diff
- param set-default FW_THR_MIN 0.05
+ param set-default FW_THR_MIN 0.00 # 0.00 enables true engine-off gliding; PX4 default 0.05 prevents it
```

---

## 3. `msg/CMakeLists.txt`

**Why:** Register the new uORB message so it is compiled and available to all modules.

```diff
  set(msg_files
+     AutosoaringControl.msg
      ActionRequest.msg
      ...
```

---

## 4. `msg/AutosoaringControl.msg` *(new file)*

**Why:** Defines the uORB topic published by the ROS2 companion via XRCE-DDS to command soaring modes. Deliberately minimal — only what is physically necessary.

```
uint64  timestamp               # time since system start (microseconds)

# Soaring mode flags (published by ROS2 companion via XRCE-DDS)
bool    glide_mode_enabled      # True: throttle=0, follow mission waypoints (glide)
bool    thermal_mode_enabled    # True: throttle=0, orbit at thermal centre (loiter)

# Thermal loiter geometry (only relevant when thermal_mode_enabled=true)
float32 loiter_radius_m         # Orbit radius [m]; 0 = use NAV_LOITER_RAD default
float32 loiter_lat              # Thermal centre latitude  [deg]; 0 = current position
float32 loiter_lon              # Thermal centre longitude [deg]; 0 = current position
```

**Usage from ROS2:**
```bash
# Start gliding (follow mission, engine off):
ros2 topic pub /fmu/in/autosoaring_control px4_msgs/msg/AutosoaringControl \
  "{timestamp: 0, glide_mode_enabled: true, thermal_mode_enabled: false, ...}"

# Start thermal at current position:
ros2 topic pub /fmu/in/autosoaring_control px4_msgs/msg/AutosoaringControl \
  "{timestamp: 0, glide_mode_enabled: false, thermal_mode_enabled: true,
    loiter_radius_m: 0.0, loiter_lat: 0.0, loiter_lon: 0.0}"

# Start thermal at specific GPS coordinates:
ros2 topic pub /fmu/in/autosoaring_control px4_msgs/msg/AutosoaringControl \
  "{timestamp: 0, glide_mode_enabled: false, thermal_mode_enabled: true,
    loiter_radius_m: 80.0, loiter_lat: 47.3977, loiter_lon: 8.5456}"

# Stop soaring:
ros2 topic pub /fmu/in/autosoaring_control px4_msgs/msg/AutosoaringControl \
  "{timestamp: 0, glide_mode_enabled: false, thermal_mode_enabled: false, ...}"
```

---

## 5. `src/lib/tecs/TECS.hpp`

### 5.1 `Param` struct — new fields

```cpp
struct Param {
    ...
    float gliding_airspeed_setpoint{15.0f};  ///< Fixed TAS during glide (best-glide speed) [m/s]
    float glide_i_decay{10.0f};              ///< Throttle integrator decay tau [s]; I(t)=I(0)*exp(-t/tau)
};
```

### 5.2 `Flag` struct — new field

```cpp
struct Flag {
    ...
    bool gliding_mode_enabled{false};  ///< True during engine-off soaring (glide or thermal)
};
```

### 5.3 Public setters (added to class TECS)

```cpp
void set_gliding_mode_enabled(bool enabled)    { _control_flag.gliding_mode_enabled = enabled; }
bool get_gliding_mode_enabled() const          { return _control_flag.gliding_mode_enabled; }
void set_gliding_airspeed_setpoint(float spd)  { _control_param.gliding_airspeed_setpoint = spd; }
void set_glide_i_decay(float tau)              { _control_param.glide_i_decay = math::max(tau, 0.1f); }
```

### 5.4 Default initialiser values updated

```cpp
static constexpr Param _default_param{
    ...
    .gliding_airspeed_setpoint = 15.0f,
    .glide_i_decay = 10.0f,
};
static constexpr Flag _default_flag{
    ...
    .gliding_mode_enabled = false,
};
```

---

## 6. `src/lib/tecs/TECS.cpp`

### 6.1 `_updateSpeedAltitudeWeights()` — gliding branch

**Physics:** In glide, pitch is the **only actuator**. It must control airspeed (speed-on-elevator). Setting `spe_weighting=0, ske_weighting=2` redirects the entire SEB error to kinetic energy, making pitch a pure airspeed controller.

```cpp
TECSControl::SpecificEnergyWeighting
TECSControl::_updateSpeedAltitudeWeights(const Param &param, const Flag &flag)
{
    SpecificEnergyWeighting weight{};

    if (flag.gliding_mode_enabled) {
        weight.spe_weighting = 0.0f;  // altitude not controlled by pitch in glide
        weight.ske_weighting = 2.0f;  // full speed control via pitch
        return weight;
    }
    // ... normal TECS weighting code ...
}
```

**Effect:** When `gliding_mode_enabled=true`, this one early-return eliminates the need for any duplicate gliding code path downstream. All TECS functions that call `_updateSpeedAltitudeWeights` automatically behave correctly in glide.

### 6.2 `calcTrueAirspeedSetpoint()` — airspeed override

**Physics:** Best-glide airspeed (V_bg) minimises the glide angle. It is different from the cruise airspeed set in the mission. We override the setpoint here, before TECS uses it.

```cpp
float TECS::calcTrueAirspeedSetpoint(...)
{
    if (_control_flag.gliding_mode_enabled) {
        return _control_param.gliding_airspeed_setpoint;  // best-glide TAS override
    }
    return lerp(eas_to_tas * eas_setpoint, _control_param.tas_max, _fast_descend);
}
```

### 6.3 `TECS::update()` — altitude rate clipping

**Physics:** A glider cannot climb. Any positive altitude rate setpoint means the navigator is requesting a climb — physically impossible without thrust. Clipping to ≤ 0 prevents TECS from pitching up to chase an unreachable altitude reference.

```cpp
if (_control_flag.gliding_mode_enabled) {
    control_setpoint.altitude_reference.alt_rate        = math::min(alt_rate, 0.0f);
    control_setpoint.altitude_rate_setpoint_direct      = math::min(alt_rate_sp_direct, 0.0f);
}
```

### 6.4 `_setFastDescend()` — fast-descend disabled in glide

**Physics:** Fast-descend sets `fast_descend→1` which drives `ske_weighting→2`. This conflicts with glide mode's weight assignment (both set the same weights but via different paths, creating undefined behaviour).

```cpp
void TECS::_setFastDescend(...)
{
    if (_control_flag.gliding_mode_enabled) {
        _fast_descend = 0.0f;              // disabled: conflicts with speed-on-elevator weighting
        _enabled_fast_descend_timestamp = 0U;
        return;
    }
    // normal fast-descend logic ...
}
```

### 6.5 `_calcThrottleControlUpdate()` — integrator decay

**Physics:** During glide the throttle integrator is not driven. Without decay it retains its powered-flight value and causes a throttle surge (and pitch disturbance) on engine restart. Exponential decay `I(t) = I(0)·exp(-t/τ)` returns it to zero over τ seconds.

```cpp
void TECSControl::_calcThrottleControlUpdate(...)
{
    if (flag.gliding_mode_enabled) {
        const float decay = math::max(param.glide_i_decay, 0.1f);
        _throttle_integ_state -= dt * _throttle_integ_state / decay;
        return;
    }
    // normal integrator update ...
}
```

Default `FW_GLIDE_I_DECAY = 10 s` → residual ~5% after 30 s, ~0% after 60 s.

### 6.6 `_calcThrottleControlOutput()` — hard-idle throttle

**Physics:** Even if some energy error is non-zero, the engine must be off. This gate overrides TECS output and returns `throttle_min` (set to 0 in the airframe file).

```cpp
float TECSControl::_calcThrottleControlOutput(...)
{
    if (flag.gliding_mode_enabled) {
        return param.throttle_min;  // hard idle — engine off
    }
    // normal throttle output ...
}
```

---

## 7. `src/modules/fw_pos_control/fw_path_navigation_params.c`

The following parameters are added in group **"FW Soaring"**:

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `FW_ALT_MIN` | 100.0 | m | Soaring altitude floor. Below this, soaring is latched off (DDS path only). |
| `FW_ALT_MAX` | 500.0 | m | Altitude ceiling. Reaching this switches thermal→glide. |
| `FW_ALT_HYST` | 20.0 | m | Hysteresis below `FW_ALT_MAX` to re-enable thermal after ceiling. |
| `FW_GLIDE_AIRSPD` | 10.0 | m/s | Best-glide TAS setpoint overrides mission airspeed during glide. |
| `FW_POLAR_A` | 0.003 | — | Glide polar coefficient a (sink = a·V² + b). Fit from flight-test. |
| `FW_POLAR_B` | 0.50 | m/s | Glide polar constant b (minimum sink rate). Fit from flight-test. |
| `FW_GLIDE_RAMP_T` | 2.0 | s | Throttle ramp time on glide→powered transition (prevents propwash surge). |
| `FW_THERMAL_BANK` | 40.0 | deg | Maximum bank angle during thermal loiter. |
| `FW_GLIDE_I_DECAY` | 10.0 | s | Throttle integrator decay time constant during gliding. |

**Wiring in `FixedwingPositionControl::parameters_update()`:**
```cpp
_tecs.set_gliding_airspeed_setpoint(_param_fw_glide_airspd.get());
_tecs.set_glide_i_decay(_param_fw_glide_i_decay.get());
```

---

## 8. `src/modules/fw_pos_control/FixedwingPositionControl.hpp`

### 8.1 New include

```cpp
#include <uORB/topics/autosoaring_control.h>    // ROS2 soaring command topic (via XRCE-DDS)
```

### 8.2 New subscription and publication

```cpp
uORB::Subscription _autosoaring_control_sub{ORB_ID(autosoaring_control)};
uORB::Publication<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};
```

### 8.3 New state variables

```cpp
// Soaring state
autosoaring_control_s _autosoaring_control{};       ///< Last received soaring command
hrt_abstime  _autosoaring_last_recv_us{0};          ///< Timestamp of last valid message
hrt_abstime  _soaring_mode_cmd_last_us{0};          ///< Thermal DO_REPOSITION debounce
hrt_abstime  _soaring_dds_inhibit_until_us{0};      ///< DDS blocked until this time (soar:off cooldown)
bool         _soaring_expect_set_mode{false};       ///< Paired DO_SET_MODE flag (soar:glide sends CUSTOM_0 + DO_SET_MODE together)
double       _soaring_last_lat{NAN};                ///< Last thermal centre lat sent to navigator
double       _soaring_last_lon{NAN};                ///< Last thermal centre lon sent to navigator
bool         _alt_max_reached{false};               ///< Altitude ceiling hysteresis state
bool         _soaring_forbidden_latched{false};     ///< Latch: soaring blocked below FW_ALT_MIN
bool         _soaring_local_override{false};        ///< CLI active: bypasses alt check + watchdog
bool         _soaring_was_thermal{false};           ///< Previous-cycle thermal state for auto-exit detection
```

### 8.4 New parameters

```cpp
(ParamFloat<px4::params::FW_GLIDE_AIRSPD>) _param_fw_glide_airspd,
(ParamFloat<px4::params::FW_ALT_MIN>)      _param_fw_alt_min,
(ParamFloat<px4::params::FW_ALT_MAX>)      _param_fw_alt_max,
(ParamFloat<px4::params::FW_ALT_HYST>)     _param_fw_alt_hyst,
(ParamFloat<px4::params::FW_GLIDE_I_DECAY>) _param_fw_glide_i_decay
```

---

## 9. `src/modules/fw_pos_control/FixedwingPositionControl.cpp`

### 9.1 `vehicle_command_poll()` — CUSTOM_0 handler (CLI soaring)

Handles `commander mode soar:glide / soar:thermal / soar:off`.

```cpp
} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_CUSTOM_0) {
    _autosoaring_control.glide_mode_enabled   = (vehicle_command.param1 > 0.5f);
    _autosoaring_control.thermal_mode_enabled = (vehicle_command.param2 > 0.5f);
    _autosoaring_control.loiter_radius_m      = (vehicle_command.param3 > FLT_EPSILON)
                                                ? vehicle_command.param3 : 0.0f;

    const bool enabling = _autosoaring_control.glide_mode_enabled
                       || _autosoaring_control.thermal_mode_enabled;

    if (enabling) {
        _soaring_local_override       = true;
        _soaring_dds_inhibit_until_us = 0;       // no cooldown — DDS still allowed
        _soaring_forbidden_latched    = false;
        _autosoaring_last_recv_us     = 0;        // disable staleness watchdog
        _soaring_expect_set_mode      = true;     // next DO_SET_MODE is the paired mode switch
        // Reset debounce so first thermal DO_REPOSITION fires immediately
        _soaring_mode_cmd_last_us     = 0;
        _soaring_last_lat             = NAN;
        _soaring_last_lon             = NAN;
    } else {
        // soar:off
        _soaring_local_override       = false;
        _soaring_forbidden_latched    = false;
        _soaring_expect_set_mode      = false;
        _soaring_dds_inhibit_until_us = hrt_absolute_time() + 5_s;  // 5 s DDS cooldown
        _autosoaring_last_recv_us     = hrt_absolute_time();
        _tecs.set_gliding_mode_enabled(false);
    }
```

### 9.2 `vehicle_command_poll()` — DO_SET_MODE handler (mode-exit pairing)

**Problem solved:** `soar:glide` sends CUSTOM_0 + DO_SET_MODE back-to-back. Without pairing, the DO_SET_MODE would be misread as "user manually switching mode → exit soaring". With pairing, the first DO_SET_MODE after CUSTOM_0 is consumed silently; only a *standalone* `commander mode auto:mission` exits soaring.

```cpp
} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE) {
    if (_soaring_local_override) {
        if (_soaring_expect_set_mode) {
            _soaring_expect_set_mode = false;   // consume pair — soaring stays ON
        } else {
            // Standalone mode switch (e.g. "commander mode auto:mission") → exit soaring
            _autosoaring_control.glide_mode_enabled   = false;
            _autosoaring_control.thermal_mode_enabled = false;
            _soaring_local_override    = false;
            _soaring_forbidden_latched = false;
            _tecs.set_gliding_mode_enabled(false);
            PX4_INFO("Autosoaring: mode switch → soaring disabled, powered flight restored");
        }
    }
}
```

### 9.3 `Run()` — DDS poll (AutosoaringControl topic)

```cpp
autosoaring_control_s soaring_msg;

if (_autosoaring_control_sub.update(&soaring_msg)) {
    const hrt_abstime now = hrt_absolute_time();
    const bool dds_inhibited    = (_soaring_dds_inhibit_until_us > 0
                                   && now < _soaring_dds_inhibit_until_us);
    const bool dds_wants_soaring = soaring_msg.glide_mode_enabled
                                || soaring_msg.thermal_mode_enabled;

    if (dds_inhibited && dds_wants_soaring) {
        // soar:off cooldown: ignore DDS re-enable for 5 s
    } else {
        if (!dds_wants_soaring) {
            _soaring_dds_inhibit_until_us = 0;  // DDS turning off → cancel cooldown
        }

        if (!_soaring_local_override) {  // CLI wins over DDS
            const bool prev_thermal = _autosoaring_control.thermal_mode_enabled;
            _autosoaring_control      = soaring_msg;
            _autosoaring_last_recv_us = now;

            // Reset debounce on thermal activation so DO_REPOSITION fires immediately
            if (!prev_thermal && soaring_msg.thermal_mode_enabled) {
                _soaring_mode_cmd_last_us = 0;
                _soaring_last_lat         = NAN;
                _soaring_last_lon         = NAN;
            }
        }
    }
}
```

### 9.4 `Run()` — staleness watchdog

```cpp
// Companion silent > 2 s → disable soaring (DDS path only, CLI bypassed)
if (!_soaring_local_override &&
    _autosoaring_last_recv_us > 0 &&
    (hrt_absolute_time() - _autosoaring_last_recv_us) > 2_s) {
    if (_autosoaring_control.glide_mode_enabled || _autosoaring_control.thermal_mode_enabled) {
        PX4_WARN("Autosoaring: companion silent >2 s — disabling soaring");
        _autosoaring_control.glide_mode_enabled   = false;
        _autosoaring_control.thermal_mode_enabled = false;
    }
    _tecs.set_gliding_mode_enabled(false);
}
```

### 9.5 `control_auto()` — full soaring control block

```cpp
const float alt_min       = _param_fw_alt_min.get();
const float alt_max       = _param_fw_alt_max.get();
const float alt_hyst      = _param_fw_alt_hyst.get();
const float current_alt   = ...;

const bool glide_cmd      = _autosoaring_control.glide_mode_enabled;
const bool thermal_cmd    = _autosoaring_control.thermal_mode_enabled;
const bool soaring_req    = glide_cmd || thermal_cmd;

// --- Altitude floor (DDS only, CLI bypassed) ---
if (!_soaring_local_override && soaring_req && current_alt < alt_min) {
    if (!_soaring_forbidden_latched) {
        _soaring_forbidden_latched = true;
        PX4_WARN("Soaring disabled: alt %.0f below min %.0f m", current_alt, alt_min);
    }
}
if (!soaring_req) { _soaring_forbidden_latched = false; }  // clear latch on disable

// --- Altitude ceiling hysteresis ---
if (_alt_max_reached && current_alt < (alt_max - alt_hyst)) { _alt_max_reached = false; }
if (current_alt >= alt_max)                                  { _alt_max_reached = true; }

const bool soaring_allowed  = soaring_req && (_soaring_local_override || !_soaring_forbidden_latched);
bool effective_glide        = soaring_allowed && (glide_cmd || _alt_max_reached);
bool effective_thermal      = soaring_allowed && thermal_cmd && !_alt_max_reached;

const hrt_abstime now = hrt_absolute_time();

if (soaring_allowed) {

    if (effective_thermal) {
        // ── Thermal loiter: send DO_REPOSITION once, then pin the centre ──
        //
        // KEY FIX — moving loiter centre bug:
        // If DDS sends loiter_lat=0/lon=0 (meaning "loiter here"), curr_pos is used
        // for the first command but NEVER again. Without this fix, every second
        // a new DO_REPOSITION would fire with the updated curr_pos, making the loiter
        // centre chase the aircraft indefinitely.

        const bool dds_has_explicit_pos =
            PX4_ISFINITE(_autosoaring_control.loiter_lat) &&
            fabsf(_autosoaring_control.loiter_lat) > FLT_EPSILON;

        const double lat = dds_has_explicit_pos
                           ? (double)_autosoaring_control.loiter_lat : curr_pos(0);
        const double lon = dds_has_explicit_pos
                           ? (double)_autosoaring_control.loiter_lon : curr_pos(1);

        const bool first_activation   = !PX4_ISFINITE(_soaring_last_lat);
        const bool explicit_pos_moved = dds_has_explicit_pos &&
                                        (fabs(lat - _soaring_last_lat) > 1e-5 ||
                                         fabs(lon - _soaring_last_lon) > 1e-5);
        const bool pos_changed = first_activation || explicit_pos_moved;

        if (pos_changed) {
            vehicle_command_s cmd{};
            cmd.timestamp        = now;
            cmd.command          = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
            cmd.param1           = -1.f;   // keep current speed
            cmd.param2           = 1.f;    // REPOSITION_ACTION_NORMAL → switch to AUTO_LOITER
            cmd.param3           = (_autosoaring_control.loiter_radius_m > FLT_EPSILON)
                                   ? _autosoaring_control.loiter_radius_m : -1.0f;
            cmd.param4           = NAN;    // yaw unchanged
            cmd.param5           = lat;
            cmd.param6           = lon;
            cmd.param7           = current_alt;
            cmd.target_system    = 1;
            cmd.target_component = 1;
            _pub_vehicle_command.publish(cmd);
            _soaring_last_lat         = lat;
            _soaring_last_lon         = lon;
            _soaring_mode_cmd_last_us = now;
        }

    } else if (effective_glide && (now - _soaring_mode_cmd_last_us) > 1_s) {
        // ── Glide: ensure aircraft is in AUTO_MISSION (follow waypoints, engine off) ──
        vehicle_command_s cmd{};
        cmd.timestamp        = now;
        cmd.command          = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1           = 1.f;                                     // CUSTOM_MODE_ENABLED
        cmd.param2           = (float)PX4_CUSTOM_MAIN_MODE_AUTO;        // main mode (Commander reads as uint8)
        cmd.param3           = (float)PX4_CUSTOM_SUB_MODE_AUTO_MISSION; // sub mode
        cmd.target_system    = 1;
        cmd.target_component = 1;
        _pub_vehicle_command.publish(cmd);
        _soaring_mode_cmd_last_us = now;
    }
}

// ── TECS flag (single point of truth) ──
_tecs.set_gliding_mode_enabled(effective_glide || effective_thermal);

// ── Thermal-end auto-exit ──
// KEY FIX — stuck-in-HOLD bug:
// When thermal ends (DDS goes silent / soar:off / companion stops), nothing used
// to switch the nav mode back from AUTO_LOITER. UAV stayed in powered HOLD.
// Fix: detect the thermal→off transition and publish DO_SET_MODE(AUTO_MISSION).
//
// KEY FIX — wrong mode encoding:
// Using px4_custom_mode.data (packed uint32 as float) is WRONG. Commander reads:
//   param2 = custom_main_mode (uint8 cast of the float)
//   param3 = custom_sub_mode  (uint8 cast of the float)
// so params must be the raw integer values as floats, NOT a packed union.

if (_soaring_was_thermal && !effective_thermal && !effective_glide) {
    vehicle_command_s exit_cmd{};
    exit_cmd.timestamp        = now;
    exit_cmd.command          = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    exit_cmd.param1           = 1.f;
    exit_cmd.param2           = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
    exit_cmd.param3           = (float)PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
    exit_cmd.target_system    = 1;
    exit_cmd.target_component = 1;
    _pub_vehicle_command.publish(exit_cmd);
    _soaring_local_override    = false;
    _soaring_forbidden_latched = false;
    PX4_INFO("Autosoaring: thermal ended → AUTO_MISSION restored");
}

_soaring_was_thermal = effective_thermal;
```

### 9.6 Throttle actuator gate

**Why:** A second hard gate at the actuator level ensures T=0 even if any path above fails to set TECS correctly.

```cpp
} else if (_tecs.get_gliding_mode_enabled()) {
    _att_sp.thrust_body[0] = 0.0f;  // hard-zero throttle gate during soaring
}
```

---

## 10. `src/modules/navigator/navigator_main.cpp`

### 10.1 `DO_REPOSITION` — honour explicit loiter radius from param3

**Why:** Stock PX4 ignores `param3` in DO_REPOSITION. The soaring system uses it to pass the thermal orbit radius from `AutosoaringControl.loiter_radius_m`.

```cpp
// Honour explicit loiter radius in param3 (e.g. from AutosoaringControl via fw_pos_control).
// param3 > 0: use specified radius.  param3 <= 0 or NaN: use NAV_LOITER_RAD default.
if (PX4_ISFINITE(cmd.param3) && cmd.param3 > FLT_EPSILON) {
    rep->current.loiter_radius = cmd.param3;
} else if (!only_alt_change_requested) {
    rep->current.loiter_radius = get_loiter_radius();
}
```

### 10.2 `NAVIGATION_STATE_ORBIT` — map to loiter handler

**Why:** Some GCS/MAVLink stacks set `ORBIT` instead of `AUTO_LOITER` for circular flight. This prevents a silent no-op when the companion requests orbit mode.

```cpp
case vehicle_status_s::NAVIGATION_STATE_ORBIT:
    // Thermal loiter: reuse the loiter navigation mode.
    // The companion selects the centre via DO_REPOSITION;
    // we simply map this nav-state to the existing loiter handler.
    _pos_sp_triplet_published_invalid_once = false;
    navigation_mode_new = &_loiter;
    break;
```

---

## 11. `src/modules/commander/Commander.cpp`

### 11.1 New CLI mode shortcuts

```cpp
} else if (!strcmp(argv[1], "soar:glide")) {
    // Enable engine-off gliding in AUTO_MISSION.
    // Step 1: tell fw_pos_control to enable TECS gliding mode.
    // Step 2: switch nav mode to AUTO_MISSION (waypoint following while gliding).
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_CUSTOM_0, 1.0f, 0.0f);
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1,
                         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION);

} else if (!strcmp(argv[1], "soar:thermal")) {
    // Enable engine-off thermal loiter.
    // Step 1: tell fw_pos_control to enable TECS gliding mode.
    // Step 2: switch nav mode to AUTO_LOITER (orbit thermal centre).
    // Loiter centre set by subsequent DO_REPOSITION from control_auto().
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_CUSTOM_0, 0.0f, 1.0f);
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1,
                         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_LOITER);

} else if (!strcmp(argv[1], "soar:off")) {
    // Disable soaring, restore powered AUTO_MISSION.
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_CUSTOM_0, 0.0f, 0.0f);
    send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1,
                         PX4_CUSTOM_MAIN_MODE_AUTO, PX4_CUSTOM_SUB_MODE_AUTO_MISSION);
}
```

**CLI usage:**
```
pxh> commander mode soar:glide    # engine off, follow mission waypoints
pxh> commander mode soar:thermal  # engine off, orbit at current position
pxh> commander mode soar:off      # restore powered flight + AUTO_MISSION
pxh> commander mode auto:mission  # also exits soaring (standalone mode switch)
```

---

## 12. `src/modules/uxrce_dds_client/dds_topics.yaml`

```yaml
subscriptions:
  ...
  - topic: /fmu/in/autosoaring_control
    type: px4_msgs::msg::AutosoaringControl
```

This makes the `/fmu/in/autosoaring_control` ROS2 topic available. ROS2 nodes publish to it; the XRCE-DDS client forwards it as the `autosoaring_control` uORB topic inside PX4.

---

## Bug Fixes Log

### Bug 1 — `commander mode soar:glide` did nothing
**Root cause:** Initial implementation only switched the nav mode but did not have a mechanism to enable TECS gliding mode from CLI.  
**Fix:** Added `VEHICLE_CMD_CUSTOM_0` as the CLI soaring command. It sets `_autosoaring_control` flags directly in `vehicle_command_poll()` without requiring DDS.

### Bug 2 — Parameter name too long (`FW_SOAR_INTEG_TAU` = 17 chars, limit = 16)
**Fix:** Renamed to `FW_GLIDE_I_DECAY` (16 chars) throughout all files.

### Bug 3 — `VEHICLE_CMD_USER_1` not a valid PX4 constant
**Fix:** Replaced with `VEHICLE_CMD_CUSTOM_0` which is a pre-defined PX4 user command.

### Bug 4 — `commander mode auto:mission` did not exit soaring (throttle stayed 0)
**Root cause:** `soar:glide` sends CUSTOM_0 + DO_SET_MODE back-to-back. Without pairing, the DO_SET_MODE was either treated as a soaring exit or ignored.  
**Fix:** `_soaring_expect_set_mode` flag pairs the two commands. A DO_SET_MODE arriving while `_soaring_expect_set_mode=true` is consumed silently (soaring stays on). A standalone `commander mode auto:mission` finds `_soaring_expect_set_mode=false` and correctly exits soaring.

### Bug 5 — `soar:off` from CLI did not stop DDS publications; after CLI off, DDS could not re-enable
**Root cause:** `_soaring_dds_inhibited` was a boolean that never cleared if DDS kept streaming `thermal=true`.  
**Fix:** Replaced with timestamp `_soaring_dds_inhibit_until_us`. After `soar:off`, DDS re-enable is blocked for 5 seconds only. After 5 s, DDS regains authority automatically. DDS publishing `glide=false, thermal=false` cancels the cooldown immediately.

### Bug 6 — Thermal loiter centre moved with aircraft (DDS with `loiter_lat=0, loiter_lon=0`)
**Root cause:** The DO_REPOSITION debounce (1 s) expired after each cycle. Since `loiter_lat=0` fell back to `curr_pos` (which changes as the aircraft flies), a new DO_REPOSITION was sent every second with the updated aircraft position → loiter centre chased the aircraft.  
**Fix:** `pos_changed` now only evaluates as true for:  
  - First activation (NAN guard) — sends once, pins centre at activation point.  
  - DDS providing explicit non-zero coordinates that changed by > 1e-5°.  
  When DDS sends `lat=0, lon=0`, `pos_changed` is permanently false after the first command.

### Bug 7 — UAV stuck in powered HOLD after thermal ends (not in powered MISSION)
**Root cause 1:** Nothing published a mode switch to AUTO_MISSION when thermal ended.  
**Root cause 2:** The DO_SET_MODE encoding was wrong. The code used `px4_custom_mode.data` (packed uint32 cast to float in param2), but Commander reads `param2` as main_mode (uint8) and `param3` as sub_mode (uint8) **separately**. The packed value decoded to garbage → Commander ignored the command → UAV stayed in HOLD.  
**Fix:**  
  1. Added `_soaring_was_thermal` flag to detect the `thermal: true → false` transition.  
  2. On that transition, publish `DO_SET_MODE` with correct separate params: `param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO`, `param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_MISSION`.  
  3. Same fix applied to the glide-activation block (which previously used the non-standard `VEHICLE_CMD_SET_NAV_STATE`).

### Bug 8 — Thermal did not start immediately (debounce blocked first command)
**Root cause:** `_soaring_mode_cmd_last_us` was shared between the glide block and thermal block. If glide was recently active, the thermal's first DO_REPOSITION was blocked for up to 1 second by the debounce.  
**Fix:** Reset `_soaring_mode_cmd_last_us = 0` and `_soaring_last_lat/lon = NAN` whenever soaring is enabled (in CUSTOM_0 handler for CLI, in DDS poll for ROS2 path). This forces immediate DO_REPOSITION on the first control cycle.

---

## Control Flow Summary

### Glide mode (engine off, follow mission)

```
CLI: commander mode soar:glide
  → CUSTOM_0(p1=1,p2=0)    → fw_pos_control: glide_cmd=true, local_override=true
  → DO_SET_MODE(MISSION)    → Commander: nav→AUTO_MISSION
  → control_auto():
      effective_glide=true
      → DO_SET_MODE(MISSION) every 1 s (ensures mode stays correct)
      → TECS.gliding_mode_enabled=true
          • pitch = speed-on-elevator (ske_weighting=2)
          • throttle = 0 (hard idle)
          • airspeed setpoint = FW_GLIDE_AIRSPD

Exit:  commander mode auto:mission  (standalone DO_SET_MODE)
  → CUSTOM_0 handler: expect_set_mode=false → sees standalone → clears soaring
  → TECS.gliding_mode_enabled=false → normal powered flight
```

### Thermal mode (engine off, orbit thermal centre)

```
DDS: thermal_mode_enabled=true, loiter_lat=0, loiter_lon=0
  → DDS poll: prev_thermal=false → reset debounce, _soaring_last_lat=NAN
  → control_auto():
      effective_thermal=true
      first_activation=true (NAN guard)
      → DO_REPOSITION(param2=1, lat=curr_pos, lon=curr_pos, radius=-1)
          Commander: nav→AUTO_LOITER
          Navigator: loiter centre = aircraft position at activation
      _soaring_last_lat = lat  (pinned — never updated again for lat=0 messages)
      → TECS.gliding_mode_enabled=true

Thermal update with new explicit position:
  DDS: thermal_mode_enabled=true, loiter_lat=47.3977, loiter_lon=8.5456
  → explicit_pos_moved=true → new DO_REPOSITION with new coordinates → loiter centre moves

Exit: DDS publishes thermal=false (or staleness watchdog fires)
  → _autosoaring_control.thermal_mode_enabled=false
  → control_auto(): _soaring_was_thermal=true, effective_thermal=false
      → DO_SET_MODE(param2=PX4_CUSTOM_MAIN_MODE_AUTO, param3=PX4_CUSTOM_SUB_MODE_AUTO_MISSION)
          Commander: nav→AUTO_MISSION
      _soaring_was_thermal = false
      → TECS.gliding_mode_enabled=false → normal powered flight
```

### Authority hierarchy

```
Priority 1 (highest): CLI  soar:off  → blocks DDS for 5 s, clears all flags
Priority 2:           CLI  soar:glide / soar:thermal → local_override=true, DDS still allowed
Priority 3:           DDS  AutosoaringControl → authoritative when no CLI override
Priority 4 (auto):    Staleness watchdog → disables soaring if DDS silent > 2 s
```

---

## Build Instructions

```bash
cd /home/radhouene/Pictures/PX4-Autopilot

# Build firmware
make px4_sitl_default

# Run simulation with advanced plane
make px4_sitl gz_advanced_plane

# In PX4 shell — test glide:
commander mode soar:glide
commander mode auto:mission    # exit soaring

# Test thermal at current position:
commander mode soar:thermal
commander mode soar:off        # exit soaring
```

---

## Scientific Justification Summary

| Feature | Justification |
|---------|--------------|
| `spe=0, ske=2` in glide | Speed-on-elevator control (Langelaan 2007, Andersson 2011). Pitch is the only actuator; it must control airspeed, not altitude. |
| Hard throttle = 0 | Engine-off gliding requires zero thrust. Two-layer gate (TECS + actuator) prevents any residual thrust. |
| Integrator decay `exp(-t/τ)` | Prevents throttle surge on engine restart. Parametric τ allows tuning per airframe. |
| Altitude rate clip ≤ 0 | A glider cannot climb. Clipping prevents TECS from commanding an impossible climb. |
| Best-glide airspeed override | V_bg minimises glide angle (drag polar minimum). Physics-based override, not heuristic. |
| Fast-descend disabled | Conflicts with speed-on-elevator weighting. Cannot coexist physically. |
| Altitude floor/ceiling | Safety: companion computer may lose situational awareness. Hard limits prevent dangerous manoeuvres near ground or above safe altitude. |
| Staleness watchdog (2 s) | Communication reliability: if companion is silent, the assumption of "glide is safe" is invalid. Fail-safe to powered flight. |
| Loiter centre pinning | Thermal centre is a physical location in the atmosphere. Once identified, it should not move with the aircraft. |
