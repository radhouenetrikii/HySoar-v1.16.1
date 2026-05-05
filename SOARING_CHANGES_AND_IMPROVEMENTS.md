# PX4 v1.16.1 — Soaring Feature: Changes Applied & Suggested Improvements

**Target repository:** `~/Pictures/PX4-Autopilot` (clean v1.16.1 tag)
**Applied from:** `PX4_vs_FV_TEST_LINE_BY_LINE.md`
**Date:** April 2026

---

## Table of Contents

1. [Summary of all changed files](#1-summary-of-all-changed-files)
2. [File-by-file line changes](#2-file-by-file-line-changes)
3. [New files created](#3-new-files-created)
4. [Skipped changes and reasons](#4-skipped-changes-and-reasons)
5. [Suggested improvements](#5-suggested-improvements)

---

## 1. Summary of all changed files

| # | File | Lines changed | Type |
|---|------|---------------|------|
| 1 | `CMakeLists.txt` | 1 line modified | C++ standard bump |
| 2 | `.gitignore` | +12 lines | Artifact ignores |
| 3 | `ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_advanced_plane` | 1 line modified | Airframe param |
| 4 | `msg/CMakeLists.txt` | +1 line | uORB message registration |
| 5 | `msg/AutosoaringControl.msg` | **NEW FILE** (+13 lines) | uORB message definition |
| 6 | `src/lib/tecs/TECS.hpp` | +12 lines, 2 signatures changed | TECS gliding mode API |
| 7 | `src/lib/tecs/TECS.cpp` | +86 lines | TECS gliding mode logic |
| 8 | `src/modules/fw_pos_control/FixedwingPositionControl.hpp` | +24 lines | Subscriptions, params, state |
| 9 | `src/modules/fw_pos_control/FixedwingPositionControl.cpp` | +323 lines, -72 lines | Main soaring control block |
| 10 | `src/modules/fw_pos_control/fw_path_navigation_params.c` | +64 lines | 4 new parameters |
| 11 | `src/modules/navigator/navigator_main.cpp` | +19 lines | DO_REPOSITION radius + ORBIT case |
| 12 | `src/modules/commander/Commander.cpp` | +55 lines | CLI shortcuts + DO_REPOSITION fix |
| 13 | `src/modules/uxrce_dds_client/dds_topics.yaml` | +4 lines | DDS subscription |
| 14 | `Tools/simulation/gz/models/advanced_plane/model.sdf` | 1 line modified | Plugin filename |

**Total:** 532 insertions, 72 deletions across 13 modified files + 1 new file

---

## 2. File-by-file line changes

---

### File 1 — `CMakeLists.txt` (root)

**Location:** line 270

```diff
-set(CMAKE_CXX_STANDARD 14)
+set(CMAKE_CXX_STANDARD 17)  # Required by std::optional and structured bindings used in Gazebo plugins
```

**Why:** C++17 is required by `std::optional`, structured bindings, and other features
used in the custom Gazebo lift-drag plugin.

---

### File 2 — `.gitignore`

**Location:** appended at end of file

```diff
+# Gazebo msgs build outputs / generated artifacts
+Tools/simulation/gz/GZ_Msgs/build/
+
+# Shared libraries from builds
+**/*.so
+**/*.so.*
+
+# Protobuf generated code (enable if you do NOT want to commit generated outputs)
+**/*.pb.cc
+**/*.pb.h
+**/*_pb2.py
```

**Why:** Prevents committing compiled Gazebo plugins, shared libraries, and
protobuf-generated C++/Python files.

---

### File 3 — `ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_advanced_plane`

**Location:** line 36

```diff
-param set-default FW_THR_MIN 0.05
+param set-default FW_THR_MIN 0.00  # 0.00 enables true engine-off gliding
```

**Why:** PX4's default minimum throttle of 5% prevents the motor from fully shutting
off. Setting it to 0% allows true engine-off gliding in soaring mode.

---

### File 4 — `msg/CMakeLists.txt`

**Location:** after `ActuatorArmed.msg` (line 42)

```diff
     ActuatorArmed.msg
+    AutosoaringControl.msg
     ActuatorControlsStatus.msg
```

**Why:** Registers the new message for uORB code generation.

---

### File 5 — `msg/AutosoaringControl.msg` *(NEW FILE)*

**Full content:**

```
uint64  timestamp               # time since system start (microseconds)

bool    glide_mode_enabled      # True: throttle=0, follow mission waypoints (glide)
bool    thermal_mode_enabled    # True: throttle=0, loiter at thermal center or current position

float32 loiter_radius_m         # Loiter radius [m]; 0 = use NAV_LOITER_RAD default
float32 loiter_lat              # Thermal center latitude  [deg]; 0 = use current position
float32 loiter_lon              # Thermal center longitude [deg]; 0 = use current position
```

**Why:** Defines the uORB message published by the ROS2 companion computer node
(`autosoaring_app`) and bridged into the FMU via XRCE-DDS on topic
`/fmu/in/autosoaring_control`.

---

### File 6 — `src/lib/tecs/TECS.hpp`

#### 6a. `Param` struct — new field (line ~238)

```diff
     float load_factor;
+    float gliding_airspeed_setpoint{15.0f};  ///< Fixed airspeed setpoint for gliding mode [m/s]
     float fast_descend;
```

#### 6b. `Flag` struct — new field (line ~285)

```diff
     bool airspeed_enabled;
     bool detect_underspeed_enabled;
+    bool gliding_mode_enabled{false};  ///< Flag if gliding mode is enabled for soaring
```

#### 6c. `_calculateTotalEnergyRateLimit` declaration (line ~397)

```diff
-STERateLimit _calculateTotalEnergyRateLimit(const Param &param) const;
+STERateLimit _calculateTotalEnergyRateLimit(const Param &param, const Flag &flag) const;
```

#### 6d. `_calcPitchControlUpdate` declaration (line ~474)

```diff
-void _calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate, const Param &param);
+void _calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate, const Param &param, const Flag &flag);
```

#### 6e. Public gliding mode setters (after `set_detect_underspeed_enabled`)

```diff
+void set_gliding_mode_enabled(bool enabled) { _control_flag.gliding_mode_enabled = enabled; }
+bool get_gliding_mode_enabled() const       { return _control_flag.gliding_mode_enabled; }
```

#### 6f. Gliding airspeed setter (after `set_seb_rate_ff_gain`)

```diff
+void set_gliding_airspeed_setpoint(float airspeed) { _control_param.gliding_airspeed_setpoint = airspeed; };
```

#### 6g. `_control_flag` initializer (line ~757)

```diff
 TECSControl::Flag _control_flag{
     .airspeed_enabled = false,
     .detect_underspeed_enabled = false,
+    .gliding_mode_enabled = false,
 };
```

---

### File 7 — `src/lib/tecs/TECS.cpp`

#### 7a. `_calculateTotalEnergyRateLimit` — definition signature

```diff
-TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Param &param) const
+TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Param &param, const Flag &flag) const
```

#### 7b. All 3 call-sites of `_calculateTotalEnergyRateLimit` (lines ~249, ~320, ~517)

```diff
-const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};
+const STERateLimit limit{_calculateTotalEnergyRateLimit(param, flag)};
```
*(Applied at all 3 occurrences)*

#### 7c. `_calcAirspeedControlState` — refactor for readability (line ~327)

```diff
-const float max_tas_rate_sp = (param.fast_descend * 0.5f + 0.5f) * limit.STE_rate_max
-                               / math::max(input.tas, FLT_EPSILON);
-const float min_tas_rate_sp = (param.fast_descend * 0.5f + 0.5f) * limit.STE_rate_min
-                               / math::max(input.tas, FLT_EPSILON);
+// Use original airspeed control logic — enables natural thermal soaring response
+const float v  = math::max(input.tas, FLT_EPSILON);
+const float fd = (param.fast_descend * 0.5f + 0.5f);
+const float max_tas_rate_sp = fd * limit.STE_rate_max / v;
+const float min_tas_rate_sp = fd * limit.STE_rate_min / v;
```

#### 7d. `_updateSpeedAltitudeWeights` — gliding mode early return (line ~391)

```diff
 SpecificEnergyWeighting weight;
+
+// Gliding mode: prioritize kinetic energy (speed) over potential energy (altitude)
+if (flag.gliding_mode_enabled) {
+    weight.spe_weighting = 0.0f;  // no altitude error correction while gliding
+    weight.ske_weighting = 2.0f;  // full speed control
+    return weight;                // early return — skip normal weighting
+}
+
 // Calculate the weight applied to control of specific kinetic energy error
```

#### 7e. `_calcPitchControl` — gliding mode block + updated call (line ~423)

```diff
 void TECSControl::_calcPitchControl(...)
 {
+    // Gliding mode: run TECS pitch control with speed-priority energy weights, then early-return
+    if (flag.gliding_mode_enabled) {
+        const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
+        ControlValues seb_rate{_calcPitchControlSebRate(weight, specific_energy_rates)};
+        _calcPitchControlUpdate(dt, input, seb_rate, param, flag);
+        const float pitch_setpoint{_calcPitchControlOutput(input, seb_rate, param, flag)};
+        const float pitch_increment = dt * param.vert_accel_limit / math::max(input.tas, FLT_EPSILON);
+        _pitch_setpoint = constrain(pitch_setpoint, _pitch_setpoint - pitch_increment,
+                                    _pitch_setpoint + pitch_increment);
+        _pitch_setpoint = constrain(_pitch_setpoint, param.pitch_min, param.pitch_max);
+        _debug_output.energy_balance_rate_estimate = seb_rate.estimate;
+        _debug_output.energy_balance_rate_sp       = seb_rate.setpoint;
+        _debug_output.pitch_integrator             = _pitch_integ_state;
+        return;  // skip the standard pitch control path below
+    }
+
+    // Normal TECS pitch control
     const SpecificEnergyWeighting weight{...};
     ControlValues seb_rate{...};
-    _calcPitchControlUpdate(dt, input, seb_rate, param);
+    _calcPitchControlUpdate(dt, input, seb_rate, param, flag);
```

#### 7f. `_calcPitchControlUpdate` — signature change (line ~460)

```diff
 void TECSControl::_calcPitchControlUpdate(float dt, const Input &input,
-        const ControlValues &seb_rate, const Param &param)
+        const ControlValues &seb_rate, const Param &param, const Flag &flag)
```

#### 7g. `_calcThrottleControlUpdate` — integrator decay in glide mode (line ~570)

```diff
 void TECSControl::_calcThrottleControlUpdate(...)
 {
+    // Gliding mode: prevent integrator windup with a gentle 10 %/s exponential decay
+    if (flag.gliding_mode_enabled) {
+        _throttle_integ_state -= dt * 0.1f * _throttle_integ_state;
+        return;
+    }
     // Calculate gain scaler ...
```

#### 7h. `_calcThrottleControlOutput` — hard idle throttle in glide mode (line ~602)

```diff
 float TECSControl::_calcThrottleControlOutput(...) const
 {
+    // Gliding mode: hard idle — never let TECS add thrust
+    if (flag.gliding_mode_enabled) {
+        return param.throttle_min;  // returns FW_THR_MIN (0.00 in the advanced_plane airframe)
+    }
     // Calculate gain scaler ...
```

#### 7i. `TECS::calcTrueAirspeedSetpoint` — fixed glide speed (line ~678)

```diff
 float TECS::calcTrueAirspeedSetpoint(float eas_to_tas, float eas_setpoint)
 {
+    // Gliding mode: ignore mission airspeed; use FW_GLIDE_AIRSPD parameter
+    if (_control_flag.gliding_mode_enabled) {
+        return _control_param.gliding_airspeed_setpoint;
+    }
     return lerp(eas_to_tas * eas_setpoint, _control_param.tas_max, _fast_descend);
```

#### 7j. `TECS::update` — altitude rate clipping (after `tas_setpoint` assignment, line ~760)

```diff
     control_setpoint.tas_setpoint = calcTrueAirspeedSetpoint(eas_to_tas, EAS_setpoint);
+
+    // Gliding mode: clip altitude-rate setpoints to zero to prevent commanded climbs
+    if (_control_flag.gliding_mode_enabled) {
+        if (PX4_ISFINITE(control_setpoint.altitude_reference.alt_rate))
+            control_setpoint.altitude_reference.alt_rate =
+                math::min(control_setpoint.altitude_reference.alt_rate, 0.0f);
+        if (PX4_ISFINITE(control_setpoint.altitude_rate_setpoint_direct))
+            control_setpoint.altitude_rate_setpoint_direct =
+                math::min(control_setpoint.altitude_rate_setpoint_direct, 0.0f);
+    }
```

#### 7k. `TECS::_setFastDescend` — disable fast-descend during gliding (line ~782)

```diff
 void TECS::_setFastDescend(const float alt_setpoint, const float alt)
 {
+    // Gliding mode: disable fast-descend — conflicts with glide energy management
+    if (_control_flag.gliding_mode_enabled) {
+        _fast_descend = 0.0f;
+        _enabled_fast_descend_timestamp = 0U;
+        return;
+    }
     if (_control_flag.airspeed_enabled && ...
```

---

### File 8 — `src/modules/fw_pos_control/FixedwingPositionControl.hpp`

#### 8a. Include (line ~75)

```diff
 #include <uORB/topics/airspeed_validated.h>
+#include <uORB/topics/autosoaring_control.h>   // ROS2 soaring command topic (via XRCE-DDS)
```

#### 8b. Subscription members (after `_airspeed_validated_sub`, line ~206)

```diff
+uORB::Subscription    _autosoaring_control_sub{ORB_ID(autosoaring_control)};
+autosoaring_control_s _autosoaring_control{};        // last received message
+hrt_abstime  _autosoaring_last_recv_us{0};            // timestamp for 2-second staleness check
+hrt_abstime  _alt_safety_warn_last_us{0};             // rate-limiter for altitude-safety warnings
```

#### 8c. Publication member (after `_flight_phase_estimation_pub`, line ~232)

```diff
+uORB::Publication<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};
```

#### 8d. Flag member (after `_tecs_is_running`, line ~406)

```diff
+bool _in_free_glide{false};   // true when soaring (glide or thermal) is active
```

#### 8e. DEFINE_PARAMETERS — 4 new soaring parameters (end of block, line ~1055)

```diff
     (ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on,
+
+    // ── Soaring altitude safety limits ──
+    (ParamFloat<px4::params::FW_GLIDE_AIRSPD>) _param_fw_glide_airspd,
+    (ParamFloat<px4::params::FW_ALT_MIN>)      _param_fw_alt_min,
+    (ParamFloat<px4::params::FW_ALT_MAX>)      _param_fw_alt_max,
+    (ParamFloat<px4::params::FW_ALT_HYST>)     _param_fw_alt_hyst
 )
```

#### 8f. Soaring state members (after DEFINE_PARAMETERS block)

```diff
+bool        _alt_max_reached{false};
+bool        _soaring_forbidden_latched{false};
+hrt_abstime _soaring_mode_cmd_last_us{0};
+double      _soaring_last_lat{(double)NAN};
+double      _soaring_last_lon{(double)NAN};
```

---

### File 9 — `src/modules/fw_pos_control/FixedwingPositionControl.cpp`

#### 9a. Includes (after `events.h`, line ~35)

```diff
+#include <parameters/param.h>
+#include <commander/px4_custom_mode.h>
```

#### 9b. `updateParams()` — wire up gliding airspeed (line ~139)

```diff
 _tecs.set_integrator_gain_pitch(_param_fw_t_I_gain_pit.get());
+_tecs.set_gliding_airspeed_setpoint(_param_fw_glide_airspd.get());
 _tecs.set_throttle_slewrate(_param_fw_thr_slew_max.get());
```

#### 9c. All 5 `pos_sp_curr.gliding_enabled` blocks replaced (lines ~1077, ~1170, ~1254, ~1381, ~1436)

```diff
-if (pos_sp_curr.gliding_enabled) {
-    /* enable gliding with this waypoint */
-    _tecs.set_speed_weight(2.0f);
-    tecs_fw_thr_min = 0.0;
-    tecs_fw_thr_max = 0.0;
-} else {
-    tecs_fw_thr_min = _param_fw_thr_min.get();
-    tecs_fw_thr_max = _param_fw_thr_max.get();
-}
+// Gliding mode is now handled internally by TECS (set_gliding_mode_enabled).
+tecs_fw_thr_min = _param_fw_thr_min.get();
+tecs_fw_thr_max = _param_fw_thr_max.get();
```

*(Replaced at all 5 call sites: `navigateWaypoint`, `control_auto_velocity`,
`navigateLoiter` x2, `control_auto_path`)*

#### 9d. `control_auto()` — full soaring control block (inserted before throttle gate, line ~929)

This is the primary 185-line block. It contains:

- **Altitude safety with hysteresis:**
  - If `_current_altitude <= FW_ALT_MIN` → latch soaring forbidden, log warning
  - If `_current_altitude >= FW_ALT_MAX` → force glide mode (exit thermal)
  - If altitude drops back below `FW_ALT_MAX - FW_ALT_HYST` → re-allow thermal

- **GLIDE MODE** (`glide_mode_enabled=true, thermal_mode_enabled=false`):
  - Sends `VEHICLE_CMD_DO_SET_MODE` → `AUTO_MISSION` (1-second debounce)
  - Calls `_tecs.set_gliding_mode_enabled(true)`

- **THERMAL MODE** (`thermal_mode_enabled=true`):
  - If thermal center provided: sends `VEHICLE_CMD_DO_REPOSITION` to lat/lon
  - If no center: sends `DO_REPOSITION` to current position
  - Center-change detection: only resends if center moves > 1e-6 degrees
  - Calls `_tecs.set_gliding_mode_enabled(true)`

- **SOARING OFF** (both false):
  - If previously in loiter: sends `DO_SET_MODE` → `AUTO_MISSION`
  - Calls `_tecs.set_gliding_mode_enabled(false)`

- **Modified throttle gate:**
```diff
-if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
+if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_IDLE
+    || mission_manual_throttle || freeclimb_mode) {
+    // Soaring modes: hard-zero throttle regardless of TECS output
     _att_sp.thrust_body[0] = 0.0f;
 } else {
     _att_sp.thrust_body[0] = (_landed) ? min(_param_fw_thr_idle.get(), 1.f) : get_tecs_thrust();
```

#### 9e. `get_tecs_thrust()` — vehicle status refresh (line ~2444)

```diff
 float FixedwingPositionControl::get_tecs_thrust()
 {
+    // Refresh vehicle_status so nav_state is current when callers query it
+    _vehicle_status_sub.update(&_vehicle_status);
+
+    // Note: gliding-mode throttle zeroing handled internally by TECS and control_auto()
     if (_tecs_is_running) {
         return min(_tecs.get_throttle_setpoint(), 1.f);
```

#### 9f. `Run()` — AutosoaringControl poll with staleness watchdog (after `wind_poll()`, line ~2613)

```diff
+// ── AutosoaringControl from ROS2 companion (via XRCE-DDS) ────────────
+if (_autosoaring_control_sub.update(&_autosoaring_control)) {
+    _autosoaring_last_recv_us = hrt_absolute_time();
+}
+
+// 2-second staleness watchdog: if companion stops publishing, clear all soaring flags
+if ((hrt_absolute_time() - _autosoaring_last_recv_us) > 2_s) {
+    _autosoaring_control.glide_mode_enabled   = false;
+    _autosoaring_control.thermal_mode_enabled = false;
+    _autosoaring_control.loiter_radius_m      = 0.0f;
+    _autosoaring_control.loiter_lat           = 0.0f;
+    _autosoaring_control.loiter_lon           = 0.0f;
+}
```

#### 9g. `tecs_update_pitch_throttle()` — TECS glide flag mirror (after `_tecs_is_running = true`)

```diff
+// ── Soaring TECS flag (mirrors altitude-safety logic from control_auto) ──
+bool tecs_glide   = _autosoaring_control.glide_mode_enabled;
+bool tecs_thermal = _autosoaring_control.thermal_mode_enabled;
+
+if (tecs_glide || tecs_thermal) {
+    bool triggered = (_current_altitude <= _param_fw_alt_min.get());
+    const float alt_max = _param_fw_alt_max.get();
+    if (_current_altitude >= alt_max && !_alt_max_reached) _alt_max_reached = true;
+    if (_current_altitude < alt_max - _param_fw_alt_hyst.get() && _alt_max_reached)
+        _alt_max_reached = false;
+    if (triggered) { tecs_glide = false; tecs_thermal = false; }
+    else if (_alt_max_reached && tecs_thermal) { tecs_glide = true; tecs_thermal = false; }
+}
+if (_soaring_forbidden_latched) { tecs_glide = false; tecs_thermal = false; }
+_tecs.set_gliding_mode_enabled((tecs_glide && !tecs_thermal) || tecs_thermal);
```

---

### File 10 — `src/modules/fw_pos_control/fw_path_navigation_params.c`

**Location:** appended at end of file

Four new parameters added with full doxygen documentation:

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `FW_ALT_MIN` | 100.0 m | 50–1000 m | Minimum altitude; soaring latched off below this |
| `FW_ALT_MAX` | 500.0 m | 100–2000 m | Maximum altitude; thermal→glide transition |
| `FW_ALT_HYST` | 20.0 m | 10–100 m | Hysteresis band below FW_ALT_MAX |
| `FW_GLIDE_AIRSPD` | 10.0 m/s | 5–50 m/s | Fixed airspeed setpoint during glide mode |

---

### File 11 — `src/modules/navigator/navigator_main.cpp`

#### 11a. DO_REPOSITION handler — loiter radius from `param3` (after lat/lon block, line ~395)

```diff
+// Honour param3 as an explicit loiter radius (used by soaring thermal mode).
+if (PX4_ISFINITE(cmd.param3) && cmd.param3 > FLT_EPSILON) {
+    rep->current.loiter_radius = fabsf(cmd.param3);
+} else if (PX4_ISFINITE(curr->current.loiter_radius) && curr->current.loiter_radius > FLT_EPSILON) {
+    rep->current.loiter_radius = curr->current.loiter_radius;
+} else {
+    rep->current.loiter_radius = get_loiter_radius();
+}
```

#### 11b. Navigation state switch — ORBIT case (before `NAVIGATION_STATE_MANUAL`, line ~828)

```diff
+case vehicle_status_s::NAVIGATION_STATE_ORBIT:
+    // Treat ORBIT as loiter so navigator publishes a valid position setpoint triplet.
+    _pos_sp_triplet_published_invalid_once = false;
+    navigation_mode_new = &_loiter;
+    break;
+
 case vehicle_status_s::NAVIGATION_STATE_MANUAL:
```

---

### File 12 — `src/modules/commander/Commander.cpp`

#### 12a. Mode CLI handler — add `ext2`–`ext8`, `thermal`, `gliding` shortcuts (after `ext1` block, line ~420)

```diff
+} else if (!strcmp(argv[1], "ext2")) {
+    send_vehicle_command(..., PX4_CUSTOM_SUB_MODE_EXTERNAL2);
+} else if (!strcmp(argv[1], "ext3")) { ... EXTERNAL3 ... }
+} else if (!strcmp(argv[1], "ext4")) { ... EXTERNAL4 ... }
+} else if (!strcmp(argv[1], "ext5")) { ... EXTERNAL5 ... }
+} else if (!strcmp(argv[1], "ext6")) { ... EXTERNAL6 ... }
+} else if (!strcmp(argv[1], "ext7")) { ... EXTERNAL7 ... }
+} else if (!strcmp(argv[1], "ext8")) { ... EXTERNAL8 ... }
+} else if (!strcmp(argv[1], "thermal")) {
+    // Switch to loiter; companion enables thermal via AutosoaringControl uORB
+    send_vehicle_command(..., PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
+    PX4_INFO("SOARING: switched to loiter — enable thermal via companion");
+} else if (!strcmp(argv[1], "gliding")) {
+    // Switch to mission; companion enables glide via AutosoaringControl uORB
+    send_vehicle_command(..., PX4_CUSTOM_SUB_MODE_AUTO_MISSION);
+    PX4_INFO("SOARING: switched to mission — enable glide via companion");
```

#### 12b. DO_REPOSITION param2 fix (line ~751)

```diff
-const bool mode_switch_not_requested = (change_mode_flags & 1) == 0;
+const bool mode_switch_requested      = (change_mode_flags & 1) != 0;
 const bool unsupported_bits_set       = (change_mode_flags & ~1) != 0;

-if (mode_switch_not_requested || unsupported_bits_set) {
+if (unsupported_bits_set) {
     answer_command(cmd, VEHICLE_CMD_RESULT_UNSUPPORTED);
-} else {
+} else if (mode_switch_requested) {
     if (_user_mode_intention.change(...AUTO_LOITER...))
         cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;
     else
         cmd_result = VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
+} else {
+    // param2 bit 0 = 0: position-only update (thermal center move), no mode switch
+    cmd_result = VEHICLE_CMD_RESULT_ACCEPTED;
```

**Why:** The original code treated `param2=0` (position-only update) as UNSUPPORTED,
preventing the thermal center from being updated while already in loiter mode.

---

### File 13 — `src/modules/uxrce_dds_client/dds_topics.yaml`

**Location:** beginning of `subscriptions:` block (line ~84)

```diff
 subscriptions:
+  # AutosoaringControl: published by a ROS2 companion node to command glide / thermal modes
+  - topic: /fmu/in/autosoaring_control
+    type: px4_msgs::msg::AutosoaringControl
+
   - topic: /fmu/in/register_ext_component_request
```

---

### File 14 — `Tools/simulation/gz/models/advanced_plane/model.sdf`

**Location:** LiftDrag plugin block (line ~607)

```diff
-<plugin filename="gz-sim-advanced-lift-drag-system"
-        name="gz::sim::systems::AdvancedLiftDrag">
+<!-- Custom advanced lift-drag plugin with thermal soaring aerodynamics -->
+<plugin filename="libLiftDragAdvanced1.so"
+        name="gz::sim::systems::AdvancedLiftDrag1">
```

**Why:** Switches from the Gazebo built-in plugin to the locally-built custom plugin
that models the advanced aerodynamics needed for thermal soaring simulation.

---

## 3. New files created

| File | Lines | Purpose |
|------|-------|---------|
| `msg/AutosoaringControl.msg` | 13 | uORB message for ROS2 → FMU soaring commands |

---

## 4. Skipped changes and reasons

| Section | File | Reason skipped |
|---------|------|----------------|
| `navigator_params.c` `NAV_LOITER_RAD @max` | Already at `@max 1000` in v1.16.1 (exceeds the 600.0 target) |
| `navigator.h` soaring include | `soaring_modes.hpp` was never part of standard v1.16.1 |
| `navigator/CMakeLists.txt` | `soaring_modes.cpp` was never part of standard v1.16.1 |
| `rc_cessna/model.sdf` | Already clean — no hardcoded `/home/radhouene/plugin/...` paths present |
| `commander/px4_custom_mode.h` | No changes needed per the diff document |

---

## 5. Suggested Improvements

The following improvements were identified after reviewing the applied code.
They are listed in priority order.

---

### 🔴 Safety / Correctness

#### IMP-1 — Gate the TECS glide flag on `FW_POSCTRL_MODE_AUTO` only

**File:** `FixedwingPositionControl.cpp` — `tecs_update_pitch_throttle()`

**Problem:** `tecs_update_pitch_throttle()` is called from **all** flight modes —
takeoff, landing, loiter, heading hold, etc. The soaring TECS flag is set based
only on the uORB message, without checking the current control mode. If the
companion publishes `glide_mode_enabled=true` during a runway takeoff or final
approach, TECS will silently disable normal throttle and energy management,
which is dangerous.

**Fix:**
```cpp
// In tecs_update_pitch_throttle() — only apply soaring TECS flag in AUTO mode
if (_control_mode_current != FW_POSCTRL_MODE_AUTO) {
    _tecs.set_gliding_mode_enabled(false);
} else {
    // ... existing tecs_glide / tecs_thermal logic
}
```

---

#### IMP-2 — Add predictive altitude-rate early exit before `FW_ALT_MIN`

**File:** `FixedwingPositionControl.cpp` — `control_auto()`

**Problem:** The safety check triggers only when `_current_altitude <= FW_ALT_MIN`,
i.e. after the limit is already breached. In a fast unpowered glide, this may be
too late for a smooth powered recovery (TECS needs time to spin up the motor and
arrest the descent).

**Fix:** Add a rate-of-descent prediction before the hard check:
```cpp
// Exit soaring proactively if we will breach FW_ALT_MIN within MARGIN seconds
const float descent_margin_s = 5.0f;
const float predicted_alt = _current_altitude
    + _local_pos.vz * descent_margin_s;  // vz positive = descending in NED
if (predicted_alt <= _param_fw_alt_min.get()) {
    altitude_safety_triggered = true;
    // (no latch — this is predictive, not a hard breach)
}
```

---

### 🟠 Code Quality / Architecture

#### IMP-3 — Extract altitude-safety logic into a single private helper

**Files:** `FixedwingPositionControl.hpp` + `.cpp`

**Problem:** The 30-line altitude-safety + hysteresis block is copy-pasted verbatim
in both `control_auto()` and `tecs_update_pitch_throttle()`. A bug fixed in one
will silently persist in the other.

**Fix:** Declare and implement a helper:
```cpp
// In .hpp
void _computeSoaringFlags(bool &glide_out, bool &thermal_out);

// In .cpp — called once per Run() cycle, results stored as members
// _effective_glide_active, _effective_thermal_active
```
Both `control_auto()` and `tecs_update_pitch_throttle()` then read the pre-computed
result instead of re-running the logic.

---

#### IMP-4 — Wire up or remove `_in_free_glide` (dead code)

**File:** `FixedwingPositionControl.hpp` / `.cpp`

**Problem:** The member `bool _in_free_glide{false}` is declared in the `.hpp` but
is never assigned or read anywhere in the `.cpp`. It was clearly intended to track
the active soaring state for logging/telemetry but was never wired up.

**Fix (option A — minimal):** Delete the declaration from the `.hpp`.

**Fix (option B — useful):** Set it at the end of the soaring block:
```cpp
_in_free_glide = mission_manual_throttle || freeclimb_mode;
```
Then use it to gate the hard-zero throttle check and for uORB status publishing
(see IMP-12).

---

#### IMP-5 — Only mutate `_alt_max_reached` in one place

**File:** `FixedwingPositionControl.cpp`

**Problem:** Both `control_auto()` and `tecs_update_pitch_throttle()` write to
`_alt_max_reached`. They run in the same cycle (in the same thread). If `control_auto`
sets `_alt_max_reached = false` and then `tecs_update_pitch_throttle` sets it back
to `true` (or vice versa), the state at the end of the cycle may not reflect what
`control_auto` computed, producing inconsistent TECS flag decisions.

**Fix:** After implementing IMP-3, `_alt_max_reached` is only mutated inside the
helper, which is called once. Both callsites then read the result, not the raw flag.

---

#### IMP-6 — Split the shared debounce timer into two

**File:** `FixedwingPositionControl.hpp` / `.cpp`

**Problem:** A single `_soaring_mode_cmd_last_us` timestamp is used for all mode
transitions. If the aircraft switches from thermal → glide (or vice versa) within
1 second, the second command is silently dropped because the debounce timer has
not yet expired.

**Fix:**
```cpp
// In .hpp
hrt_abstime _soaring_glide_cmd_last_us{0};   // debounce for glide-mode switches
hrt_abstime _soaring_thermal_cmd_last_us{0}; // debounce for thermal repositioning

// In glide-mode block
if (hrt_elapsed_time(&_soaring_glide_cmd_last_us) > 1_s) { ... }

// In thermal-mode block
if (hrt_elapsed_time(&_soaring_thermal_cmd_last_us) > 1_s) { ... }
```

---

#### IMP-7 — Simplify the redundant `set_gliding_mode_enabled` expression

**File:** `FixedwingPositionControl.cpp` — `tecs_update_pitch_throttle()`

**Problem:**
```cpp
_tecs.set_gliding_mode_enabled((tecs_glide && !tecs_thermal) || tecs_thermal);
```
This simplifies to `tecs_glide || tecs_thermal` (since if `tecs_thermal` is true
the whole expression is true, and if false the expression reduces to `tecs_glide`).

**Fix:**
```cpp
_tecs.set_gliding_mode_enabled(tecs_glide || tecs_thermal);
```

---

### 🟡 Precision / Robustness

#### IMP-8 — Change `loiter_lat` / `loiter_lon` from `float32` to `float64`

**File:** `msg/AutosoaringControl.msg`

**Problem:** `float32` gives only ~7 significant decimal digits. A latitude like
`48.123456°` uses all 7 digits just for the integer + first 6 decimals. At
mid-latitudes this produces positioning errors of ±10–100 m when the coordinate
is large. GPS coordinates fundamentally require `float64`.

**Fix:**
```
# In AutosoaringControl.msg
float64 loiter_lat   # Thermal center latitude  [deg]; 0 = use current position
float64 loiter_lon   # Thermal center longitude [deg]; 0 = use current position
```
This also removes the `(double)lat` / `(double)lon` casts scattered in the `.cpp`.

---

#### IMP-9 — Use `NAN` instead of `0.0f` in the staleness watchdog

**File:** `FixedwingPositionControl.cpp` — `Run()`

**Problem:** When the companion stops publishing and the 2-second watchdog fires,
the coordinates are reset to `0.0f`. The coordinate `(0°, 0°)` is a real geographic
location in the Gulf of Guinea. While the `has_center` guard (`fabsf(lat) > 1e-4f`)
prevents this specific pair from being used, it is semantically confusing.

**Fix:**
```cpp
_autosoaring_control.loiter_lat = NAN;
_autosoaring_control.loiter_lon = NAN;
```
The existing `PX4_ISFINITE(lat)` check in `has_center` correctly handles `NAN`.

---

#### IMP-10 — Replace magic threshold `1e-4f` with an explicit validity flag

**File:** `msg/AutosoaringControl.msg` + `FixedwingPositionControl.cpp`

**Problem:** The `has_center` check uses `fabsf(lat) > 1e-4f && fabsf(lon) > 1e-4f`
to distinguish "no thermal center" from "valid thermal center". This convention
cannot represent a thermal at `(0°, 0°)` and is not obvious to a reader.

**Fix:** Add a dedicated boolean to the message:
```
# In AutosoaringControl.msg
bool    loiter_center_valid  # True if loiter_lat/lon contain a valid thermal center
```
Replace `has_center` check with:
```cpp
const bool has_center = _autosoaring_control.loiter_center_valid;
```

---

### 🟢 Feature Improvements

#### IMP-11 — Add `loiter_alt_m` field to `AutosoaringControl.msg`

**File:** `msg/AutosoaringControl.msg`

**Problem:** When a DO_REPOSITION is sent, `param7 = NAN` so the navigator defaults
to the current vehicle altitude. The companion may want to specify a target altitude
for the thermal exploitation (e.g., climb to 250 m AGL at the thermal center).

**Fix:**
```
# In AutosoaringControl.msg
float32 loiter_alt_m   # Thermal loiter altitude [m AMSL]; 0 = use current altitude
```
In `control_auto()`:
```cpp
cmd.param7 = (_autosoaring_control.loiter_alt_m > FLT_EPSILON)
             ? (double)_autosoaring_control.loiter_alt_m
             : NAN;
```

---

#### IMP-12 — Publish a `SoaringStatus` topic back to ROS2

**Files:** new `msg/SoaringStatus.msg` + `dds_topics.yaml` + `.cpp`

**Problem:** The companion computer has no direct way to know the FMU's active
soaring state. It can only infer it from `nav_state`, which does not distinguish
"in glide mode" from "normal mission". Without feedback, the companion cannot
implement a proper closed-loop soaring state machine.

**Fix:** Create and bridge a small status message:

```
# msg/SoaringStatus.msg
uint64  timestamp
bool    glide_active               # TECS glide flag is active
bool    thermal_active             # thermal loiter is active
bool    soaring_forbidden_latched  # below FW_ALT_MIN, recovery in progress
float32 current_altitude_m         # current vehicle altitude
float32 alt_min_m                  # FW_ALT_MIN parameter
float32 alt_max_m                  # FW_ALT_MAX parameter
```

In `dds_topics.yaml`, add to `publications`:
```yaml
- topic: /fmu/out/soaring_status
  type: px4_msgs::msg::SoaringStatus
```

Publish from `Run()` alongside the existing uORB poll.

---

#### IMP-13 — Remove redundant altitude-rate clipping in `TECS.cpp`

**File:** `src/lib/tecs/TECS.cpp` — `TECS::update()`

**Problem:** The clipping of `altitude_reference.alt_rate` to `≤ 0.0f` during
gliding prevents TECS from requesting any altitude gain. However, when the aircraft
enters glide mode at a speed above `FW_GLIDE_AIRSPD`, TECS may need to pitch up
slightly to decelerate, which involves a small kinetic→potential energy trade.
The altitude-rate clip interferes with this natural deceleration.

Since `spe_weighting = 0.0f` already removes all altitude-holding authority from
the pitch controller, the altitude-rate clip is redundant and may delay airspeed
convergence when first entering glide mode.

**Fix:** Remove the `math::min(..., 0.0f)` clip and rely solely on the energy
weight `spe_weighting=0, ske_weighting=2` to control pitch behaviour during gliding.

---

### Summary of suggested improvements

| ID | Priority | File(s) | Change |
|----|----------|---------|--------|
| IMP-1 | 🔴 Safety | `FixedwingPositionControl.cpp` | Gate TECS flag on `FW_POSCTRL_MODE_AUTO` |
| IMP-2 | 🔴 Safety | `FixedwingPositionControl.cpp` | Predictive altitude-rate early exit |
| IMP-3 | 🟠 Architecture | `.hpp` + `.cpp` | Extract altitude-safety to private helper |
| IMP-4 | 🟠 Dead code | `.hpp` + `.cpp` | Wire up or remove `_in_free_glide` |
| IMP-5 | 🟠 State bug | `.cpp` | Mutate `_alt_max_reached` in one place only |
| IMP-6 | 🟠 Logic bug | `.hpp` + `.cpp` | Split into two separate debounce timers |
| IMP-7 | 🟠 Simplification | `.cpp` | Simplify `set_gliding_mode_enabled(...)` call |
| IMP-8 | 🟡 Precision | `AutosoaringControl.msg` | Change `loiter_lat/lon` to `float64` |
| IMP-9 | 🟡 Robustness | `.cpp` | Use `NAN` not `0.0f` in staleness watchdog |
| IMP-10 | 🟡 Clarity | `msg` + `.cpp` | Add `loiter_center_valid` boolean field |
| IMP-11 | 🟢 Feature | `msg` + `.cpp` | Add `loiter_alt_m` field to message |
| IMP-12 | 🟢 Feature | new `msg` + `.yaml` + `.cpp` | Publish `SoaringStatus` back to ROS2 |
| IMP-13 | 🟢 Robustness | `TECS.cpp` | Remove redundant altitude-rate clipping |
