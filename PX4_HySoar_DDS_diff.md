# PX4 HySoar DDS-only — unified diff (Pictures vs Music)
Compare **Music** (`Music/PX4-Autopilot`, baseline) with **Pictures** (`Pictures/PX4-Autopilot`, branch `hysoar-dds-only`).
## Design note

- Companion (ROS 2 / XRCE-DDS) is the **only** authority for soaring mode.
- Safety: **2 s** staleness watchdog and **FW_ALT_MIN** altitude floor (no CLI override).

## Files in this diff (14 paths)

| Path | Change | Notes |
|------|--------|-------|
| `msg/AutosoaringControl.msg` | NEW | uORB command message |
| `msg/AutosoaringStatus.msg` | NEW | uORB status feedback |
| `msg/CMakeLists.txt` | modified | Register new messages |
| `src/lib/tecs/TECS.hpp` | modified | Glide parameter API |
| `src/lib/tecs/TECS.cpp` | modified | Glide control law |
| `src/modules/fw_pos_control/FixedwingPositionControl.hpp` | modified | Soaring state |
| `src/modules/fw_pos_control/FixedwingPositionControl.cpp` | modified | DDS poll, TECS, navigator cmds |
| `src/modules/fw_pos_control/fw_path_navigation_params.c` | modified | Soaring parameters |
| `src/modules/commander/Commander.cpp` | modified | `DO_REPOSITION` param2=0 fix |
| `src/modules/navigator/mission.cpp` | modified | Mission helpers |
| `src/modules/navigator/mission_base.h` | modified | `_mission_paused_seq` |
| `src/modules/navigator/mission_base.cpp` | modified | Rewind guard |
| `src/modules/navigator/navigator_main.cpp` | modified | Navigator tweaks |
| `src/modules/uxrce_dds_client/dds_topics.yaml` | modified | DDS bridge topics |

## How to use this document

- Each path below has its **own** `diff` code fence so editors and GitHub render reliably (one tall fence used to confuse some previews).
- To build one patch file, concatenate all fenced bodies or run from Music tree:

```bash
diff -u Music/PX4-Autopilot/src/.../File.cpp Pictures/PX4-Autopilot/src/.../File.cpp
```

## Per-file unified diffs

### `msg/AutosoaringControl.msg` (NEW)

```diff
--- /dev/null
+++ b/msg/AutosoaringControl.msg
@@ -0,0 +1,44 @@
+# AutosoaringControl — uORB message published by a ROS2 companion-computer node
+# and bridged into the FMU via XRCE-DDS (/fmu/in/autosoaring_control).
+# The FMU fw_pos_control module reads this message to select the soaring mode.
+#
+# ── Mode encoding ──────────────────────────────────────────────────────────────
+# soaring_mode  |  TECS         | Navigator       | Fields used
+# ─────────────────────────────────────────────────────────────────────────────
+# 0  OFF         | powered       | unchanged       | (none)
+# 1  GLIDE_FIXED | gliding       | AUTO_MISSION    | airspeed_cmd
+# 2  GLIDE_POLAR | gliding       | AUTO_MISSION    | (polar from params)
+# 3  THERMAL_LOITER | gliding    | AUTO_LOITER     | loiter_*, bank_angle_cmd, loiter_clockwise
+# 4  THERMAL_BANK   | gliding    | AUTO_LOITER     | bank_angle_cmd, loiter_clockwise (roll sign)
+# ─────────────────────────────────────────────────────────────────────────────
+# Fields not listed for a mode are silently ignored.
+
+uint64  timestamp               # time since system start (microseconds)
+
+# ── Mode command ───────────────────────────────────────────────────────────────
+uint8   soaring_mode            # Active soaring mode (see constants below)
+
+uint8 SOARING_OFF             = 0   # Normal powered flight
+uint8 SOARING_GLIDE_FIXED     = 1   # Engine-off glide, fixed EAS = FW_GLIDE_AIRSPD (or airspeed_cmd)
+uint8 SOARING_GLIDE_POLAR     = 2   # Engine-off glide, polar best-glide EAS = sqrt(FW_POLAR_B/FW_POLAR_A)
+uint8 SOARING_THERMAL_LOITER  = 3   # Thermalling, radius-guided loiter via navigator
+uint8 SOARING_THERMAL_BANK    = 4   # Thermalling, direct bank-angle command (companion controls centering)
+
+# ── Loiter geometry — used only by SOARING_THERMAL_LOITER (3) ─────────────────
+float32 loiter_radius_m         # Loiter radius [m]; NaN = derive minimum from bank_angle_cmd
+float32 loiter_lat              # Thermal centre latitude  [deg]; NaN or 0 = use current position
+float32 loiter_lon              # Thermal centre longitude [deg]; NaN or 0 = use current position
+
+# ── Per-cycle optimisation — NaN means "use PX4 default parameter" ─────────────
+float32 airspeed_cmd            # MacCready airspeed [m/s EAS]; used by SOARING_GLIDE_FIXED only
+float32 bank_angle_cmd          # Bank angle magnitude [deg]; used by SOARING_THERMAL_LOITER and SOARING_THERMAL_BANK
+bool    loiter_clockwise        # Turn direction: true = clockwise (right), false = CCW (left). Default: true
+
+# ── Metadata ───────────────────────────────────────────────────────────────────
+uint8   soaring_phase           # Companion-reported phase (for logging / GCS display)
+
+uint8 SOARING_PHASE_CRUISE    = 0   # Normal powered cruise
+uint8 SOARING_PHASE_GLIDING   = 1   # Engine-off glide along mission
+uint8 SOARING_PHASE_SEARCHING = 2   # Searching for thermal (circling without confirmed lift)
+uint8 SOARING_PHASE_CENTERING = 3   # Adjusting circle to centre on thermal core
+uint8 SOARING_PHASE_THERMAL   = 4   # Confirmed thermal — climbing
```

### `msg/AutosoaringStatus.msg` (NEW)

```diff
--- /dev/null
+++ b/msg/AutosoaringStatus.msg
@@ -0,0 +1,21 @@
+# AutosoaringStatus — uORB message published by fw_pos_control.
+# Bridged to the companion via XRCE-DDS as /fmu/out/autosoaring_status.
+# Provides FMU-side feedback: active mode, staleness events, and derived soaring phase.
+
+uint64 timestamp               # time since system start (microseconds)
+
+uint8 source                   # What caused this status emission (see constants below)
+uint8 SOURCE_DDS_OFF              = 0   # Companion sent SOARING_OFF
+uint8 SOURCE_STALENESS_WATCHDOG   = 1   # Companion silent >2 s — soaring forced OFF
+uint8 SOURCE_PERIODIC             = 2   # Periodic heartbeat during active soaring (2 Hz)
+
+uint8 soaring_mode_effective   # Mode stored in _autosoaring_control after the event (same encoding as AutosoaringControl.soaring_mode)
+
+uint64 dds_inhibit_until_us    # Reserved — always 0 in DDS-only mode
+
+# FMU-derived phase — what the FMU is actually doing this cycle (same encoding as AutosoaringControl.soaring_phase)
+uint8 fmu_soaring_phase
+uint8 SOARING_PHASE_CRUISE    = 0   # Powered cruise (soaring OFF or forbidden)
+uint8 SOARING_PHASE_GLIDING   = 1   # Engine-off glide along mission (effective_glide)
+uint8 SOARING_PHASE_SEARCHING = 2   # Thermal active but loiter not yet positioned (_soaring_last_lat==NAN)
+uint8 SOARING_PHASE_THERMAL   = 3   # Thermal loiter active and loiter centre established
```

### `msg/CMakeLists.txt` (modified)

```diff
--- a/msg/CMakeLists.txt
+++ b/msg/CMakeLists.txt
@@ -40,6 +40,8 @@
 	AckermannVelocitySetpoint.msg
 	ActionRequest.msg
 	ActuatorArmed.msg
+	AutosoaringControl.msg
+	AutosoaringStatus.msg
 	ActuatorControlsStatus.msg
 	ActuatorOutputs.msg
 	ActuatorServosTrim.msg
```

### `src/lib/tecs/TECS.hpp` (modified)

```diff
--- a/src/lib/tecs/TECS.hpp
+++ b/src/lib/tecs/TECS.hpp
@@ -238,6 +238,11 @@
 		float load_factor;					///< Additional normal load factor.
 
 		float fast_descend;
+
+		// Soaring mode parameters
+		float gliding_airspeed_setpoint{10.0f}; 	///< EAS setpoint during gliding [m/s EAS]; pre-computed by FixedwingPositionControl (fixed or polar); converted to TAS inside calcTrueAirspeedSetpoint.
+		float glide_i_decay{10.0f};			///< Throttle integrator decay time constant during gliding [s]. I(t)=I(0)*exp(-t/tau).
+		float glide_stall_eas{3.0f};			///< Hard absolute minimum EAS [m/s] for glide airspeed floor; set from FW_AIRSPD_STALL.
 	};
 
 	/**
@@ -283,6 +288,7 @@
 	struct Flag {
 		bool airspeed_enabled;			///< Flag if the airspeed sensor is enabled.
 		bool detect_underspeed_enabled;		///< Flag if underspeed detection is enabled.
+		bool gliding_mode_enabled{false};	///< True during engine-off soaring (glide or thermal). Modifies TECS energy weighting, throttle, and integrators.
 	};
 public:
 	TECSControl() = default;
@@ -501,10 +507,11 @@
 	 * @param limit is the specific total energy rate limits in [m²/s³].
 	 * @param specific_energy_rate is the specific energy rates in [m²/s³].
 	 * @param param is the control parameters.
+	 * @param flag is the control flags (used to skip load-factor correction in gliding).
 	 * @return specific total energy rate values in [m²/s³]
 	 */
 	ControlValues _calcThrottleControlSteRate(const STERateLimit &limit, const SpecificEnergyRates &specific_energy_rate,
-			const Param &param) const;
+			const Param &param, const Flag &flag) const;
 
 	/**
 	 * @brief Calculate the throttle control update function.
@@ -536,6 +543,7 @@
 	AlphaFilter<float> _ste_rate_estimate_filter;		///< Low pass filter for the specific total energy rate.
 	float _pitch_integ_state{0.0f};				///< Pitch integrator state [rad].
 	float _throttle_integ_state{0.0f};			///< Throttle integrator state [-].
+	bool  _prev_gliding_mode{false};			///< Gliding mode flag from the previous update cycle; used to detect glide-entry transition.
 
 	// Output
 	DebugOutput _debug_output;				///< Debug output.
@@ -597,6 +605,13 @@
 
 	void set_detect_underspeed_enabled(bool enabled) { _control_flag.detect_underspeed_enabled = enabled; };
 
+	// Soaring mode interface
+	void set_gliding_mode_enabled(bool enabled) { _control_flag.gliding_mode_enabled = enabled; }
+	bool get_gliding_mode_enabled() const { return _control_flag.gliding_mode_enabled; }
+	void set_gliding_airspeed_setpoint(float eas) { _control_param.gliding_airspeed_setpoint = eas; } ///< Set glide EAS setpoint [m/s]; pre-computed by caller (fixed or polar-optimal); converted to TAS internally.
+	void set_glide_i_decay(float tau) { _control_param.glide_i_decay = math::max(tau, 0.1f); }
+	void set_glide_stall_eas(float eas) { _control_param.glide_stall_eas = math::max(eas, 1.0f); } ///< Set stall EAS floor for glide mode [m/s]; from FW_AIRSPD_STALL.
+
 	// setters for parameters
 	void set_airspeed_measurement_std_dev(float std_dev) {_airspeed_filter_param.airspeed_measurement_std_dev = std_dev;};
 	void set_airspeed_rate_measurement_std_dev(float std_dev) {_airspeed_filter_param.airspeed_rate_measurement_std_dev = std_dev;};
@@ -751,12 +766,16 @@
 		.throttle_slewrate = 0.0f,
 		.load_factor_correction = 0.0f,
 		.load_factor = 1.0f,
-		.fast_descend = 0.f
+		.fast_descend = 0.f,
+		.gliding_airspeed_setpoint = 10.0f,
+		.glide_i_decay = 10.0f,
+		.glide_stall_eas = 3.0f,
 	};
 
 	TECSControl::Flag _control_flag{
 		.airspeed_enabled = false,
 		.detect_underspeed_enabled = false,
+		.gliding_mode_enabled = false,
 	};
 
 	/**
```

### `src/lib/tecs/TECS.cpp` (modified)

```diff
--- a/src/lib/tecs/TECS.cpp
+++ b/src/lib/tecs/TECS.cpp
@@ -250,7 +250,7 @@
 
 	_ste_rate_estimate_filter.reset(specific_energy_rate.spe_rate.estimate + specific_energy_rate.ske_rate.estimate);
 
-	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rate, param)};
+	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rate, param, flag)};
 
 	_throttle_setpoint = _calcThrottleControlOutput(limit, ste_rate, param, flag);
 
@@ -305,7 +305,8 @@
 TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Param &param) const
 {
 	TECSControl::STERateLimit limit;
-	// Calculate the specific total energy rate limits from the max throttle limits
+	// Energy limits define the valid flight envelope in all modes including gliding.
+	// Gliding energy management is handled by throttle zeroing and energy weighting, not by changing these limits.
 	limit.STE_rate_max = math::max(param.max_climb_rate, FLT_EPSILON) * CONSTANTS_ONE_G;
 	limit.STE_rate_min = - math::max(param.min_sink_rate, FLT_EPSILON) * CONSTANTS_ONE_G;
 
@@ -323,9 +324,18 @@
 	// if airspeed measurement is not enabled then always set the rate setpoint to zero in order to avoid constant rate setpoints
 	if (flag.airspeed_enabled) {
 		// Calculate limits for the demanded rate of change of speed based on physical performance limits
-		// with a 50% margin to allow the total energy controller to correct for errors. Increase it in case of fast descend
-		const float max_tas_rate_sp = (param.fast_descend * 0.5f + 0.5f) * limit.STE_rate_max / math::max(input.tas,
-					      FLT_EPSILON);
+		// with a 50% margin to allow the total energy controller to correct for errors. Increase it in case of fast descend.
+		//
+		// In gliding mode no thrust is available, so the only energy source for acceleration is gravity
+		// (trading altitude for speed by pitching down).  The achievable energy rate is bounded by the
+		// maximum sink rate (|STE_rate_min|), NOT by STE_rate_max which is derived from max_climb_rate
+		// (a powered-flight value).  Using the powered limit would allow the setpoint to demand kinetic
+		// energy gains that are physically impossible without thrust, producing spurious pitch transients.
+		const float available_accel_energy_rate = flag.gliding_mode_enabled
+				? fabsf(limit.STE_rate_min)   // gravity-only: bounded by glide polar sink rate
+				: limit.STE_rate_max;          // powered: bounded by max climb rate
+		const float max_tas_rate_sp = (param.fast_descend * 0.5f + 0.5f) * available_accel_energy_rate
+					      / math::max(input.tas, FLT_EPSILON);
 		const float min_tas_rate_sp = (param.fast_descend * 0.5f + 0.5f) * limit.STE_rate_min / math::max(input.tas,
 					      FLT_EPSILON);
 		airspeed_rate_output = constrain((setpoint.tas_setpoint - input.tas) * param.airspeed_error_gain, min_tas_rate_sp,
@@ -370,6 +380,18 @@
 		return;
 	}
 
+	// In gliding mode the airspeed setpoint is FW_GLIDE_AIRSPD, which is deliberately
+	// lower than the cruise trim speed.  The underspeed boundary is computed relative to
+	// tas_min and param.equivalent_airspeed_trim (cruise), so glide speed would fall
+	// inside the "starting to underspeed" zone and produce a non-zero _ratio_undersped.
+	// That falsely activates underspeed mitigation (which tries to raise throttle and
+	// force speed-on-elevator weighting) even though both are already correctly set by
+	// the gliding flag.  Clear the ratio and return to keep the control path clean.
+	if (flag.gliding_mode_enabled) {
+		_ratio_undersped = 0.0f;
+		return;
+	}
+
 	// this is the expected (something like standard) deviation from the airspeed setpoint that we allow the airspeed
 	// to vary in before ramping in underspeed mitigation
 	const float tas_error_bound = param.tas_error_percentage * param.equivalent_airspeed_trim;
@@ -390,6 +412,14 @@
 {
 
 	SpecificEnergyWeighting weight;
+
+	
+	if (flag.gliding_mode_enabled) {
+		weight.spe_weighting = 0.0f;  // altitude not controlled by pitch in glide
+		weight.ske_weighting = 2.0f;  // full speed control via pitch
+		return weight;
+	}
+
 	// Calculate the weight applied to control of specific kinetic energy error
 	float pitch_speed_weight = constrain(param.pitch_speed_weight, 0.0f, 2.0f);
 
@@ -412,8 +442,34 @@
 }
 
 void TECSControl::_calcPitchControl(float dt, const Input &input, const SpecificEnergyRates &specific_energy_rates,
-				    const Param &param, const Flag &flag)
+			    const Param &param, const Flag &flag)
 {
+	// Detect glide entry/exit transitions and partially reset the pitch integrator.
+	//
+	// ENTRY (powered → glide): the cruise pitch integrator holds an altitude-control
+	// bias (w_spe > 0 trim).  When gliding starts, pitch switches to speed-on-elevator
+	// (w_ske = 2, w_spe = 0).  That bias now fights the speed setpoint, causing an
+	// airspeed overshoot for the first few seconds.  Halving it on entry removes most
+	// of the mismatch while retaining CG/trim knowledge.
+	//
+	// EXIT (glide → powered): the glide pitch integrator has wound up to hold the glide
+	// airspeed at zero throttle (speed-control bias).  When throttle ramps back in, the
+	// total energy rises quickly while this integrator still commands a nose-down pitch,
+	// producing a large pitch-rate spike (~50 deg/s) before the integrator decays.
+	// Halving it on exit suppresses the spike symmetrically.
+	//
+	// Partial reset (×0.5): fast handover without fully discarding steady-state trim.
+	if (flag.gliding_mode_enabled && !_prev_gliding_mode) {
+		// glide entry
+		_pitch_integ_state *= 0.5f;
+
+	} else if (!flag.gliding_mode_enabled && _prev_gliding_mode) {
+		// glide exit — mirror reset to damp the pitch-rate spike on power restoration
+		_pitch_integ_state *= 0.5f;
+	}
+
+	_prev_gliding_mode = flag.gliding_mode_enabled;
+
 	const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
 	ControlValues seb_rate{_calcPitchControlSebRate(weight, specific_energy_rates)};
 
@@ -460,9 +516,10 @@
 void TECSControl::_calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate,
 		const Param &param)
 {
+	
 	if (param.integrator_gain_pitch > FLT_EPSILON) {
 
-		// Calculate derivative from change in climb angle to rate of change of specific energy balance
+		// Normalisation: ΔSEB_rate / Δpitch ≈ TAS × g  (small-angle, SPE-dominant, Lambregts 1983)
 		const float climb_angle_to_SEB_rate = input.tas * CONSTANTS_ONE_G;
 
 		// Calculate pitch integrator input term
@@ -520,7 +577,7 @@
 	const float STE_rate_estimate_raw = specific_energy_rates.spe_rate.estimate + specific_energy_rates.ske_rate.estimate;
 	_ste_rate_estimate_filter.setParameters(dt, param.ste_rate_time_const);
 	_ste_rate_estimate_filter.update(STE_rate_estimate_raw);
-	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rates, param)};
+	ControlValues ste_rate{_calcThrottleControlSteRate(limit, specific_energy_rates, param, flag)};
 	float throttle_setpoint{param.throttle_min};
 
 	if (1.f - param.fast_descend < FLT_EPSILON) {
@@ -550,7 +607,7 @@
 
 TECSControl::ControlValues TECSControl::_calcThrottleControlSteRate(const STERateLimit &limit,
 		const SpecificEnergyRates &specific_energy_rates,
-		const Param &param) const
+		const Param &param, const Flag &flag) const
 {
 	// Output ste rate values
 	ControlValues ste_rate;
@@ -558,8 +615,12 @@
 
 	// Adjust the demanded total energy rate to compensate for induced drag rise in turns.
 	// Assume induced drag scales linearly with normal load factor.
-	// The additional normal load factor is given by (1/cos(bank angle) - 1)
-	ste_rate.setpoint += param.load_factor_correction * (param.load_factor - 1.f);
+	// The additional normal load factor is given by (1/cos(bank angle) - 1).
+	// Skip this correction in gliding: throttle is hard-zero anyway, so the correction
+	// only contaminates the STE debug signal without affecting the output.
+	if (!flag.gliding_mode_enabled) {
+		ste_rate.setpoint += param.load_factor_correction * (param.load_factor - 1.f);
+	}
 
 	ste_rate.setpoint = constrain(ste_rate.setpoint, limit.STE_rate_min, limit.STE_rate_max);
 	ste_rate.estimate = _ste_rate_estimate_filter.getState();
@@ -570,6 +631,13 @@
 void TECSControl::_calcThrottleControlUpdate(float dt, const STERateLimit &limit, const ControlValues &ste_rate,
 		const Param &param, const Flag &flag)
 {
+	
+	if (flag.gliding_mode_enabled) {
+		const float decay = math::max(param.glide_i_decay, 0.1f);
+		_throttle_integ_state -= dt * _throttle_integ_state / decay;
+		return;
+	}
+
 	// Calculate gain scaler from specific energy rate error to throttle
 	const float STE_rate_to_throttle = 1.0f / (limit.STE_rate_max - limit.STE_rate_min);
 
@@ -603,6 +671,11 @@
 		const Param &param,
 		const Flag &flag) const
 {
+
+	if (flag.gliding_mode_enabled) {
+		return param.throttle_min;
+	}
+
 	// Calculate gain scaler from specific energy rate error to throttle
 	const float STE_rate_to_throttle = 1.0f / (limit.STE_rate_max - limit.STE_rate_min);
 
@@ -675,6 +748,33 @@
 
 float TECS::calcTrueAirspeedSetpoint(float eas_to_tas, float eas_setpoint)
 {
+	if (_control_flag.gliding_mode_enabled) {
+		// gliding_airspeed_setpoint is pre-computed by FixedwingPositionControl:
+		//   SOARING_GLIDE_FIXED  → FW_GLIDE_AIRSPD (or companion airspeed_cmd)
+		//   SOARING_GLIDE_POLAR  → sqrt(FW_POLAR_B / FW_POLAR_A)  (best-glide EAS)
+		//   SOARING_THERMAL_*    → FW_GLIDE_AIRSPD (speed held during thermalling)
+		// TECS only needs to convert EAS → TAS and apply the banked stall floor.
+		if (_control_param.gliding_airspeed_setpoint > FLT_EPSILON &&
+		    PX4_ISFINITE(_control_param.gliding_airspeed_setpoint)) {
+			const float tas_sp = eas_to_tas * _control_param.gliding_airspeed_setpoint;
+
+			// In glide mode underspeed detection is already bypassed (_ratio_undersped = 0),
+			// so tas_min (the powered-flight minimum from FW_AIRSPD_MIN) must NOT be
+			// applied here — it would silently floor FW_GLIDE_AIRSPD to FW_AIRSPD_MIN.
+			// Use tas_max as upper bound and FW_AIRSPD_STALL (converted to TAS) as the
+			// hard absolute minimum. The banked stall floor (load_factor) is retained
+			// for safety in turns.
+			const float tas_stall   = eas_to_tas * _control_param.glide_stall_eas;
+			const float glide_floor = math::max(tas_stall,
+							    _control_param.tas_min * sqrtf(_control_param.load_factor) * 0.5f);
+			return math::constrain(tas_sp, glide_floor, _control_param.tas_max);
+		}
+
+		PX4_WARN("TECS glide: airspeed setpoint invalid, falling back to trim speed");
+		return math::constrain(eas_to_tas * _control_param.equivalent_airspeed_trim,
+				       _control_param.tas_min, _control_param.tas_max);
+	}
+
 	return lerp(eas_to_tas * eas_setpoint, _control_param.tas_max, _fast_descend);
 }
 
@@ -740,8 +840,16 @@
 		_airspeed_filter.update(dt, airspeed_input, _airspeed_filter_param, _control_flag.airspeed_enabled);
 
 		// Update Reference model submodule
-		if (1.f - _fast_descend < FLT_EPSILON) {
-			// Reset the altitude reference model, while we are in fast descend.
+		if (_control_flag.gliding_mode_enabled) {
+	
+			const TECSAltitudeReferenceModel::AltitudeReferenceState frozen_state{
+				.alt = altitude,
+				.alt_rate = hgt_rate};
+			_altitude_reference_model.initialize(frozen_state);
+
+		} else 
+		 if (1.f - _fast_descend < FLT_EPSILON) {
+			// Reset the altitude reference model while in fast descend.
 			const TECSAltitudeReferenceModel::AltitudeReferenceState init_state{
 				.alt = altitude,
 				.alt_rate = hgt_rate};
@@ -758,6 +866,7 @@
 		control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
 		control_setpoint.altitude_rate_setpoint_direct = _altitude_reference_model.getHeightRateSetpointDirect();
 		control_setpoint.tas_setpoint = calcTrueAirspeedSetpoint(eas_to_tas, EAS_setpoint);
+	
 
 		const TECSControl::Input control_input{ .altitude = altitude,
 							.altitude_rate = hgt_rate,
@@ -781,6 +890,13 @@
 
 void TECS::_setFastDescend(const float alt_setpoint, const float alt)
 {
+
+	if (_control_flag.gliding_mode_enabled) {
+		_fast_descend = 0.0f;
+		_enabled_fast_descend_timestamp = 0U;
+		return;
+	}
+
 	if (_control_flag.airspeed_enabled && (_fast_descend_alt_err > FLT_EPSILON)
 	    && ((alt_setpoint + _fast_descend_alt_err) < alt)) {
 		auto now = hrt_absolute_time();
```

### `src/modules/fw_pos_control/FixedwingPositionControl.hpp` (modified)

```diff
--- a/src/modules/fw_pos_control/FixedwingPositionControl.hpp
+++ b/src/modules/fw_pos_control/FixedwingPositionControl.hpp
@@ -72,9 +72,12 @@
 #include <uORB/Subscription.hpp>
 #include <uORB/SubscriptionCallback.hpp>
 #include <uORB/topics/airspeed_validated.h>
+#include <uORB/topics/autosoaring_control.h>    // ROS2 soaring command topic (via XRCE-DDS)
+#include <uORB/topics/autosoaring_status.h>     // FMU → companion autosoaring status (via XRCE-DDS)
 #include <uORB/topics/flight_phase_estimation.h>
 #include <uORB/topics/landing_gear.h>
 #include <uORB/topics/launch_detection_status.h>
+#include <uORB/topics/mission.h>
 #include <uORB/topics/manual_control_setpoint.h>
 #include <uORB/topics/normalized_unsigned_setpoint.h>
 #include <uORB/topics/npfg_status.h>
@@ -204,10 +207,12 @@
 	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
 
 	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
+	uORB::Subscription _autosoaring_control_sub{ORB_ID(autosoaring_control)};  // ROS2 soaring commands
 	uORB::Subscription _wind_sub{ORB_ID(wind)};
 	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
 	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
 	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
+	uORB::Subscription _mission_sub{ORB_ID(mission)};
 	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};
 	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
 	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
@@ -229,6 +234,8 @@
 	uORB::Publication<normalized_unsigned_setpoint_s> _flaps_setpoint_pub{ORB_ID(flaps_setpoint)};
 	uORB::Publication<normalized_unsigned_setpoint_s> _spoilers_setpoint_pub{ORB_ID(spoilers_setpoint)};
 	uORB::PublicationData<flight_phase_estimation_s> _flight_phase_estimation_pub{ORB_ID(flight_phase_estimation)};
+	uORB::Publication<autosoaring_status_s> _autosoaring_status_pub{ORB_ID(autosoaring_status)};
+	uORB::Publication<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};  // mode-switch commands
 
 	manual_control_setpoint_s _manual_control_setpoint{};
 	position_setpoint_triplet_s _pos_sp_triplet{};
@@ -404,6 +411,28 @@
 
 	bool _tecs_is_running{false};
 
+	// Soaring state (driven by AutosoaringControl uORB from ROS2 companion via XRCE-DDS)
+	autosoaring_control_s _autosoaring_control{};       ///< Last received soaring command message
+	hrt_abstime  _autosoaring_last_recv_us{0};          ///< Timestamp of last valid AutosoaringControl message
+	hrt_abstime  _soaring_mode_cmd_last_us{0};          ///< Debounce: minimum 1 s between mode-switch commands
+	hrt_abstime  _autosoaring_status_periodic_us{0};    ///< Last periodic phase heartbeat while soaring active (2 Hz)
+	double       _soaring_last_lat{NAN};                ///< Last thermal centre latitude sent to navigator
+	double       _soaring_last_lon{NAN};                ///< Last thermal centre longitude sent to navigator
+	bool         _soaring_last_clockwise{true};         ///< Last turn direction sent to navigator; change triggers re-DO_REPOSITION
+	bool         _alt_max_reached{false};               ///< Hysteresis state: altitude ceiling reached
+	bool         _soaring_forbidden_latched{false};     ///< Latch: soaring disabled below FW_ALT_MIN until companion re-enables
+	bool         _soaring_was_thermal{false};           ///< Previous-cycle thermal state (modes 3 or 4); used to detect thermal→off transition for auto-exit
+	bool         _soaring_was_bank_thermal{false};      ///< Previous-cycle SOARING_THERMAL_BANK (mode 4) state; used to clean up roll override on exit
+	bool         _soaring_was_glide{false};             ///< Previous-cycle glide state (modes 1 or 2); used to detect glide→off transition for auto mission-restore
+	bool         _soaring_was_active{false};            ///< Previous-cycle combined soaring state (glide OR thermal); used for throttle ramp detection
+	hrt_abstime  _soaring_ramp_start_us{0};             ///< Timestamp when glide-exit throttle ramp began; 0 = ramp not active
+	hrt_abstime  _soaring_entry_ramp_start_us{0};       ///< Timestamp when glide-entry throttle ramp began; 0 = ramp not active
+	float        _soaring_pre_glide_thrust{0.0f};       ///< TECS thrust cached one cycle before glide activated; used as entry-ramp start value
+	float        _soaring_bank_angle_cmd_deg{40.0f};   ///< Effective thermal bank angle [deg]: companion bank_angle_cmd or FW_THERMAL_BANK fallback
+	uint16_t     _soaring_mission_resume_seq{0};       ///< mission.current_seq latched on thermal entry (first DO_REPOSITION)
+	bool         _soaring_mission_resume_valid{false}; ///< True if _soaring_mission_resume_seq is meaningful for MISSION_START after thermal
+	bool         _soaring_pending_mission_restore{false}; ///< Defer VEHICLE_CMD_MISSION_START until nav_state==AUTO_MISSION
+
 	// Smooths changes in the altitude tracking error time constant value
 	SlewRate<float> _tecs_alt_time_const_slew_rate;
 
@@ -469,6 +498,8 @@
 	void manual_control_setpoint_poll();
 	void vehicle_attitude_poll();
 	void vehicle_command_poll();
+	void publish_autosoaring_status(uint8_t source, uint8_t soaring_mode_effective, hrt_abstime dds_inhibit_until_us,
+					uint8_t fmu_soaring_phase);
 	void vehicle_control_mode_poll();
 	void vehicle_status_poll();
 	void wind_poll();
@@ -1051,7 +1082,19 @@
 		(ParamFloat<px4::params::FW_TKO_AIRSPD>) _param_fw_tko_airspd,
 
 		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
-		(ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on
+		(ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on,
+
+		// Soaring mode parameters (companion sends AutosoaringControl uORB)
+		(ParamFloat<px4::params::FW_GLIDE_AIRSPD>)  _param_fw_glide_airspd,   ///< Fixed glide EAS setpoint [m/s]; used by SOARING_GLIDE_FIXED
+		(ParamFloat<px4::params::FW_AIRSPD_STALL>)  _param_fw_airspd_stall,   ///< Stall EAS [m/s]; used as hard minimum floor for glide airspeed setpoint
+		(ParamFloat<px4::params::FW_ALT_MIN>)        _param_fw_alt_min,        ///< Soaring altitude floor [m]
+		(ParamFloat<px4::params::FW_ALT_MAX>)        _param_fw_alt_max,        ///< Soaring altitude ceiling [m]
+		(ParamFloat<px4::params::FW_ALT_HYST>)       _param_fw_alt_hyst,       ///< Ceiling hysteresis band [m]
+		(ParamFloat<px4::params::FW_GLIDE_I_DECAY>)  _param_fw_glide_i_decay,  ///< Throttle integrator decay tau [s]
+		(ParamFloat<px4::params::FW_POLAR_A>)        _param_fw_polar_a,        ///< Polar: parasitic drag coefficient a [s/m]; used by SOARING_GLIDE_POLAR
+		(ParamFloat<px4::params::FW_POLAR_B>)        _param_fw_polar_b,        ///< Polar: minimum sink rate b [m/s]; used by SOARING_GLIDE_POLAR
+		(ParamFloat<px4::params::FW_GLIDE_RAMP_T>)   _param_fw_glide_ramp_t,   ///< Throttle ramp duration after glide exit [s]
+		(ParamFloat<px4::params::FW_THERMAL_BANK>)   _param_fw_thermal_bank    ///< Default bank angle during thermalling [deg]
 	)
 
 };
```

### `src/modules/fw_pos_control/FixedwingPositionControl.cpp` (modified)

```diff
--- a/src/modules/fw_pos_control/FixedwingPositionControl.cpp
+++ b/src/modules/fw_pos_control/FixedwingPositionControl.cpp
@@ -34,6 +34,8 @@
 #include "FixedwingPositionControl.hpp"
 
 #include <px4_platform_common/events.h>
+#include <parameters/param.h>
+#include <commander/px4_custom_mode.h>
 
 using math::constrain;
 using math::max;
@@ -136,6 +138,22 @@
 	_tecs.set_throttle_damp(_param_fw_t_thr_damping.get());
 	_tecs.set_integrator_gain_throttle(_param_fw_t_thr_integ.get());
 	_tecs.set_integrator_gain_pitch(_param_fw_t_I_gain_pit.get());
+	// Glide airspeed setpoint: computed here so TECS stays mode-agnostic.
+	// SOARING_GLIDE_POLAR (mode 2) uses polar best-glide EAS = sqrt(b/a).
+	// All other soaring modes use FW_GLIDE_AIRSPD (overridable per-cycle by airspeed_cmd).
+	// The active soaring_mode is checked via _autosoaring_control; at parameters_update time
+	// we set the polar value eagerly so the first TECS cycle after a mode-2 activation is correct.
+	{
+		const float polar_a = _param_fw_polar_a.get();
+		const float polar_b = _param_fw_polar_b.get();
+		const float v_polar_eas = (polar_a > FLT_EPSILON && polar_b > FLT_EPSILON)
+					  ? sqrtf(polar_b / polar_a) : 0.0f;
+		const bool polar_mode = (_autosoaring_control.soaring_mode == autosoaring_control_s::SOARING_GLIDE_POLAR);
+		_tecs.set_gliding_airspeed_setpoint(polar_mode && v_polar_eas > FLT_EPSILON
+						    ? v_polar_eas : _param_fw_glide_airspd.get());
+	}
+	_tecs.set_glide_i_decay(_param_fw_glide_i_decay.get()); // throttle integrator decay (FW_GLIDE_I_DECAY)
+	_tecs.set_glide_stall_eas(_param_fw_airspd_stall.get()); // absolute glide floor = stall speed (FW_AIRSPD_STALL)
 	_tecs.set_throttle_slewrate(_param_fw_thr_slew_max.get());
 	_tecs.set_vertical_accel_limit(_param_fw_t_vert_acc.get());
 	_tecs.set_roll_throttle_compensation(_param_fw_t_rll2thr.get());
@@ -507,6 +525,19 @@
 	_pos_ctrl_landing_status_pub.publish(pos_ctrl_landing_status);
 }
 
+void
+FixedwingPositionControl::publish_autosoaring_status(uint8_t source, uint8_t soaring_mode_effective,
+		hrt_abstime dds_inhibit_until_us, uint8_t fmu_soaring_phase)
+{
+	autosoaring_status_s status{};
+	status.timestamp = hrt_absolute_time();
+	status.source = source;
+	status.soaring_mode_effective = soaring_mode_effective;
+	status.dds_inhibit_until_us = dds_inhibit_until_us;
+	status.fmu_soaring_phase = fmu_soaring_phase;
+	_autosoaring_status_pub.publish(status);
+}
+
 float FixedwingPositionControl::getCorrectedNpfgRollSetpoint()
 {
 	// Scale the npfg output to zero if npfg is not certain for correct output
@@ -864,6 +895,331 @@
 	position_setpoint_s current_sp = pos_sp_curr;
 	move_position_setpoint_for_vtol_transition(current_sp);
 
+	// After thermal → AUTO_LOITER → AUTO_MISSION, force the mission index we latched at
+	// thermal entry. Commander applies DO_SET_MODE before the navigator has necessarily
+	// finished reconciling mission state; publishing here only once nav_state is
+	// AUTO_MISSION avoids racing the mode switch and overrides stale seq rewinds.
+	if (_soaring_pending_mission_restore && _soaring_mission_resume_valid
+	    && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION)) {
+		_soaring_pending_mission_restore = false;
+
+		vehicle_command_s restore_cmd{};
+		restore_cmd.timestamp        = hrt_absolute_time();
+		restore_cmd.command          = vehicle_command_s::VEHICLE_CMD_MISSION_START;
+		restore_cmd.param1           = (float)_soaring_mission_resume_seq;
+		restore_cmd.param2           = 0.f;
+		restore_cmd.target_system    = 1;
+		restore_cmd.target_component = 1;
+		_pub_vehicle_command.publish(restore_cmd);
+
+		_soaring_mission_resume_valid = false;
+		PX4_INFO("Autosoaring: MISSION_START seq %u (post-thermal)", (unsigned)_soaring_mission_resume_seq);
+	}
+
+	// -------------------------------------------------------------------------
+	// Autosoaring control block
+	// -------------------------------------------------------------------------
+	// Single entry path: ROS 2 companion via XRCE-DDS (AutosoaringControl uORB).
+	// Altitude safety enforced via FW_ALT_MIN; staleness watchdog active.
+	// -------------------------------------------------------------------------
+
+	const float alt_min  = _param_fw_alt_min.get();
+	const float alt_max  = _param_fw_alt_max.get();
+	const float alt_hyst = _param_fw_alt_hyst.get();
+	const float current_altitude = _local_pos.z_global ? -_local_pos.z + _local_pos.ref_alt : _current_altitude;
+
+	const uint8_t soaring_cmd     = _autosoaring_control.soaring_mode;
+	const bool    soaring_requested = (soaring_cmd != autosoaring_control_s::SOARING_OFF);
+	const bool    glide_cmd   = (soaring_cmd == autosoaring_control_s::SOARING_GLIDE_FIXED ||
+				     soaring_cmd == autosoaring_control_s::SOARING_GLIDE_POLAR);
+	const bool    thermal_cmd = (soaring_cmd == autosoaring_control_s::SOARING_THERMAL_LOITER ||
+				     soaring_cmd == autosoaring_control_s::SOARING_THERMAL_BANK);
+
+	// Altitude floor — DDS soaring disabled below FW_ALT_MIN.
+	if (soaring_requested && current_altitude < alt_min) {
+		if (!_soaring_forbidden_latched) {
+			_soaring_forbidden_latched = true;
+			PX4_WARN("Soaring disabled: alt %.0f m below min %.0f m (FW_ALT_MIN)",
+				 (double)current_altitude, (double)alt_min);
+		}
+	}
+
+	// Clear latch when soaring is turned off (from any path)
+	if (!soaring_requested) {
+		_soaring_forbidden_latched = false;
+	}
+
+	// Altitude ceiling hysteresis (thermal → glide when ceiling reached)
+	if (_alt_max_reached && current_altitude < (alt_max - alt_hyst)) {
+		_alt_max_reached = false;
+	}
+
+	if (current_altitude >= alt_max) {
+		_alt_max_reached = true;
+	}
+
+	// soaring_allowed: DDS command is accepted only when altitude latch is clear
+	const bool soaring_allowed = soaring_requested && !_soaring_forbidden_latched;
+
+	// Effective modes: altitude ceiling forces glide even when thermal was requested.
+	// When ceiling forces a thermal→glide transition, fall back to SOARING_GLIDE_FIXED.
+	const bool effective_glide         = soaring_allowed && (glide_cmd || _alt_max_reached);
+	const bool effective_thermal       = soaring_allowed && thermal_cmd && !_alt_max_reached;
+	const bool effective_thermal_bank  = effective_thermal &&
+					     (soaring_cmd == autosoaring_control_s::SOARING_THERMAL_BANK);
+
+	const hrt_abstime now = hrt_absolute_time();
+
+	if (soaring_allowed) {
+		if (effective_thermal) {
+			// Issue DO_REPOSITION to navigator with thermal centre and loiter radius.
+			// We send on first activation (NAN guard) or when DDS provides a new explicit position.
+			// If DDS sends lat/lon=0 (meaning "loiter here"), we pin the centre at the activation
+			// point and NEVER update it again — otherwise the loiter centre would follow the
+			// aircraft every second as curr_pos changes (the moving-loiter-centre bug).
+		const bool dds_has_explicit_pos = (PX4_ISFINITE(_autosoaring_control.loiter_lat) &&
+						   fabsf(_autosoaring_control.loiter_lat) > FLT_EPSILON);
+		const double lat = dds_has_explicit_pos ? (double)_autosoaring_control.loiter_lat : curr_pos(0);
+		const double lon = dds_has_explicit_pos ? (double)_autosoaring_control.loiter_lon : curr_pos(1);
+
+			// pos_changed is true only on:
+			//   a) first activation (_soaring_last_lat is NAN) — always send once, or
+			//   b) DDS explicitly changed the coordinates (non-zero lat/lon moved > 1e-5°)
+			// When DDS lat/lon=0, pos_changed stays false after the first command → centre is pinned.
+			const bool first_activation = !PX4_ISFINITE(_soaring_last_lat);
+			const bool explicit_pos_changed = dds_has_explicit_pos &&
+							  (fabs(lat - _soaring_last_lat) > 1e-5 ||
+							   fabs(lon - _soaring_last_lon) > 1e-5);
+
+			// T3: Before the very first thermal roll, verify we have sufficient airspeed
+			// margin for the commanded bank angle.  Compute the load-factor-corrected
+			// minimum EAS and apply a 5 % stall margin.  If too slow, suppress the
+			// DO_REPOSITION this cycle; _soaring_last_lat stays NAN so we retry next cycle.
+			bool airspeed_ok_for_thermal = true;
+
+			if (first_activation && _airspeed_valid) {
+				const float bank_rad_thermal    = math::radians(_soaring_bank_angle_cmd_deg);
+				const float load_factor_thermal = 1.f / math::max(cosf(bank_rad_thermal), 0.1f);
+				const float eas_banked_floor    = _performance_model.getMinimumCalibratedAirspeed(load_factor_thermal) * 1.05f;
+
+				if (_airspeed_eas < eas_banked_floor) {
+					airspeed_ok_for_thermal = false;
+					PX4_WARN("Autosoaring: thermal entry delayed - EAS %.1f < banked-stall floor"
+						 " %.1f m/s (bank %.0f deg); waiting for speed",
+						 (double)_airspeed_eas, (double)eas_banked_floor,
+						 (double)_soaring_bank_angle_cmd_deg);
+				}
+			}
+
+			// Also re-send when turn direction flips (clockwise ↔ CCW): navigator must
+		// receive a new DO_REPOSITION with the updated radius sign.
+		// direction_changed is suppressed on first_activation (direction is already
+		// encoded in the first command).
+		const bool direction_changed = !first_activation &&
+					       (_autosoaring_control.loiter_clockwise != _soaring_last_clockwise);
+
+		const bool pos_changed = (first_activation && airspeed_ok_for_thermal) ||
+					 explicit_pos_changed || direction_changed;
+
+		if (pos_changed) {
+			if (first_activation) {
+				// Latch mission index now (AUTO_MISSION) so we can restore it after
+				// AUTO_LOITER even if mission uORB is rewound while thermalling.
+				_soaring_pending_mission_restore = false;
+				mission_s mission_snap{};
+
+					if (_mission_sub.copy(&mission_snap) && (mission_snap.count > 0)) {
+						const int32_t cs = math::constrain(mission_snap.current_seq, INT32_C(0),
+										     (int32_t)mission_snap.count - 1);
+						_soaring_mission_resume_seq   = (uint16_t)cs;
+						_soaring_mission_resume_valid = true;
+
+					} else {
+						_soaring_mission_resume_valid = false;
+					}
+				}
+
+				// Enforce minimum loiter radius from bank angle limit to avoid exceeding the
+				// maximum bank angle during thermalling.  Physics: r_min = V²/(g·tan(φ_max)).
+				// Uses companion bank_angle_cmd if provided, else FW_THERMAL_BANK param.
+				const float bank_max_rad = math::radians(_soaring_bank_angle_cmd_deg);
+				const float v_tas        = _param_fw_glide_airspd.get() * _eas2tas;
+				const float r_min_bank   = (v_tas * v_tas) /
+							    (CONSTANTS_ONE_G * math::max(tanf(bank_max_rad), 0.1f));
+
+			float effective_radius;
+			const float companion_radius = _autosoaring_control.loiter_radius_m;
+
+			if (PX4_ISFINITE(companion_radius) && companion_radius > FLT_EPSILON) {
+				// Companion specified a radius: clamp to the bank-angle floor.
+				effective_radius = math::max(companion_radius, r_min_bank);
+
+			} else {
+				// No radius from companion: use bank-angle-derived minimum.
+				// For SOARING_THERMAL_BANK the navigator loiter is just an anchor;
+				// the roll is overridden below, so the exact radius matters little.
+				effective_radius = r_min_bank;
+			}
+
+			// PX4 convention: negative loiter radius = CCW (left) turn.
+			const float signed_radius = _autosoaring_control.loiter_clockwise
+						    ? effective_radius : -effective_radius;
+
+			vehicle_command_s cmd{};
+			cmd.timestamp         = now;
+			cmd.command           = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
+			cmd.param1            = -1.f;  // keep current speed
+			cmd.param2            = 1.f;   // REPOSITION_ACTION_NORMAL → switch to AUTO_LOITER
+			cmd.param3            = signed_radius;
+			cmd.param4            = NAN;   // yaw unchanged
+				cmd.param5            = lat;
+				cmd.param6            = lon;
+				cmd.param7            = current_altitude;
+				cmd.target_system     = 1;
+				cmd.target_component  = 1;
+			_pub_vehicle_command.publish(cmd);
+			_soaring_last_lat         = lat;
+			_soaring_last_lon         = lon;
+			_soaring_last_clockwise   = _autosoaring_control.loiter_clockwise;
+			_soaring_mode_cmd_last_us = now;
+			}
+
+		} else if (effective_glide && (now - _soaring_mode_cmd_last_us) > 1_s) {
+			// Return to AUTO_MISSION to continue waypoint track while gliding (engine off).
+			vehicle_command_s cmd{};
+			cmd.timestamp        = now;
+			cmd.command          = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
+			cmd.param1           = 1.f;                                     // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
+			cmd.param2           = (float)PX4_CUSTOM_MAIN_MODE_AUTO;        // main mode
+			cmd.param3           = (float)PX4_CUSTOM_SUB_MODE_AUTO_MISSION; // sub mode
+			cmd.target_system    = 1;
+			cmd.target_component = 1;
+			_pub_vehicle_command.publish(cmd);
+			_soaring_mode_cmd_last_us = now;
+		}
+	}
+
+	// Pass consolidated gliding flag to TECS (single point of truth).
+	// All soaring modes (1–4) share engine-off / speed-on-elevator behaviour in TECS.
+	_tecs.set_gliding_mode_enabled(effective_glide || effective_thermal);
+
+	// Apply per-cycle dynamic overrides from the companion (NaN = keep PX4 default).
+	if (soaring_allowed) {
+		// Airspeed setpoint — used only by SOARING_GLIDE_FIXED (mode 1).
+		// SOARING_GLIDE_POLAR uses polar-derived EAS (set in parameters_update).
+		// Thermal modes always use FW_GLIDE_AIRSPD as the hold speed.
+		if (soaring_cmd == autosoaring_control_s::SOARING_GLIDE_FIXED &&
+		    PX4_ISFINITE(_autosoaring_control.airspeed_cmd) &&
+		    _autosoaring_control.airspeed_cmd > FLT_EPSILON) {
+			_tecs.set_gliding_airspeed_setpoint(_autosoaring_control.airspeed_cmd);
+
+		} else if (soaring_cmd == autosoaring_control_s::SOARING_GLIDE_POLAR) {
+			// Recompute polar best-glide EAS each cycle (air density may change).
+			const float a = _param_fw_polar_a.get();
+			const float b = _param_fw_polar_b.get();
+			if (a > FLT_EPSILON && b > FLT_EPSILON) {
+				_tecs.set_gliding_airspeed_setpoint(sqrtf(b / a));
+			}
+
+		} else {
+			_tecs.set_gliding_airspeed_setpoint(_param_fw_glide_airspd.get());
+		}
+
+		// Bank angle — used by thermal modes (3 and 4).
+		if (PX4_ISFINITE(_autosoaring_control.bank_angle_cmd) &&
+		    _autosoaring_control.bank_angle_cmd > FLT_EPSILON) {
+			_soaring_bank_angle_cmd_deg = _autosoaring_control.bank_angle_cmd;
+		} else {
+			_soaring_bank_angle_cmd_deg = _param_fw_thermal_bank.get();
+		}
+
+	} else {
+		_tecs.set_gliding_airspeed_setpoint(_param_fw_glide_airspd.get());
+		_soaring_bank_angle_cmd_deg = _param_fw_thermal_bank.get();
+	}
+
+	// Detect thermal→off transition: return to powered AUTO_MISSION automatically.
+	// This handles both DDS staleness watchdog expiry and explicit soaring mode changes.
+	// Condition: we were thermalling last cycle, thermal just ended, and we are NOT switching
+	// directly into glide mode (altitude-ceiling event handles that separately).
+	if (_soaring_was_thermal && !effective_thermal && !effective_glide) {
+		// Switch back to AUTO_MISSION so the navigator resumes flying the plan.
+		vehicle_command_s exit_cmd{};
+		exit_cmd.timestamp       = now;
+		exit_cmd.command         = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
+		exit_cmd.param1          = 1.f;                                   // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
+		exit_cmd.param2          = (float)PX4_CUSTOM_MAIN_MODE_AUTO;      // main mode (read as uint8 by Commander)
+		exit_cmd.param3          = (float)PX4_CUSTOM_SUB_MODE_AUTO_MISSION; // sub mode
+		exit_cmd.target_system   = 1;
+		exit_cmd.target_component = 1;
+		_pub_vehicle_command.publish(exit_cmd);
+		_soaring_forbidden_latched = false;
+	// Reset the thermal-centre cache so the NEXT thermal entry is treated as a
+	// fresh first_activation (new DO_REPOSITION + new mission-index latch).
+	_soaring_last_lat          = NAN;
+	_soaring_last_lon          = NAN;
+	_soaring_last_clockwise    = true;  // reset to default so next entry re-applies direction
+
+		if (_soaring_mission_resume_valid) {
+			_soaring_pending_mission_restore = true;
+		}
+
+		PX4_INFO("Autosoaring: thermal ended → AUTO_MISSION");
+	}
+
+	// Glide → off transition: send DO_SET_MODE → AUTO_MISSION so the navigator
+	// resumes the mission plan after an altitude-floor event or companion disable.
+	// Only fires once per transition (_soaring_was_glide guards it), same pattern as
+	// the thermal→off block above.
+	const bool soaring_was_glide_only = _soaring_was_glide && !_soaring_was_thermal;
+
+	if (soaring_was_glide_only && !effective_glide && !effective_thermal) {
+		vehicle_command_s glide_exit_cmd{};
+		glide_exit_cmd.timestamp        = now;
+		glide_exit_cmd.command          = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
+		glide_exit_cmd.param1           = 1.f;
+		glide_exit_cmd.param2           = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
+		glide_exit_cmd.param3           = (float)PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
+		glide_exit_cmd.target_system    = 1;
+		glide_exit_cmd.target_component = 1;
+		_pub_vehicle_command.publish(glide_exit_cmd);
+		_soaring_forbidden_latched = false;
+		PX4_INFO("Autosoaring: glide ended → AUTO_MISSION");
+	}
+
+	_soaring_was_glide        = effective_glide;
+	_soaring_was_thermal      = effective_thermal;
+	_soaring_was_bank_thermal = effective_thermal_bank;
+
+	// 2 Hz phase heartbeat: keep the companion informed of the FMU's current soaring phase
+	// while soaring is active. Also published when soaring is off so the companion can confirm.
+	{
+		const hrt_abstime phase_now = hrt_absolute_time();
+
+		if (phase_now - _autosoaring_status_periodic_us >= 500_ms) {
+			_autosoaring_status_periodic_us = phase_now;
+
+			uint8_t fmu_phase;
+
+			if (effective_thermal) {
+				fmu_phase = PX4_ISFINITE(_soaring_last_lat)
+					    ? autosoaring_status_s::SOARING_PHASE_THERMAL
+					    : autosoaring_status_s::SOARING_PHASE_SEARCHING;
+
+			} else if (effective_glide) {
+				fmu_phase = autosoaring_status_s::SOARING_PHASE_GLIDING;
+
+			} else {
+				fmu_phase = autosoaring_status_s::SOARING_PHASE_CRUISE;
+			}
+
+		publish_autosoaring_status(autosoaring_status_s::SOURCE_PERIODIC,
+					   _autosoaring_control.soaring_mode,
+					   0,
+					   fmu_phase);
+		}
+	}
+
 	const uint8_t position_sp_type = handle_setpoint_type(current_sp, pos_sp_next);
 
 	_position_sp_type = position_sp_type;
@@ -882,45 +1238,79 @@
 		}
 	}
 
-	switch (position_sp_type) {
-	case position_setpoint_s::SETPOINT_TYPE_IDLE: {
-			_att_sp.thrust_body[0] = 0.0f;
-			const float roll_body = 0.0f;
-			const float pitch_body = radians(_param_fw_psp_off.get());
-			const float yaw_body = 0.0f;
+	if (effective_thermal_bank) {
+		// SOARING_THERMAL_BANK (mode 4): bypass NPFG and loiter geometry entirely.
+		//
+		// Problem with the old "override after switch" approach: control_auto_loiter()
+		// calls NPFG which computes an airspeed reference that is passed to TECS.
+		// TECS pitch is then computed from that contaminated reference, so the aircraft
+		// still partially tracks the loiter circle even after the roll is overridden.
+		//
+		// Fix: skip the switch completely.  Run TECS directly with the glide airspeed
+		// setpoint; the navigator's loiter geometry has zero influence on pitch, thrust,
+		// or roll.  The loiter anchor (DO_REPOSITION) is only used to keep the navigator
+		// in AUTO_LOITER so it doesn't interfere with the mission sequence.
+		tecs_update_pitch_throttle(control_interval,
+					   current_altitude,             // hold current altitude; TECS freezes ref in glide
+					   _param_fw_glide_airspd.get(), // glide hold speed; NPFG airspeed ref NOT used
+					   radians(_param_fw_p_lim_min.get()),
+					   radians(_param_fw_p_lim_max.get()),
+					   _param_fw_thr_min.get(),      // TECS returns throttle_min (=0) in glide
+					   _param_fw_thr_max.get(),
+					   _param_sinkrate_target.get(),
+					   _param_climbrate_target.get(),
+					   false);
+
+		// Direct roll from companion bank angle; pitch from TECS; yaw uncontrolled
+		// (coordinated turn maintained by the rudder/yaw-rate controller).
+		// Sign encodes turn direction: positive = right/clockwise, negative = left/CCW.
+		const float bank_sign = _autosoaring_control.loiter_clockwise ? 1.f : -1.f;
+		const float bank_rad  = bank_sign * math::radians(fabsf(_soaring_bank_angle_cmd_deg));
+		const Quatf q_bank(Eulerf(bank_rad, get_tecs_pitch(), _yaw));
+		q_bank.copyTo(_att_sp.q_d);
+		_att_sp.thrust_body[0] = get_tecs_thrust(); // throttle ramp below may override this
 
-			const Quatf setpoint(Eulerf(roll_body, pitch_body, yaw_body));
-			setpoint.copyTo(_att_sp.q_d);
+	} else {
+		switch (position_sp_type) {
+		case position_setpoint_s::SETPOINT_TYPE_IDLE: {
+				_att_sp.thrust_body[0] = 0.0f;
+				const float roll_body = 0.0f;
+				const float pitch_body = radians(_param_fw_psp_off.get());
+				const float yaw_body = 0.0f;
+
+				const Quatf setpoint(Eulerf(roll_body, pitch_body, yaw_body));
+				setpoint.copyTo(_att_sp.q_d);
+				break;
+			}
+
+		case position_setpoint_s::SETPOINT_TYPE_POSITION:
+			control_auto_position(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);
 			break;
-		}
 
-	case position_setpoint_s::SETPOINT_TYPE_POSITION:
-		control_auto_position(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);
-		break;
-
-	case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
-		control_auto_velocity(control_interval, curr_pos, ground_speed, current_sp);
-		break;
+		case position_setpoint_s::SETPOINT_TYPE_VELOCITY:
+			control_auto_velocity(control_interval, curr_pos, ground_speed, current_sp);
+			break;
 
-	case position_setpoint_s::SETPOINT_TYPE_LOITER:
+		case position_setpoint_s::SETPOINT_TYPE_LOITER:
 #ifdef CONFIG_FIGURE_OF_EIGHT
-		if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
-			controlAutoFigureEight(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);
+			if (current_sp.loiter_pattern == position_setpoint_s::LOITER_TYPE_FIGUREEIGHT) {
+				controlAutoFigureEight(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp);
 
-		} else
+			} else
 #endif // CONFIG_FIGURE_OF_EIGHT
-		{
-			control_auto_loiter(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp, pos_sp_next);
+			{
+				control_auto_loiter(control_interval, curr_pos, ground_speed, pos_sp_prev, current_sp, pos_sp_next);
+			}
 
+			break;
 		}
-
-		break;
 	}
 
 #ifdef CONFIG_FIGURE_OF_EIGHT
 
 	/* reset loiter state */
-	if ((position_sp_type != position_setpoint_s::SETPOINT_TYPE_LOITER) ||
+	if (effective_thermal_bank ||
+	    (position_sp_type != position_setpoint_s::SETPOINT_TYPE_LOITER) ||
 	    ((position_sp_type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
 	     (current_sp.loiter_pattern != position_setpoint_s::LOITER_TYPE_FIGUREEIGHT))) {
 		_figure_eight.resetPattern();
@@ -929,13 +1319,88 @@
 #endif // CONFIG_FIGURE_OF_EIGHT
 
 	/* Copy thrust output for publication, handle special cases */
+
+	// ------------------------------------------------------------------
+	// Symmetric glide entry/exit throttle ramps  (FW_GLIDE_RAMP_T)
+	//
+	// Entry ramp (powered → glide):
+	//   Ramp throttle from the last powered value DOWN to 0 over FW_GLIDE_RAMP_T seconds.
+	//   Prevents the abrupt propwash collapse that causes a nose-down pitch transient
+	//   before the speed-on-elevator controller can react.
+	//
+	// Exit ramp (glide → powered):
+	//   Ramp throttle from 0 UP to TECS demand over FW_GLIDE_RAMP_T seconds.
+	//   Prevents the propwash surge that causes a nose-up pitch transient on restart.
+	// ------------------------------------------------------------------
+	const bool currently_soaring = _tecs.get_gliding_mode_enabled();
+	const float ramp_t = _param_fw_glide_ramp_t.get();
+
+	// ---- Cache pre-glide thrust (must run before _soaring_was_active is updated) ----
+	// This stores the last powered TECS thrust so the entry ramp knows where to start from.
+	// The 1-cycle lag (we save cycle N-1 and use it at cycle N) is negligible for a 2 s ramp.
+	if (!currently_soaring && !_landed) {
+		_soaring_pre_glide_thrust = get_tecs_thrust();
+		_soaring_entry_ramp_start_us = 0;  // clear any stale entry timer
+
+	} else if (!_soaring_was_active && currently_soaring && _soaring_entry_ramp_start_us == 0) {
+		// Glide just activated this cycle: start the entry ramp.
+		_soaring_entry_ramp_start_us = hrt_absolute_time();
+	}
+
+	// ---- Exit ramp: soaring → powered ----
+	if (currently_soaring) {
+		_soaring_ramp_start_us = 0;  // keep exit timer clear while gliding
+
+	} else if (_soaring_was_active && _soaring_ramp_start_us == 0) {
+		_soaring_ramp_start_us = hrt_absolute_time();  // glide just ended — start exit ramp
+	}
+
+	// Exit ramp scale: 0 → 1 over ramp_t seconds (1.0 = full TECS thrust)
+	float exit_ramp_scale = 1.0f;
+
+	if (_soaring_ramp_start_us > 0 && ramp_t > FLT_EPSILON) {
+		const float t_elapsed = (float)(hrt_absolute_time() - _soaring_ramp_start_us) * 1e-6f;
+
+		if (t_elapsed < ramp_t) {
+			exit_ramp_scale = t_elapsed / ramp_t;
+
+		} else {
+			_soaring_ramp_start_us = 0;  // exit ramp complete
+		}
+	}
+
+	// Entry ramp: compute ramped throttle (pre_glide_thrust → 0 over ramp_t)
+	// When the ramp is not active (timer=0), entry_throttle stays 0 (steady glide).
+	float entry_throttle = 0.0f;
+
+	if (_soaring_entry_ramp_start_us > 0 && ramp_t > FLT_EPSILON) {
+		const float t_elapsed = (float)(hrt_absolute_time() - _soaring_entry_ramp_start_us) * 1e-6f;
+
+		if (t_elapsed < ramp_t) {
+			entry_throttle = _soaring_pre_glide_thrust * (1.0f - t_elapsed / ramp_t);
+
+		} else {
+			_soaring_entry_ramp_start_us = 0;  // entry ramp complete → engine fully off
+		}
+	}
+
+	_soaring_was_active = currently_soaring;
+
 	if (position_sp_type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
 
 		_att_sp.thrust_body[0] = 0.0f;
 
+	} else if (currently_soaring) {
+		// During glide the engine is off.  While the entry ramp is active, throttle
+		// is deliberately non-zero (ramping down) — this is intentional and safe because
+		// the ramp value came from powered-flight TECS output.  Once the ramp completes,
+		// entry_throttle = 0, which is the steady-state engine-off condition.
+		_att_sp.thrust_body[0] = entry_throttle;
+
 	} else {
-		// when we are landed state we want the motor to spin at idle speed
-		_att_sp.thrust_body[0] = (_landed) ? min(_param_fw_thr_idle.get(), 1.f) : get_tecs_thrust();
+		// Powered flight: apply TECS thrust, scaled by the exit ramp if glide just ended.
+		const float tecs_thr = (_landed) ? min(_param_fw_thr_idle.get(), 1.f) : get_tecs_thrust();
+		_att_sp.thrust_body[0] = tecs_thr * exit_ramp_scale;
 	}
 
 	if (!_vehicle_status.in_transition_to_fw) {
@@ -1074,16 +1539,11 @@
 	float tecs_fw_thr_min;
 	float tecs_fw_thr_max;
 
-	if (pos_sp_curr.gliding_enabled) {
-		/* enable gliding with this waypoint */
-		_tecs.set_speed_weight(2.0f);
-		tecs_fw_thr_min = 0.0;
-		tecs_fw_thr_max = 0.0;
-
-	} else {
-		tecs_fw_thr_min = _param_fw_thr_min.get();
-		tecs_fw_thr_max = _param_fw_thr_max.get();
-	}
+	// Soaring glide/thermal mode is now handled internally by TECS (gliding_mode_enabled flag).
+	// Standard throttle limits are used here; TECS _calcThrottleControlOutput returns throttle_min
+	// when gliding, and _updateSpeedAltitudeWeights switches to speed-on-elevator (w_ske=2).
+	tecs_fw_thr_min = _param_fw_thr_min.get();
+	tecs_fw_thr_max = _param_fw_thr_max.get();
 
 	// waypoint is a plain navigation waypoint
 	float position_sp_alt = pos_sp_curr.alt;
@@ -1167,16 +1627,11 @@
 	float tecs_fw_thr_min;
 	float tecs_fw_thr_max;
 
-	if (pos_sp_curr.gliding_enabled) {
-		/* enable gliding with this waypoint */
-		_tecs.set_speed_weight(2.0f);
-		tecs_fw_thr_min = 0.0;
-		tecs_fw_thr_max = 0.0;
-
-	} else {
-		tecs_fw_thr_min = _param_fw_thr_min.get();
-		tecs_fw_thr_max = _param_fw_thr_max.get();
-	}
+	// Soaring glide/thermal mode is now handled internally by TECS (gliding_mode_enabled flag).
+	// Standard throttle limits are used here; TECS _calcThrottleControlOutput returns throttle_min
+	// when gliding, and _updateSpeedAltitudeWeights switches to speed-on-elevator (w_ske=2).
+	tecs_fw_thr_min = _param_fw_thr_min.get();
+	tecs_fw_thr_max = _param_fw_thr_max.get();
 
 	// waypoint is a plain navigation waypoint
 	float position_sp_alt = pos_sp_curr.alt;
@@ -1251,16 +1706,11 @@
 	float tecs_fw_thr_min;
 	float tecs_fw_thr_max;
 
-	if (pos_sp_curr.gliding_enabled) {
-		/* enable gliding with this waypoint */
-		_tecs.set_speed_weight(2.0f);
-		tecs_fw_thr_min = 0.0;
-		tecs_fw_thr_max = 0.0;
-
-	} else {
-		tecs_fw_thr_min = _param_fw_thr_min.get();
-		tecs_fw_thr_max = _param_fw_thr_max.get();
-	}
+	// Soaring glide/thermal mode is now handled internally by TECS (gliding_mode_enabled flag).
+	// Standard throttle limits are used here; TECS _calcThrottleControlOutput returns throttle_min
+	// when gliding, and _updateSpeedAltitudeWeights switches to speed-on-elevator (w_ske=2).
+	tecs_fw_thr_min = _param_fw_thr_min.get();
+	tecs_fw_thr_max = _param_fw_thr_max.get();
 
 	/* waypoint is a loiter waypoint */
 	float loiter_radius = pos_sp_curr.loiter_radius;
@@ -1378,16 +1828,11 @@
 	float tecs_fw_thr_min;
 	float tecs_fw_thr_max;
 
-	if (pos_sp_curr.gliding_enabled) {
-		/* enable gliding with this waypoint */
-		_tecs.set_speed_weight(2.0f);
-		tecs_fw_thr_min = 0.0;
-		tecs_fw_thr_max = 0.0;
-
-	} else {
-		tecs_fw_thr_min = _param_fw_thr_min.get();
-		tecs_fw_thr_max = _param_fw_thr_max.get();
-	}
+	// Soaring glide/thermal mode is now handled internally by TECS (gliding_mode_enabled flag).
+	// Standard throttle limits are used here; TECS _calcThrottleControlOutput returns throttle_min
+	// when gliding, and _updateSpeedAltitudeWeights switches to speed-on-elevator (w_ske=2).
+	tecs_fw_thr_min = _param_fw_thr_min.get();
+	tecs_fw_thr_max = _param_fw_thr_max.get();
 
 	const bool is_low_height = checkLowHeightConditions();
 
@@ -1433,16 +1878,11 @@
 	float tecs_fw_thr_min;
 	float tecs_fw_thr_max;
 
-	if (pos_sp_curr.gliding_enabled) {
-		/* enable gliding with this waypoint */
-		_tecs.set_speed_weight(2.0f);
-		tecs_fw_thr_min = 0.0;
-		tecs_fw_thr_max = 0.0;
-
-	} else {
-		tecs_fw_thr_min = _param_fw_thr_min.get();
-		tecs_fw_thr_max = _param_fw_thr_max.get();
-	}
+	// Soaring glide/thermal mode is now handled internally by TECS (gliding_mode_enabled flag).
+	// Standard throttle limits are used here; TECS _calcThrottleControlOutput returns throttle_min
+	// when gliding, and _updateSpeedAltitudeWeights switches to speed-on-elevator (w_ske=2).
+	tecs_fw_thr_min = _param_fw_thr_min.get();
+	tecs_fw_thr_max = _param_fw_thr_max.get();
 
 	// waypoint is a plain navigation waypoint
 	float target_airspeed = adapt_airspeed_setpoint(control_interval, pos_sp_curr.cruising_speed,
@@ -2480,6 +2920,43 @@
 			parameters_update();
 		}
 
+		// AutosoaringControl: poll ROS2 companion commands (published via XRCE-DDS)
+		autosoaring_control_s soaring_msg;
+
+		if (_autosoaring_control_sub.update(&soaring_msg)) {
+			const hrt_abstime now = hrt_absolute_time();
+			const bool prev_thermal = (
+				_autosoaring_control.soaring_mode == autosoaring_control_s::SOARING_THERMAL_LOITER ||
+				_autosoaring_control.soaring_mode == autosoaring_control_s::SOARING_THERMAL_BANK);
+			const bool new_thermal = (
+				soaring_msg.soaring_mode == autosoaring_control_s::SOARING_THERMAL_LOITER ||
+				soaring_msg.soaring_mode == autosoaring_control_s::SOARING_THERMAL_BANK);
+			_autosoaring_control      = soaring_msg;
+			_autosoaring_last_recv_us = now;
+
+			// If thermal just became active reset the DO_REPOSITION debounce.
+			if (!prev_thermal && new_thermal) {
+				_soaring_mode_cmd_last_us = 0;
+				_soaring_last_lat         = NAN;
+				_soaring_last_lon         = NAN;
+			}
+		}
+
+		// Staleness watchdog: companion silent >2 s → disable soaring.
+		if (_autosoaring_last_recv_us > 0 &&
+		    (hrt_absolute_time() - _autosoaring_last_recv_us) > 2_s) {
+			if (_autosoaring_control.soaring_mode != autosoaring_control_s::SOARING_OFF) {
+				PX4_WARN("Autosoaring: companion silent >2 s - disabling soaring");
+				_autosoaring_control.soaring_mode = autosoaring_control_s::SOARING_OFF;
+			publish_autosoaring_status(autosoaring_status_s::SOURCE_STALENESS_WATCHDOG,
+						   autosoaring_control_s::SOARING_OFF, 0,
+						   autosoaring_status_s::SOARING_PHASE_CRUISE);
+			}
+
+			_tecs.set_gliding_mode_enabled(false);
+			_autosoaring_last_recv_us = 0;
+		}
+
 		vehicle_global_position_s gpos;
 
 		if (_global_pos_sub.update(&gpos)) {
```

### `src/modules/fw_pos_control/fw_path_navigation_params.c` (modified)

```diff
--- a/src/modules/fw_pos_control/fw_path_navigation_params.c
+++ b/src/modules/fw_pos_control/fw_path_navigation_params.c
@@ -952,3 +952,154 @@
  * @group FW Attitude Control
  */
 PARAM_DEFINE_FLOAT(FW_SPOILERS_LND, 0.f);
+
+// ============================================================================
+// SOARING MODE PARAMETERS
+// ============================================================================
+
+/**
+ * Minimum altitude for soaring mode
+ *
+ * Below this altitude soaring is disabled and normal powered mission is forced.
+ * A latch prevents re-enabling until the companion computer publishes both flags false.
+ *
+ * @unit m
+ * @min 50.0
+ * @max 1000.0
+ * @decimal 0
+ * @increment 10
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_ALT_MIN, 100.0f);
+
+/**
+ * Maximum altitude for soaring mode
+ *
+ * When this altitude is reached during thermal mode the aircraft switches to glide mode.
+ * Returning below FW_ALT_MAX - FW_ALT_HYST re-enables thermal mode.
+ *
+ * @unit m
+ * @min 100.0
+ * @max 2000.0
+ * @decimal 0
+ * @increment 10
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_ALT_MAX, 500.0f);
+
+/**
+ * Soaring altitude hysteresis
+ *
+ * Hysteresis band below FW_ALT_MAX used to re-enable thermal mode after
+ * the altitude ceiling is reached.
+ *
+ * @unit m
+ * @min 10.0
+ * @max 100.0
+ * @decimal 1
+ * @increment 5.0
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_ALT_HYST, 20.0f);
+
+/**
+ * Gliding airspeed setpoint
+ *
+ * Fixed equivalent airspeed (EAS) target used by TECS during SOARING_GLIDE_FIXED (mode 1)
+ * and as the hold speed during thermal modes (modes 3 and 4).
+ * For polar best-glide (SOARING_GLIDE_POLAR, mode 2) this parameter is ignored and
+ * EAS is computed from sqrt(FW_POLAR_B / FW_POLAR_A) instead.
+ * TECS internally converts EAS to TAS using the current air-density ratio.
+ *
+ * @unit m/s
+ * @min 5.0
+ * @max 50.0
+ * @decimal 1
+ * @increment 0.5
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_GLIDE_AIRSPD, 10.0f);
+
+/**
+ * Glide polar parabolic coefficient a  (sink = a*V^2 + b)
+ *
+ * Fit from flight-test data at ≥3 airspeeds (e.g. 12, 15, 20 m/s):
+ * log sink rate vs airspeed and fit a parabola.
+ * Used to enforce a physics-based sink-rate limit in the altitude control output.
+ *
+ * @unit norm
+ * @min 0.0001
+ * @max 0.05
+ * @decimal 5
+ * @increment 0.0001
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_POLAR_A, 0.003f);
+
+/**
+ * Glide polar constant term b  (sink = a*V^2 + b)
+ *
+ * Minimum sink rate at best-glide airspeed.
+ * Fit from level glide flight-test data.
+ *
+ * @unit m/s
+ * @min 0.1
+ * @max 5.0
+ * @decimal 2
+ * @increment 0.05
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_POLAR_B, 0.50f);
+
+/**
+ * Throttle ramp time on glide exit
+ *
+ * Time [s] over which throttle ramps from 0 to the TECS demand after
+ * leaving gliding mode. Prevents sudden propwash disturbance and
+ * the associated pitch transient on engine restart.
+ *
+ * @unit s
+ * @min 0.5
+ * @max 10.0
+ * @decimal 1
+ * @increment 0.5
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_GLIDE_RAMP_T, 2.0f);
+
+/**
+ * Maximum bank angle during thermalling loiter
+ *
+ * Steeper bank = tighter loiter circle = better thermal centering.
+ * Constrained by airspeed and stall margin. The minimum safe radius
+ * is enforced in fw_pos_control (r_min = V^2 / (g * tan(bank_max))).
+ *
+ * @unit deg
+ * @min 20.0
+ * @max 60.0
+ * @decimal 1
+ * @increment 1.0
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_THERMAL_BANK, 40.0f);
+
+/**
+ * Throttle integrator decay time constant during gliding
+ *
+ * During engine-off gliding the TECS throttle integrator is not updated
+ * but instead decays exponentially with this time constant so it returns
+ * to zero before engine restart.  Prevents a throttle surge on glide exit.
+ *
+ * Derivation: I(t) = I(0)*exp(-t/tau).  After one tau the integrator is
+ * at 37% of its entry value; after 3*tau it is at 5%.  Choose tau so that
+ * a typical glide segment (30-120 s) leaves the integrator near zero.
+ * Default 10 s gives ~5% residual after 30 s and ~0% after 60 s.
+ *
+ * @unit s
+ * @min 1.0
+ * @max 60.0
+ * @decimal 1
+ * @increment 0.5
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_GLIDE_I_DECAY, 10.0f);
```

### `src/modules/commander/Commander.cpp` (modified)

```diff
--- a/src/modules/commander/Commander.cpp
+++ b/src/modules/commander/Commander.cpp
@@ -751,7 +751,15 @@
 			const bool mode_switch_not_requested = (change_mode_flags & 1) == 0;
 			const bool unsupported_bits_set = (change_mode_flags & ~1) != 0;
 
-			if (mode_switch_not_requested || unsupported_bits_set) {
+			// Soaring: param2==1 requests a mode switch to LOITER; param2==0 means
+			// position-only update without a mode change (navigator will process it regardless).
+			// The original stock logic rejected param2==0 with UNSUPPORTED; we now pass it
+			// through so companion-initiated thermal loiter repositions work correctly.
+			if (mode_switch_not_requested) {
+				// Position-only update: accept and let navigator process; no mode switch needed
+				cmd_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
+
+			} else if (unsupported_bits_set) {
 				answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
 
 			} else {
```

### `src/modules/navigator/mission.cpp` (modified)

```diff
--- a/src/modules/navigator/mission.cpp
+++ b/src/modules/navigator/mission.cpp
@@ -122,6 +122,7 @@
 
 		// User has actively set new index, reset.
 		_inactivation_index = -1;
+		_mission_paused_seq = -1;
 
 		return true;
 	}
```

### `src/modules/navigator/mission_base.h` (modified)

```diff
--- a/src/modules/navigator/mission_base.h
+++ b/src/modules/navigator/mission_base.h
@@ -331,6 +331,7 @@
 	mission_s _mission;					/**< Currently active mission*/
 	float _mission_init_climb_altitude_amsl{NAN}; 		/**< altitude AMSL the vehicle will climb to when mission starts */
 	int _inactivation_index{-1}; // index of mission item at which the mission was paused. Used to resume survey missions at previous waypoint to not lose images.
+	int32_t _mission_paused_seq{-1}; /**< current_seq at mission deactivation (e.g. AUTO_LOITER); prevents stale mission uORB from rewinding progress until resume */
 	int _mission_activation_index{-1};					/**< Index of the mission item that will bring the vehicle back to a mission waypoint */
 	bool _speed_replayed_on_activation{false};			/**< Flag indicating if the speed change items have been replayed on activation */
 
```

### `src/modules/navigator/mission_base.cpp` (modified)

```diff
--- a/src/modules/navigator/mission_base.cpp
+++ b/src/modules/navigator/mission_base.cpp
@@ -95,13 +95,26 @@
 		_mission_sub.update(&new_mission);
 
 		const bool mission_items_changed = (new_mission.mission_id != _mission.mission_id);
-		const bool mission_data_changed = checkMissionDataChanged(new_mission);
 
 		if (new_mission.current_seq < 0) {
 			new_mission.current_seq = math::constrain(_mission.current_seq, INT32_C(0),
 						  static_cast<int32_t>(new_mission.count) - 1);
 		}
 
+		// Mission uORB is still published while the mission *mode* is inactive (AUTO_LOITER during
+		// thermalling, etc.). Duplicate/stale publications with a lower current_seq rewind the
+		// stored mission state; on resume AUTO_MISSION flies an already-passed item. Hold at
+		// least the sequence captured in on_inactivation() until on_activation() clears it.
+		// Explicit jumps via set_current_mission_index() clear _mission_paused_seq; a new mission
+		// plan clears it in onMissionUpdate().
+		if (!isActive() && (_mission_paused_seq >= 0) && !mission_items_changed
+		    && (new_mission.mission_dataman_id == _mission.mission_dataman_id)
+		    && (new_mission.current_seq < _mission_paused_seq)) {
+			new_mission.current_seq = _mission_paused_seq;
+		}
+
+		const bool mission_data_changed = checkMissionDataChanged(new_mission);
+
 		if (new_mission.geofence_id != _mission.geofence_id) {
 			// New geofence data, need to check mission again.
 			_mission_checked = false;
@@ -124,6 +137,7 @@
 	if (has_mission_items_changed) {
 		_dataman_cache.invalidate();
 		_load_mission_index = -1;
+		_mission_paused_seq = -1;
 
 		if (canRunMissionFeasibility()) {
 			_mission_checked = true;
@@ -196,6 +210,7 @@
 	_mission_type = MissionType::MISSION_TYPE_NONE;
 
 	_inactivation_index = _mission.current_seq;
+	_mission_paused_seq = _mission.current_seq;
 }
 
 void
@@ -246,10 +261,22 @@
 	}
 
 	checkClimbRequired(_mission.current_seq);
+
+	// Clear the position setpoint triplet inherited from the previous navigation mode
+	// (e.g. loiter setpoint from thermal circling) before building the mission triplet.
+	// Without this, setActiveMissionItems() snapshots the loiter position into
+	// pos_sp_triplet->previous, which the FW guidance uses as the start of the track
+	// line.  That makes the aircraft approach the next waypoint from the thermal centre
+	// direction and — when the centre is near the previous mission waypoint — visually
+	// appear to "go back" to that waypoint.  Resetting here is consistent with what
+	// Mission::set_current_mission_index() already does when active.
+	_navigator->reset_triplets();
+
 	set_mission_items();
 
 	_mission_activation_index = _mission.current_seq;
 	_inactivation_index = -1; // reset
+	_mission_paused_seq = -1;
 
 	// reset cruise speed
 	_navigator->reset_cruising_speed();
@@ -753,6 +780,7 @@
 	    && ((_mission.current_seq + 1) == _mission.count)) {
 		setMissionIndex(0);
 		_inactivation_index = -1; // reset
+		_mission_paused_seq = -1;
 		_is_current_planned_mission_item_valid = isMissionValid();
 		resetMissionJumpCounter();
 		_navigator->reset_cruising_speed();
@@ -1226,6 +1254,7 @@
 	}
 
 	/* Set a new mission*/
+	_mission_paused_seq = -1;
 	_mission.timestamp = hrt_absolute_time();
 	_mission.current_seq = 0;
 	_mission.land_start_index = -1;
```

### `src/modules/navigator/navigator_main.cpp` (modified)

```diff
--- a/src/modules/navigator/navigator_main.cpp
+++ b/src/modules/navigator/navigator_main.cpp
@@ -392,17 +392,30 @@
 							rep->current.loiter_pattern = position_setpoint_s::LOITER_TYPE_ORBIT;
 						}
 
-						rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;
-					}
+					rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;
+				}
 
-					rep->previous.timestamp = hrt_absolute_time();
+				rep->previous.timestamp = hrt_absolute_time(); // stamp the "previous" leg
 
-					rep->current.valid = true;
-					rep->current.timestamp = hrt_absolute_time();
+				rep->current.valid = true;  // ← CRITICAL: Loiter checks this flag
+				rep->current.timestamp = hrt_absolute_time(); // ← CRITICAL: Loiter checks age < 500ms
 
-					rep->next.valid = false;
-
-					_time_loitering_after_gf_breach = 0; // have to manually reset this in all LOITER cases
+				rep->next.valid = false;
+
+				// Honour explicit loiter radius and direction in param3.
+				// Convention: |param3| = radius [m]; sign encodes direction:
+				//   param3 > 0  → clockwise (CW)   loiter_direction_counter_clockwise = false
+				//   param3 < 0  → counter-clockwise loiter_direction_counter_clockwise = true
+				//   param3 == 0 or NaN → use NAV_LOITER_RAD default, keep existing direction
+				if (PX4_ISFINITE(cmd.param3) && fabsf(cmd.param3) > FLT_EPSILON) {
+					rep->current.loiter_radius = fabsf(cmd.param3);
+					rep->current.loiter_direction_counter_clockwise = (cmd.param3 < 0.f);
+
+				} else if (!only_alt_change_requested) {
+					rep->current.loiter_radius = get_loiter_radius();
+					// loiter_direction_counter_clockwise stays at navigator default (false = CW)
+				}
+				_time_loitering_after_gf_breach = 0; // have to manually reset this in all LOITER cases
 
 				} else {
 					mavlink_log_critical(&_mavlink_log_pub, "Reposition is outside geofence\t");
```

### `src/modules/uxrce_dds_client/dds_topics.yaml` (modified)

```diff
--- a/src/modules/uxrce_dds_client/dds_topics.yaml
+++ b/src/modules/uxrce_dds_client/dds_topics.yaml
@@ -74,6 +74,9 @@
   - topic: /fmu/out/airspeed_validated
     type: px4_msgs::msg::AirspeedValidated
 
+  - topic: /fmu/out/autosoaring_status
+    type: px4_msgs::msg::AutosoaringStatus
+
   - topic: /fmu/out/vtol_vehicle_status
     type: px4_msgs::msg::VtolVehicleStatus
 
@@ -130,6 +133,9 @@
   - topic: /fmu/in/trajectory_setpoint
     type: px4_msgs::msg::TrajectorySetpoint
 
+  - topic: /fmu/in/autosoaring_control
+    type: px4_msgs::msg::AutosoaringControl
+
   - topic: /fmu/in/vehicle_attitude_setpoint
     type: px4_msgs::msg::VehicleAttitudeSetpoint
 
```

