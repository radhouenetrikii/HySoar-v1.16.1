# Line-by-line comparison: Music vs Pictures PX4-Autopilot

**Music (baseline):** `/home/radhouene/Music/PX4-Autopilot/`  
**Pictures (compare):** `/home/radhouene/Pictures/PX4-Autopilot/`

Unified diffs use the standard `diff -u` format: lines prefixed with `-` are from Music; lines prefixed with `+` are from Pictures; context lines have a leading space.

This document includes **every differing source file** among the requested areas (`src/`, `msg/`, `ROMFS/`, root `CMakeLists.txt`) that is **not** Python `__pycache__` bytecode.

**`Tools/`:** No differing non-cache files between the two trees (only `__pycache__/*.pyc` differed). Many paths exist only under Music as populated git submodules (e.g. jMAVSim, FlightGear bridge); those are not line-by-line comparable on disk in Pictures.

**File only in Pictures:** `msg/AutosoaringControl.msg` — full contents are included at the end.

---

## Root `CMakeLists.txt`

```diff
--- /home/radhouene/Music/PX4-Autopilot/CMakeLists.txt	2026-04-20 11:26:06.544134546 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/CMakeLists.txt	2026-04-20 10:53:38.424983181 -0400
@@ -267,7 +267,7 @@
 
 set(package-contact "px4users@googlegroups.com")
 
-set(CMAKE_CXX_STANDARD 14)
+set(CMAKE_CXX_STANDARD 17) # Required by std::optional and structured bindings used in Gazebo soaring plugins
 set(CMAKE_CXX_STANDARD_REQUIRED ON)
 set(CMAKE_C_STANDARD 11)
 set(CMAKE_C_STANDARD_REQUIRED ON)
```

## `msg/CMakeLists.txt`

```diff
--- /home/radhouene/Music/PX4-Autopilot/msg/CMakeLists.txt	2026-04-20 11:26:06.828192386 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/msg/CMakeLists.txt	2026-04-20 10:53:38.423983410 -0400
@@ -40,6 +40,7 @@
 	AckermannVelocitySetpoint.msg
 	ActionRequest.msg
 	ActuatorArmed.msg
+	AutosoaringControl.msg
 	ActuatorControlsStatus.msg
 	ActuatorOutputs.msg
 	ActuatorServosTrim.msg
```

## `ROMFS/.../4008_gz_advanced_plane`

```diff
--- /home/radhouene/Music/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_advanced_plane	2026-04-20 11:26:06.545134749 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_advanced_plane	2026-04-20 10:53:38.430981807 -0400
@@ -33,7 +33,7 @@
 
 param set-default FW_SPOILERS_LND 0.4
 
-param set-default FW_THR_MIN 0.05
+param set-default FW_THR_MIN 0.00 # 0.00 enables true engine-off gliding; PX4 default 0.05 prevents it
 param set-default FW_THR_TRIM 0.25
 param set-default FW_THR_MAX 0.6
 
```

## `src/lib/tecs/TECS.hpp`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/lib/tecs/TECS.hpp	2026-04-20 11:12:27.664817520 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/lib/tecs/TECS.hpp	2026-04-20 10:53:38.423983410 -0400
@@ -238,6 +238,13 @@
 		float load_factor;					///< Additional normal load factor.
 
 		float fast_descend;
+
+		// Soaring mode parameters
+		float gliding_airspeed_setpoint{15.0f};	///< Fixed TAS setpoint during gliding (FW_GLIDE_MODE=0) [m/s].
+		float glide_i_decay{10.0f};			///< Throttle integrator decay time constant during gliding [s]. I(t)=I(0)*exp(-t/tau).
+		float polar_a{0.003f};				///< Parabolic polar coefficient a: sink = a*V^2 + b  [s/m].
+		float polar_b{0.50f};				///< Parabolic polar constant term b: minimum sink rate  [m/s].
+		int   glide_mode_select{0};			///< 0=fixed FW_GLIDE_AIRSPD, 1=polar best-glide sqrt(b/a).
 	};
 
 	/**
@@ -283,6 +290,7 @@
 	struct Flag {
 		bool airspeed_enabled;			///< Flag if the airspeed sensor is enabled.
 		bool detect_underspeed_enabled;		///< Flag if underspeed detection is enabled.
+		bool gliding_mode_enabled{false};	///< True during engine-off soaring (glide or thermal). Modifies TECS energy weighting, throttle, and integrators.
 	};
 public:
 	TECSControl() = default;
@@ -394,7 +402,7 @@
 	 * @param[in] param are the control parametes.
 	 * @return Specific total energy rate limits in [m²/s³].
 	 */
-	STERateLimit _calculateTotalEnergyRateLimit(const Param &param) const;
+	STERateLimit _calculateTotalEnergyRateLimit(const Param &param, const Flag &flag) const;
 	/**
 	 * @brief calculate airspeed control proportional output.
 	 *
@@ -471,7 +479,7 @@
 	 * @param seb_rate is the specific energy balance rate in [m²/s³].
 	 * @param param is the control parameters.
 	 */
-	void _calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate, const Param &param);
+	void _calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate, const Param &param, const Flag &flag);
 
 	/**
 	 * @brief Calculate the pitch control output function.
@@ -597,6 +605,14 @@
 
 	void set_detect_underspeed_enabled(bool enabled) { _control_flag.detect_underspeed_enabled = enabled; };
 
+	// Soaring mode interface
+	void set_gliding_mode_enabled(bool enabled) { _control_flag.gliding_mode_enabled = enabled; }
+	bool get_gliding_mode_enabled() const { return _control_flag.gliding_mode_enabled; }
+	void set_gliding_airspeed_setpoint(float airspeed) { _control_param.gliding_airspeed_setpoint = airspeed; }
+	void set_glide_i_decay(float tau) { _control_param.glide_i_decay = math::max(tau, 0.1f); }
+	void set_glide_polar(float a, float b) { _control_param.polar_a = a; _control_param.polar_b = b; }
+	void set_glide_mode_select(int mode) { _control_param.glide_mode_select = mode; }
+
 	// setters for parameters
 	void set_airspeed_measurement_std_dev(float std_dev) {_airspeed_filter_param.airspeed_measurement_std_dev = std_dev;};
 	void set_airspeed_rate_measurement_std_dev(float std_dev) {_airspeed_filter_param.airspeed_rate_measurement_std_dev = std_dev;};
@@ -751,12 +767,18 @@
 		.throttle_slewrate = 0.0f,
 		.load_factor_correction = 0.0f,
 		.load_factor = 1.0f,
-		.fast_descend = 0.f
+		.fast_descend = 0.f,
+		.gliding_airspeed_setpoint = 15.0f,
+		.glide_i_decay = 10.0f,
+		.polar_a = 0.003f,
+		.polar_b = 0.50f,
+		.glide_mode_select = 0,
 	};
 
 	TECSControl::Flag _control_flag{
 		.airspeed_enabled = false,
 		.detect_underspeed_enabled = false,
+		.gliding_mode_enabled = false,
 	};
 
 	/**
```

## `src/lib/tecs/TECS.cpp`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/lib/tecs/TECS.cpp	2026-04-20 11:26:06.872201347 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/lib/tecs/TECS.cpp	2026-04-20 10:53:38.430981807 -0400
@@ -246,7 +246,7 @@
 
 	_pitch_setpoint = _calcPitchControlOutput(input, seb_rate, param, flag);
 
-	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};
+	const STERateLimit limit{_calculateTotalEnergyRateLimit(param, flag)};
 
 	_ste_rate_estimate_filter.reset(specific_energy_rate.spe_rate.estimate + specific_energy_rate.ske_rate.estimate);
 
@@ -302,10 +302,11 @@
 	_debug_output.throttle_integrator = _throttle_integ_state;
 }
 
-TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Param &param) const
+TECSControl::STERateLimit TECSControl::_calculateTotalEnergyRateLimit(const Param &param, const Flag &flag) const
 {
 	TECSControl::STERateLimit limit;
-	// Calculate the specific total energy rate limits from the max throttle limits
+	// Energy limits are preserved in gliding mode — they define the valid flight envelope regardless of throttle state.
+	// Gliding energy management is handled by throttle zeroing and energy weighting, not by changing these limits.
 	limit.STE_rate_max = math::max(param.max_climb_rate, FLT_EPSILON) * CONSTANTS_ONE_G;
 	limit.STE_rate_min = - math::max(param.min_sink_rate, FLT_EPSILON) * CONSTANTS_ONE_G;
 
@@ -317,7 +318,7 @@
 {
 	float airspeed_rate_output{0.0f};
 
-	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};
+	const STERateLimit limit{_calculateTotalEnergyRateLimit(param, flag)};
 
 	// calculate the demanded true airspeed rate of change based on first order response of true airspeed error
 	// if airspeed measurement is not enabled then always set the rate setpoint to zero in order to avoid constant rate setpoints
@@ -390,6 +391,17 @@
 {
 
 	SpecificEnergyWeighting weight;
+
+	// Gliding mode: full kinetic energy (speed) priority, no altitude control via pitch.
+	// Physical basis: in unpowered flight the pitch actuator controls airspeed (kinetic energy)
+	// while altitude is a consequence of the glide polar — not an independent control variable.
+	// Ref: Lambregts (1983) TECS theory; w_spe=0 w_ske=2 = "speed on elevator" configuration.
+	if (flag.gliding_mode_enabled) {
+		weight.spe_weighting = 0.0f;  // altitude not controlled by pitch in glide
+		weight.ske_weighting = 2.0f;  // full speed control via pitch
+		return weight;
+	}
+
 	// Calculate the weight applied to control of specific kinetic energy error
 	float pitch_speed_weight = constrain(param.pitch_speed_weight, 0.0f, 2.0f);
 
@@ -414,10 +426,14 @@
 void TECSControl::_calcPitchControl(float dt, const Input &input, const SpecificEnergyRates &specific_energy_rates,
 				    const Param &param, const Flag &flag)
 {
+	// Single code path for both normal and gliding modes.
+	// Gliding is handled transparently: _updateSpeedAltitudeWeights returns w_spe=0/w_ske=2
+	// when gliding_mode_enabled, so pitch acts as a pure speed-on-elevator controller.
+	// No duplicate branch needed — the weight function is the mode selector.
 	const SpecificEnergyWeighting weight{_updateSpeedAltitudeWeights(param, flag)};
 	ControlValues seb_rate{_calcPitchControlSebRate(weight, specific_energy_rates)};
 
-	_calcPitchControlUpdate(dt, input, seb_rate, param);
+	_calcPitchControlUpdate(dt, input, seb_rate, param, flag);
 	const float pitch_setpoint{_calcPitchControlOutput(input, seb_rate, param, flag)};
 
 	// Comply with the specified vertical acceleration limit by applying a pitch rate limit
@@ -458,11 +474,14 @@
 }
 
 void TECSControl::_calcPitchControlUpdate(float dt, const Input &input, const ControlValues &seb_rate,
-		const Param &param)
+		const Param &param, const Flag &flag)
 {
+	// Pitch integrator runs in all modes including gliding.
+	// Keeping it active during glide trims out steady pitch errors from CG offset or sensor bias.
+	// (Decaying it during glide would introduce persistent airspeed tracking errors.)
 	if (param.integrator_gain_pitch > FLT_EPSILON) {
 
-		// Calculate derivative from change in climb angle to rate of change of specific energy balance
+		// Normalisation: ΔSEB_rate / Δpitch ≈ TAS × g  (small-angle, SPE-dominant, Lambregts 1983)
 		const float climb_angle_to_SEB_rate = input.tas * CONSTANTS_ONE_G;
 
 		// Calculate pitch integrator input term
@@ -514,7 +533,7 @@
 void TECSControl::_calcThrottleControl(float dt, const SpecificEnergyRates &specific_energy_rates, const Param &param,
 				       const Flag &flag)
 {
-	const STERateLimit limit{_calculateTotalEnergyRateLimit(param)};
+	const STERateLimit limit{_calculateTotalEnergyRateLimit(param, flag)};
 
 	// Update STE rate estimate LP filter
 	const float STE_rate_estimate_raw = specific_energy_rates.spe_rate.estimate + specific_energy_rates.ske_rate.estimate;
@@ -570,6 +589,17 @@
 void TECSControl::_calcThrottleControlUpdate(float dt, const STERateLimit &limit, const ControlValues &ste_rate,
 		const Param &param, const Flag &flag)
 {
+	// Gliding mode: throttle integrator decays exponentially rather than accumulating.
+	// Rationale: with T=0 the STE estimate is always < STE_sp (can't add energy), so the integrator
+	// would wind up to maximum during a long glide and cause a throttle surge on engine restart.
+	// Decay law: I(t) = I(0)*exp(-t/tau), tau = param.glide_i_decay [s] (FW_GLIDE_I_DECAY).
+	// After one tau the integrator is at 37%; after 3*tau it is <5% — effectively zero.
+	if (flag.gliding_mode_enabled) {
+		const float decay = math::max(param.glide_i_decay, 0.1f);
+		_throttle_integ_state -= dt * _throttle_integ_state / decay;
+		return;
+	}
+
 	// Calculate gain scaler from specific energy rate error to throttle
 	const float STE_rate_to_throttle = 1.0f / (limit.STE_rate_max - limit.STE_rate_min);
 
@@ -603,6 +633,14 @@
 		const Param &param,
 		const Flag &flag) const
 {
+	// Gliding / thermalling: hard engine-off.
+	// Physics: T=0 → STE_rate = aerodynamic terms only → aircraft descends along polar.
+	// This is the primary soaring actuator command; the throttle gate in fw_pos_control
+	// provides a second layer of protection (defense-in-depth).
+	if (flag.gliding_mode_enabled) {
+		return param.throttle_min;
+	}
+
 	// Calculate gain scaler from specific energy rate error to throttle
 	const float STE_rate_to_throttle = 1.0f / (limit.STE_rate_max - limit.STE_rate_min);
 
@@ -675,6 +713,43 @@
 
 float TECS::calcTrueAirspeedSetpoint(float eas_to_tas, float eas_setpoint)
 {
+	if (_control_flag.gliding_mode_enabled) {
+		float tas_sp = NAN;
+
+		if (_control_param.glide_mode_select == 1) {
+			// FW_GLIDE_MODE = 1: polar best-glide speed.
+			// Parabolic polar: sink(V) = a*V^2 + b
+			// Best-glide speed minimises glide angle (sink/V):
+			//   d(sink/V)/dV = 0  →  V_bestLD = sqrt(b/a)
+			// Ref: Lissaman & Grosenbaugh (1993), Pennycuick (2008).
+			if (_control_param.polar_a > FLT_EPSILON && _control_param.polar_b > FLT_EPSILON) {
+				const float v_best_eas = sqrtf(_control_param.polar_b /
+							       _control_param.polar_a);
+				tas_sp = eas_to_tas * v_best_eas;
+			}
+
+		} else {
+			// FW_GLIDE_MODE = 0: fixed TAS setpoint (FW_GLIDE_AIRSPD).
+			if (_control_param.gliding_airspeed_setpoint > FLT_EPSILON &&
+			    PX4_ISFINITE(_control_param.gliding_airspeed_setpoint)) {
+				tas_sp = _control_param.gliding_airspeed_setpoint;
+			}
+		}
+
+		// Clamp to aircraft speed envelope [tas_min, tas_max].
+		// If the computed/configured value is outside this range it is clamped rather than
+		// rejected, so the aircraft always flies at the closest safe speed.
+		// Fallback to trim TAS only when the value is not finite (bad polar coefficients).
+		if (PX4_ISFINITE(tas_sp)) {
+			return math::constrain(tas_sp, _control_param.tas_min, _control_param.tas_max);
+		}
+
+		// Polar coefficients are invalid — fall back to trim speed.
+		PX4_WARN("TECS glide: polar coefficients invalid, falling back to trim speed");
+		return math::constrain(eas_to_tas * _control_param.equivalent_airspeed_trim,
+				       _control_param.tas_min, _control_param.tas_max);
+	}
+
 	return lerp(eas_to_tas * eas_setpoint, _control_param.tas_max, _fast_descend);
 }
 
@@ -740,8 +815,23 @@
 		_airspeed_filter.update(dt, airspeed_input, _airspeed_filter_param, _control_flag.airspeed_enabled);
 
 		// Update Reference model submodule
-		if (1.f - _fast_descend < FLT_EPSILON) {
-			// Reset the altitude reference model, while we are in fast descend.
+		if (_control_flag.gliding_mode_enabled) {
+			// Gliding: freeze the altitude reference model at the current state.
+			// Rationale: in gliding mode w_spe = 0 (speed-on-elevator weighting), so the
+			// altitude reference output (ref.alt, ref.alt_rate) is multiplied by zero inside
+			// _calcPitchControlSebRate and has NO effect on pitch.  The throttle output is
+			// also hard-zeroed in _calcThrottleControlOutput.  Running the trajectory generator
+			// is therefore pure waste and can produce confusing debug telemetry.
+			// By initialising every cycle we pin ref.alt = current altitude and ref.alt_rate = 0,
+			// which is the neutral, physically honest state for an unpowered aircraft.
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
@@ -758,6 +848,11 @@
 		control_setpoint.altitude_reference = _altitude_reference_model.getAltitudeReference();
 		control_setpoint.altitude_rate_setpoint_direct = _altitude_reference_model.getHeightRateSetpointDirect();
 		control_setpoint.tas_setpoint = calcTrueAirspeedSetpoint(eas_to_tas, EAS_setpoint);
+		// Note: the altitude rate clamp (min(..., 0)) previously applied here in gliding mode
+		// has been removed. With w_spe=0 the altitude reference output is multiplied by zero
+		// in the SEB calculation, so clamping it had no effect on pitch or throttle control.
+		// The frozen reference model above already produces alt_rate = current hgt_rate,
+		// which is the actual physical descent rate — no clamping needed.
 
 		const TECSControl::Input control_input{ .altitude = altitude,
 							.altitude_rate = hgt_rate,
@@ -781,6 +876,16 @@
 
 void TECS::_setFastDescend(const float alt_setpoint, const float alt)
 {
+	// Gliding mode: fast-descend is disabled entirely.
+	// The weight change in _updateSpeedAltitudeWeights (w_ske=2) already puts pitch in
+	// speed-control mode; activating fast_descend on top would conflict and produce
+	// inconsistent throttle blending.  Reset and return immediately.
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

## `src/modules/fw_pos_control/FixedwingPositionControl.hpp`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/modules/fw_pos_control/FixedwingPositionControl.hpp	2026-04-20 11:26:06.895206032 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/modules/fw_pos_control/FixedwingPositionControl.hpp	2026-04-20 10:53:38.430981807 -0400
@@ -72,6 +72,7 @@
 #include <uORB/Subscription.hpp>
 #include <uORB/SubscriptionCallback.hpp>
 #include <uORB/topics/airspeed_validated.h>
+#include <uORB/topics/autosoaring_control.h>    // ROS2 soaring command topic (via XRCE-DDS)
 #include <uORB/topics/flight_phase_estimation.h>
 #include <uORB/topics/landing_gear.h>
 #include <uORB/topics/launch_detection_status.h>
@@ -204,6 +205,7 @@
 	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
 
 	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
+	uORB::Subscription _autosoaring_control_sub{ORB_ID(autosoaring_control)};  // ROS2 soaring commands
 	uORB::Subscription _wind_sub{ORB_ID(wind)};
 	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
 	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
@@ -229,6 +231,7 @@
 	uORB::Publication<normalized_unsigned_setpoint_s> _flaps_setpoint_pub{ORB_ID(flaps_setpoint)};
 	uORB::Publication<normalized_unsigned_setpoint_s> _spoilers_setpoint_pub{ORB_ID(spoilers_setpoint)};
 	uORB::PublicationData<flight_phase_estimation_s> _flight_phase_estimation_pub{ORB_ID(flight_phase_estimation)};
+	uORB::Publication<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};  // mode-switch commands
 
 	manual_control_setpoint_s _manual_control_setpoint{};
 	position_setpoint_triplet_s _pos_sp_triplet{};
@@ -404,6 +407,19 @@
 
 	bool _tecs_is_running{false};
 
+	// Soaring state (driven by AutosoaringControl uORB from ROS2 companion via XRCE-DDS)
+	autosoaring_control_s _autosoaring_control{};       ///< Last received soaring command message
+	hrt_abstime  _autosoaring_last_recv_us{0};          ///< Timestamp of last valid AutosoaringControl message
+	hrt_abstime  _soaring_mode_cmd_last_us{0};          ///< Debounce: minimum 1 s between mode-switch commands
+	hrt_abstime  _soaring_dds_inhibit_until_us{0};      ///< DDS re-enable blocked until this timestamp (set by soar:off, 5 s cooldown)
+	bool         _soaring_expect_set_mode{false};       ///< True when CUSTOM_0 (soar:glide/thermal) was just received; next DO_SET_MODE is paired
+	double       _soaring_last_lat{NAN};                ///< Last thermal centre latitude sent to navigator
+	double       _soaring_last_lon{NAN};                ///< Last thermal centre longitude sent to navigator
+	bool         _alt_max_reached{false};               ///< Hysteresis state: altitude ceiling reached
+	bool         _soaring_forbidden_latched{false};     ///< Latch: soaring disabled below FW_ALT_MIN until companion re-enables
+	bool         _soaring_local_override{false};        ///< True when soaring was enabled via CLI (bypasses alt check + staleness watchdog)
+	bool         _soaring_was_thermal{false};           ///< Previous-cycle thermal state; used to detect thermal→off transition for auto-exit
+
 	// Smooths changes in the altitude tracking error time constant value
 	SlewRate<float> _tecs_alt_time_const_slew_rate;
 
@@ -1051,7 +1067,17 @@
 		(ParamFloat<px4::params::FW_TKO_AIRSPD>) _param_fw_tko_airspd,
 
 		(ParamFloat<px4::params::RWTO_PSP>) _param_rwto_psp,
-		(ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on
+		(ParamBool<px4::params::FW_LAUN_DETCN_ON>) _param_fw_laun_detcn_on,
+
+		// Soaring mode parameters (companion sends AutosoaringControl uORB)
+		(ParamFloat<px4::params::FW_GLIDE_AIRSPD>) _param_fw_glide_airspd,  ///< Fixed glide TAS setpoint [m/s] (FW_GLIDE_MODE=0)
+		(ParamFloat<px4::params::FW_ALT_MIN>)      _param_fw_alt_min,       ///< Soaring altitude floor [m]
+		(ParamFloat<px4::params::FW_ALT_MAX>)      _param_fw_alt_max,       ///< Soaring altitude ceiling [m]
+		(ParamFloat<px4::params::FW_ALT_HYST>)     _param_fw_alt_hyst,      ///< Ceiling hysteresis band [m]
+		(ParamFloat<px4::params::FW_GLIDE_I_DECAY>) _param_fw_glide_i_decay, ///< Throttle integrator decay tau [s]
+		(ParamFloat<px4::params::FW_POLAR_A>)      _param_fw_polar_a,       ///< Polar: parasitic drag coefficient a [s/m]
+		(ParamFloat<px4::params::FW_POLAR_B>)      _param_fw_polar_b,       ///< Polar: minimum sink rate b [m/s]
+		(ParamInt<px4::params::FW_GLIDE_MODE>)     _param_fw_glide_mode     ///< 0=fixed, 1=polar best-glide
 	)
 
 };
```

## `src/modules/fw_pos_control/FixedwingPositionControl.cpp`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/modules/fw_pos_control/FixedwingPositionControl.cpp	2026-04-20 11:26:06.895206032 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/modules/fw_pos_control/FixedwingPositionControl.cpp	2026-04-20 10:53:38.433981120 -0400
@@ -34,6 +34,8 @@
 #include "FixedwingPositionControl.hpp"
 
 #include <px4_platform_common/events.h>
+#include <parameters/param.h>
+#include <commander/px4_custom_mode.h>
 
 using math::constrain;
 using math::max;
@@ -136,6 +138,10 @@
 	_tecs.set_throttle_damp(_param_fw_t_thr_damping.get());
 	_tecs.set_integrator_gain_throttle(_param_fw_t_thr_integ.get());
 	_tecs.set_integrator_gain_pitch(_param_fw_t_I_gain_pit.get());
+	_tecs.set_gliding_airspeed_setpoint(_param_fw_glide_airspd.get());  // fixed glide TAS (FW_GLIDE_MODE=0)
+	_tecs.set_glide_i_decay(_param_fw_glide_i_decay.get());             // throttle integrator decay (FW_GLIDE_I_DECAY)
+	_tecs.set_glide_polar(_param_fw_polar_a.get(), _param_fw_polar_b.get()); // polar coefficients (FW_POLAR_A/B)
+	_tecs.set_glide_mode_select(_param_fw_glide_mode.get());             // 0=fixed, 1=polar best-glide (FW_GLIDE_MODE)
 	_tecs.set_throttle_slewrate(_param_fw_thr_slew_max.get());
 	_tecs.set_vertical_accel_limit(_param_fw_t_vert_acc.get());
 	_tecs.set_roll_throttle_compensation(_param_fw_t_rll2thr.get());
@@ -200,6 +206,60 @@
 				}
 			}
 
+		} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_CUSTOM_0) {
+			// CLI soaring command: param1=1→glide, param2=1→thermal, both 0→off.
+			// CLI always wins over DDS — it represents the ground operator's intent.
+			_autosoaring_control.glide_mode_enabled   = (vehicle_command.param1 > 0.5f);
+			_autosoaring_control.thermal_mode_enabled = (vehicle_command.param2 > 0.5f);
+			_autosoaring_control.loiter_radius_m      = (vehicle_command.param3 > FLT_EPSILON) ? vehicle_command.param3 : 0.0f;
+
+			const bool enabling = _autosoaring_control.glide_mode_enabled || _autosoaring_control.thermal_mode_enabled;
+
+		if (enabling) {
+			// CLI enables soaring: take local authority, clear any cooldown so DDS
+			// is also re-allowed (CLI and DDS cooperate when both want soaring).
+			_soaring_local_override       = true;
+			_soaring_dds_inhibit_until_us = 0;          // no cooldown active
+			_soaring_forbidden_latched    = false;      // CLI always clears latch
+			_autosoaring_last_recv_us     = 0;          // disable staleness watchdog
+			// Mark that the paired DO_SET_MODE (auto:mission/loiter) is soaring-related
+			_soaring_expect_set_mode      = true;
+			// Reset DO_REPOSITION debounce so the first thermal command fires immediately
+			// on the very next control_auto() cycle (not blocked by a recent glide command)
+			_soaring_mode_cmd_last_us     = 0;
+			_soaring_last_lat             = NAN;
+			_soaring_last_lon             = NAN;
+			} else {
+				// CLI soar:off: stop immediately and block DDS for 5 s.
+				// After 5 s the DDS path is automatically re-allowed so the companion
+				// can take back control without needing a CLI soar:glide first.
+				_soaring_local_override       = false;
+				_soaring_forbidden_latched    = false;
+				_soaring_expect_set_mode      = false;
+				_soaring_dds_inhibit_until_us = hrt_absolute_time() + 5_s;
+				_autosoaring_last_recv_us     = hrt_absolute_time();
+				_tecs.set_gliding_mode_enabled(false);
+			}
+
+		} else if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE) {
+			// When soaring is CLI-active and a DO_SET_MODE arrives:
+			//  - If paired with a recent CUSTOM_0 (soar:glide sends both back-to-back):
+			//    → it IS the soaring mode switch, leave soaring on.
+			//  - If standalone (user typed `commander mode auto:mission` manually):
+			//    → treat as "exit soaring, restore powered flight".
+			if (_soaring_local_override) {
+				if (_soaring_expect_set_mode) {
+					_soaring_expect_set_mode = false;  // consume the pairing — soaring stays ON
+				} else {
+					// Standalone mode switch → exit soaring
+					_autosoaring_control.glide_mode_enabled   = false;
+					_autosoaring_control.thermal_mode_enabled = false;
+					_soaring_local_override    = false;
+					_soaring_forbidden_latched = false;
+					_tecs.set_gliding_mode_enabled(false);
+					PX4_INFO("Autosoaring: mode switch → soaring disabled, powered flight restored");
+				}
+			}
 		}
 	}
 }
@@ -864,6 +924,154 @@
 	position_setpoint_s current_sp = pos_sp_curr;
 	move_position_setpoint_for_vtol_transition(current_sp);
 
+	// -------------------------------------------------------------------------
+	// Autosoaring control block
+	// -------------------------------------------------------------------------
+	// Three entry paths (priority order):
+	//   1. CLI  (`commander mode soar:glide/thermal/off`) via VEHICLE_CMD_CUSTOM_0
+	//      → _soaring_local_override=true, altitude checks bypassed (pilot in command)
+	//   2. ROS2 companion via XRCE-DDS (AutosoaringControl uORB)
+	//      → altitude safety enforced, staleness watchdog active
+	//   3. Any other mode switch (soar:off, mode change in vehicle_command_poll)
+	//      → clears flags, powered flight restored
+	//
+	// BUG FIXES vs previous version:
+	//   - Removed nav_state auto-exit guard: it fired before Commander finished the
+	//     mode switch, killing soaring on the very first cycle.
+	//   - Altitude floor (FW_ALT_MIN) is SKIPPED when _soaring_local_override=true
+	//     so CLI testing works at any altitude.
+	//   - _soaring_forbidden_latched is cleared whenever soaring is disabled (not
+	//     just when soaring_requested=false), preventing permanent lock-out.
+	// -------------------------------------------------------------------------
+
+	const float alt_min  = _param_fw_alt_min.get();
+	const float alt_max  = _param_fw_alt_max.get();
+	const float alt_hyst = _param_fw_alt_hyst.get();
+	const float current_altitude = _local_pos.z_global ? -_local_pos.z + _local_pos.ref_alt : _current_altitude;
+
+	const bool glide_cmd   = _autosoaring_control.glide_mode_enabled;
+	const bool thermal_cmd = _autosoaring_control.thermal_mode_enabled;
+	const bool soaring_requested = glide_cmd || thermal_cmd;
+
+	// Altitude floor — only enforced for DDS path (companion may lose situational awareness).
+	// CLI local override means a pilot is in control: skip altitude check.
+	if (!_soaring_local_override && soaring_requested && current_altitude < alt_min) {
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
+	// soaring_allowed: DDS path respects latch; CLI local override bypasses it
+	const bool soaring_allowed = soaring_requested && (_soaring_local_override || !_soaring_forbidden_latched);
+
+	// Effective modes: ceiling forces glide even when thermal was requested
+	bool effective_glide   = soaring_allowed && (glide_cmd || _alt_max_reached);
+	bool effective_thermal = soaring_allowed && thermal_cmd && !_alt_max_reached;
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
+			const bool dds_has_explicit_pos = (PX4_ISFINITE(_autosoaring_control.loiter_lat) &&
+							   fabsf(_autosoaring_control.loiter_lat) > FLT_EPSILON);
+			const double lat = dds_has_explicit_pos
+					   ? (double)_autosoaring_control.loiter_lat : curr_pos(0);
+			const double lon = dds_has_explicit_pos
+					   ? (double)_autosoaring_control.loiter_lon : curr_pos(1);
+
+			// pos_changed is true only on:
+			//   a) first activation (_soaring_last_lat is NAN) — always send once, or
+			//   b) DDS explicitly changed the coordinates (non-zero lat/lon moved > 1e-5°)
+			// When DDS lat/lon=0, pos_changed stays false after the first command → centre is pinned.
+			const bool first_activation = !PX4_ISFINITE(_soaring_last_lat);
+			const bool explicit_pos_changed = dds_has_explicit_pos &&
+							  (fabs(lat - _soaring_last_lat) > 1e-5 ||
+							   fabs(lon - _soaring_last_lon) > 1e-5);
+			const bool pos_changed = first_activation || explicit_pos_changed;
+
+			if (pos_changed) {
+				vehicle_command_s cmd{};
+				cmd.timestamp         = now;
+				cmd.command           = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
+				cmd.param1            = -1.f;  // keep current speed
+				cmd.param2            = 1.f;   // REPOSITION_ACTION_NORMAL → switch to AUTO_LOITER
+				cmd.param3            = (_autosoaring_control.loiter_radius_m > FLT_EPSILON)
+							? _autosoaring_control.loiter_radius_m : -1.0f;  // -1 = use NAV_LOITER_RAD
+				cmd.param4            = NAN;   // yaw unchanged
+				cmd.param5            = lat;
+				cmd.param6            = lon;
+				cmd.param7            = current_altitude;
+				cmd.target_system     = 1;
+				cmd.target_component  = 1;
+				_pub_vehicle_command.publish(cmd);
+				_soaring_last_lat         = lat;
+				_soaring_last_lon         = lon;
+				_soaring_mode_cmd_last_us = now;
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
+	// Pass consolidated flag to TECS (single point of truth)
+	_tecs.set_gliding_mode_enabled(effective_glide || effective_thermal);
+
+	// Detect thermal→off transition: return to powered AUTO_MISSION automatically.
+	// This handles both DDS staleness watchdog expiry and explicit soar:off/soar:glide switches.
+	// Condition: we were thermalling last cycle, thermal just ended, and we are NOT switching
+	// directly into glide mode (altitude-ceiling event handles that separately).
+	if (_soaring_was_thermal && !effective_thermal && !effective_glide) {
+		vehicle_command_s exit_cmd{};
+		exit_cmd.timestamp       = now;
+		exit_cmd.command         = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
+		exit_cmd.param1          = 1.f;                                   // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
+		exit_cmd.param2          = (float)PX4_CUSTOM_MAIN_MODE_AUTO;      // main mode (read as uint8 by Commander)
+		exit_cmd.param3          = (float)PX4_CUSTOM_SUB_MODE_AUTO_MISSION; // sub mode
+		exit_cmd.target_system   = 1;
+		exit_cmd.target_component = 1;
+		_pub_vehicle_command.publish(exit_cmd);
+		// Clear local override so vehicle_command_poll() does not intercept
+		// the command above as a "standalone mode switch exit soaring" (soaring already off).
+		_soaring_local_override    = false;
+		_soaring_forbidden_latched = false;
+		PX4_INFO("Autosoaring: thermal ended → AUTO_MISSION restored");
+	}
+
+	_soaring_was_thermal = effective_thermal;
+	// -------------------------------------------------------------------------
+
 	const uint8_t position_sp_type = handle_setpoint_type(current_sp, pos_sp_next);
 
 	_position_sp_type = position_sp_type;
@@ -933,6 +1141,11 @@
 
 		_att_sp.thrust_body[0] = 0.0f;
 
+	} else if (_tecs.get_gliding_mode_enabled()) {
+		// Soaring defense-in-depth: even if TECS returns a non-zero throttle due to a bug,
+		// this gate at the actuator output level guarantees T=0 during soaring.
+		_att_sp.thrust_body[0] = 0.0f;
+
 	} else {
 		// when we are landed state we want the motor to spin at idle speed
 		_att_sp.thrust_body[0] = (_landed) ? min(_param_fw_thr_idle.get(), 1.f) : get_tecs_thrust();
@@ -1074,16 +1287,11 @@
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
@@ -1167,16 +1375,11 @@
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
@@ -1251,16 +1454,11 @@
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
@@ -1378,16 +1576,11 @@
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
 
@@ -1433,16 +1626,11 @@
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
@@ -2480,6 +2668,58 @@
 			parameters_update();
 		}
 
+		// AutosoaringControl: poll ROS2 companion commands (published via XRCE-DDS)
+		autosoaring_control_s soaring_msg;
+
+		if (_autosoaring_control_sub.update(&soaring_msg)) {
+			const hrt_abstime now = hrt_absolute_time();
+			const bool dds_inhibited = (_soaring_dds_inhibit_until_us > 0 && now < _soaring_dds_inhibit_until_us);
+			const bool dds_wants_soaring = soaring_msg.glide_mode_enabled || soaring_msg.thermal_mode_enabled;
+
+			if (dds_inhibited && dds_wants_soaring) {
+				// soar:off cooldown active: ignore DDS re-enable attempts.
+				// The companion keeps streaming — we simply discard these messages.
+				// After 5 s the cooldown expires and DDS takes back control automatically.
+
+			} else {
+				// Normal DDS update: accept the message.
+				// If cooldown has expired or DDS is disabling soaring, always accept.
+				if (!dds_wants_soaring) {
+					// DDS explicitly disabled soaring → cancel any remaining cooldown
+					_soaring_dds_inhibit_until_us = 0;
+				}
+
+			// DDS is only authoritative when CLI is not holding local override
+			if (!_soaring_local_override) {
+				const bool prev_thermal = _autosoaring_control.thermal_mode_enabled;
+				_autosoaring_control      = soaring_msg;
+				_autosoaring_last_recv_us = now;
+
+				// If thermal just became active (or position changed), reset the
+				// DO_REPOSITION debounce so control_auto() fires immediately on next cycle.
+				if (!prev_thermal && soaring_msg.thermal_mode_enabled) {
+					_soaring_mode_cmd_last_us = 0;
+					_soaring_last_lat         = NAN;
+					_soaring_last_lon         = NAN;
+				}
+			}
+			}
+		}
+
+		// Staleness watchdog: companion silent >2 s → disable soaring (DDS path only).
+		// Skipped when CLI has local override (no companion expected in that mode).
+		if (!_soaring_local_override &&
+		    _autosoaring_last_recv_us > 0 &&
+		    (hrt_absolute_time() - _autosoaring_last_recv_us) > 2_s) {
+			if (_autosoaring_control.glide_mode_enabled || _autosoaring_control.thermal_mode_enabled) {
+				PX4_WARN("Autosoaring: companion silent >2 s — disabling soaring");
+				_autosoaring_control.glide_mode_enabled   = false;
+				_autosoaring_control.thermal_mode_enabled = false;
+			}
+
+			_tecs.set_gliding_mode_enabled(false);
+		}
+
 		vehicle_global_position_s gpos;
 
 		if (_global_pos_sub.update(&gpos)) {
```

## `src/modules/fw_pos_control/fw_path_navigation_params.c`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/modules/fw_pos_control/fw_path_navigation_params.c	2026-04-20 11:26:06.895206032 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/modules/fw_pos_control/fw_path_navigation_params.c	2026-04-20 10:53:38.433981120 -0400
@@ -952,3 +952,221 @@
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
+ * Fixed true airspeed target used by TECS during glide mode.
+ * Replaces the mission-supplied airspeed so the aircraft holds best-glide speed.
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
+ * Speed-on-elevator proportional gain (gliding mode)
+ *
+ * Converts SKE rate error [m²/s³] to pitch demand [rad].
+ * System is 1×1: pitch is the only actuator, airspeed is the only objective.
+ * Increase if aircraft lags airspeed setpoint. Decrease if pitch oscillates.
+ * Start with this value only; add FW_GLIDE_KI once P-only is stable.
+ *
+ *
+ * @min 0.001
+ * @max 0.5
+ * @decimal 4
+ * @increment 0.001
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_GLIDE_KP, 0.01f);
+
+/**
+ * Speed-on-elevator integral gain (gliding mode)
+ *
+ * Eliminates steady-state airspeed error in glide.
+ * A gentle 5%/s decay prevents windup from powered-flight integrator state.
+ * Set to 0 during initial tuning; add slowly after FW_GLIDE_KP is stable.
+ *
+ *
+ * @min 0.0
+ * @max 0.1
+ * @decimal 4
+ * @increment 0.001
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_GLIDE_KI, 0.005f);
+
+/**
+ * Speed-on-elevator integrator saturation limit (gliding mode)
+ *
+ * Maximum absolute integrator state [rad]. Prevents pitch windup
+ * during long glides or large persistent disturbances.
+ *
+ * @unit rad
+ * @min 0.05
+ * @max 0.5
+ * @decimal 2
+ * @increment 0.01
+ * @group FW Soaring
+ */
+PARAM_DEFINE_FLOAT(FW_GLIDE_ILIM, 0.20f);
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
+ * Glide airspeed selection mode
+ *
+ * Controls how the airspeed setpoint is chosen during unpowered gliding.
+ *
+ * 0 — Fixed: use FW_GLIDE_AIRSPD directly.
+ * 1 — Polar: compute best-glide speed V_bestLD = sqrt(FW_POLAR_B / FW_POLAR_A)
+ *     from the parabolic glide polar. Falls back to FW_AIRSPD_TRIM if the
+ *     polar coefficients are invalid or the result is out of [V_min, V_max].
+ *
+ * Mode 1 is the scientifically preferred choice: it derives the optimal
+ * range speed from first principles (aerodynamic polar), removing the need
+ * to tune FW_GLIDE_AIRSPD by hand.
+ *
+ * @min 0
+ * @max 1
+ * @value 0 Fixed (FW_GLIDE_AIRSPD)
+ * @value 1 Polar best-glide (sqrt(FW_POLAR_B/FW_POLAR_A))
+ * @group FW Soaring
+ */
+PARAM_DEFINE_INT32(FW_GLIDE_MODE, 0);
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

## `src/modules/commander/Commander.cpp`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/modules/commander/Commander.cpp	2026-04-20 11:26:06.876202162 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/modules/commander/Commander.cpp	2026-04-20 10:53:38.437980205 -0400
@@ -417,6 +417,28 @@
 				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
 						     PX4_CUSTOM_SUB_MODE_EXTERNAL1);
 
+			} else if (!strcmp(argv[1], "soar:glide")) {
+				// CLI shortcut: enable engine-off gliding in AUTO_MISSION.
+				// Sends VEHICLE_CMD_CUSTOM_0(param1=1) → fw_pos_control sets TECS gliding_mode_enabled.
+				// Also switches nav mode to AUTO_MISSION so waypoints are followed while gliding.
+				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_CUSTOM_0, 1.0f, 0.0f);
+				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
+						     PX4_CUSTOM_SUB_MODE_AUTO_MISSION);
+
+			} else if (!strcmp(argv[1], "soar:thermal")) {
+				// CLI shortcut: enable engine-off thermal loiter.
+				// Sends VEHICLE_CMD_CUSTOM_0(param2=1) → fw_pos_control sets TECS gliding_mode_enabled.
+				// Also switches nav mode to AUTO_LOITER so the aircraft orbits the thermal centre.
+				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_CUSTOM_0, 0.0f, 1.0f);
+				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
+						     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
+
+			} else if (!strcmp(argv[1], "soar:off")) {
+				// CLI shortcut: disable soaring and restore powered flight.
+				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_CUSTOM_0, 0.0f, 0.0f);
+				send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
+						     PX4_CUSTOM_SUB_MODE_AUTO_MISSION);
+
 			} else {
 				PX4_ERR("argument %s unsupported.", argv[1]);
 			}
@@ -751,7 +773,15 @@
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

## `src/modules/navigator/navigator_main.cpp`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/modules/navigator/navigator_main.cpp	2026-04-20 11:26:06.908208679 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/modules/navigator/navigator_main.cpp	2026-04-20 10:53:38.433981120 -0400
@@ -392,17 +392,25 @@
 							rep->current.loiter_pattern = position_setpoint_s::LOITER_TYPE_ORBIT;
 						}
 
-						rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;
-					}
+					rep->current.loiter_direction_counter_clockwise = curr->current.loiter_direction_counter_clockwise;
+				}
 
-					rep->previous.timestamp = hrt_absolute_time();
+				rep->previous.timestamp = hrt_absolute_time();
 
-					rep->current.valid = true;
-					rep->current.timestamp = hrt_absolute_time();
+				rep->current.valid = true;
+				rep->current.timestamp = hrt_absolute_time();
 
-					rep->next.valid = false;
+				rep->next.valid = false;
 
-					_time_loitering_after_gf_breach = 0; // have to manually reset this in all LOITER cases
+				// Honour explicit loiter radius in param3 (e.g. from AutosoaringControl via fw_pos_control).
+				// param3 > 0: use specified radius.  param3 <= 0 or NaN: use NAV_LOITER_RAD default.
+				if (PX4_ISFINITE(cmd.param3) && cmd.param3 > FLT_EPSILON) {
+					rep->current.loiter_radius = cmd.param3;
+
+				} else if (!only_alt_change_requested) {
+					rep->current.loiter_radius = get_loiter_radius();
+				}
+				_time_loitering_after_gf_breach = 0; // have to manually reset this in all LOITER cases
 
 				} else {
 					mavlink_log_critical(&_mavlink_log_pub, "Reposition is outside geofence\t");
@@ -776,6 +784,14 @@
 			_pos_sp_triplet_published_invalid_once = false;
 			navigation_mode_new = &_loiter;
 			break;
+
+		case vehicle_status_s::NAVIGATION_STATE_ORBIT:
+			// Thermal loiter: reuse the loiter navigation mode.
+			// The companion selects the centre via DO_REPOSITION (handled above);
+			// we simply need to map this nav-state to the existing loiter handler.
+			_pos_sp_triplet_published_invalid_once = false;
+			navigation_mode_new = &_loiter;
+			break;
 
 		case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
 
```

## `src/modules/uxrce_dds_client/dds_topics.yaml`

```diff
--- /home/radhouene/Music/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml	2026-04-20 11:26:06.917210512 -0400
+++ /home/radhouene/Pictures/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml	2026-04-20 10:53:38.434980891 -0400
@@ -130,6 +130,9 @@
   - topic: /fmu/in/trajectory_setpoint
     type: px4_msgs::msg::TrajectorySetpoint
 
+  - topic: /fmu/in/autosoaring_control
+    type: px4_msgs::msg::AutosoaringControl
+
   - topic: /fmu/in/vehicle_attitude_setpoint
     type: px4_msgs::msg::VehicleAttitudeSetpoint
 
```

## `msg/AutosoaringControl.msg` (only in Pictures)

This file does not exist under Music.

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
