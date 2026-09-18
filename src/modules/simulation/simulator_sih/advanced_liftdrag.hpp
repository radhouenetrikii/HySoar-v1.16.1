/****************************************************************************
 *
 * AdvancedLiftDrag aerodynamics for SIH fixed-wing.
 *
 * Base coefficients from Gazebo advanced_plane / AdvancedLiftDrag1, rescaled to
 * approximate Phoenix 2400 geometry:
 *   - wingspan 2.40 m
 *   - overall length 1.132 m (reference for CP / fuselage scale)
 *   - flying mass ~1.16 kg (set via SIH_MASS in airframes)
 *
 * Control moment / force surface derivatives use Gazebo advanced_plane baselines.
 * Runtime scale SIH_CTRL_EFF (default 0.12) is applied in update() so larger
 * PX4 surface commands are needed after Phoenix area×span growth.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

#include <cmath>

class AdvancedLiftDrag
{
public:
	struct Config {
		// Phoenix 2400–scaled geometry (AR kept 6.5 → area = b²/AR)
		float area{0.886f};   // m²
		float mac{0.369f};    // m
		float ar{6.5f};
		float eff{0.97f};
		float rho{1.2041f};
		matrix::Vector3f cp{-0.20f, 0.f, 0.f}; // scaled with MAC vs advanced_plane

		// Longitudinal coefficients
		float cl0{0.15188f};
		float cla{5.015f};
		float cd0{0.029f};
		float cem0{0.075f};
		float cema{-0.463966f};
		float alpha_stall{0.3391428111f};
		float cema_stall{0.f};

		// Lateral-directional (alpha/beta derivatives)
		float cya{0.f};
		float cella{0.f};
		float cena{0.f};
		float clb{0.f};
		float cyb{-0.258244f};
		float cellb{-0.039250f};
		float cemb{0.f};
		float cenb{0.100826f};

		// Stability derivatives (model.sdf)
		float cdp{0.f};
		float cyp{0.065861f};
		float clp{0.f};
		float cellp{-0.487407f};
		float cemp{0.f};
		float cenp{-0.040416f};

		float cdq{0.055166f};
		float cyq{0.f};
		float clq{7.971792f};
		float cellq{0.f};
		float cemq{-12.140140f};
		float cenq{0.f};

		float cdr{0.f};
		float cyr{0.230299f};
		float clr{0.f};
		float cellr{0.078165f};
		float cemr{0.f};
		float cenr{-0.089947f};

		// Stall blend (plugin defaults)
		float blend_m{15.f};
		float cd_fp_k1{-0.224f};
		float cd_fp_k2{-0.115f};

		// Four control surfaces: left elevon, right elevon, elevator, rudder
		// Baselines from Gazebo advanced_plane; scaled at runtime by SIH_CTRL_EFF
		static constexpr int NUM_CTRL = 4;
		float ctrl_dir[NUM_CTRL] {1.f, 1.f, -1.f, 1.f};
		float cd_ctrl[NUM_CTRL] {-0.000059f, -0.000059f, 0.000274f, 0.f};
		float cy_ctrl[NUM_CTRL] {0.000171f, -0.000171f, 0.f, -0.003913f};
		float cl_ctrl[NUM_CTRL] {-0.011940f, -0.011940f, 0.010696f, 0.f};
		float cell_ctrl[NUM_CTRL] {-0.003331f, 0.003331f, 0.f, -0.000257f};
		float cem_ctrl[NUM_CTRL] {0.001498f, 0.001498f, -0.025798f, 0.f};
		float cen_ctrl[NUM_CTRL] {-0.000057f, 0.000057f, 0.f, 0.001613f};
	};

	struct Output {
		matrix::Vector3f Fa_B{};
		matrix::Vector3f Ma_B{};
	};

	static Config advancedPlaneConfig()
	{
		return Config{};
	}

	/** Body FRD: v_B = airflow velocity relative to body [m/s], w_B body rates [rad/s].
	 *  Control cmds are normalized actuator outputs in [-1, 1]; max_def_rad scales to ±deflection.
	 *  ctrl_eff_scale multiplies all control-surface aero contributions (SIH_CTRL_EFF).
	 *  w_cg, w_left_tip, w_right_tip: Allen updraft [m/s] for differential thermal roll moment.
	 */
	Output update(const Config &cfg, const matrix::Vector3f &v_B, const matrix::Vector3f &w_B,
		      float roll_cmd, float pitch_cmd, float yaw_cmd, float max_def_rad,
		      float w_cg, float w_left_tip, float w_right_tip,
		      float ctrl_eff_scale = 1.f) const
	{
		Output out{};

		const matrix::Vector3f body_x(1.f, 0.f, 0.f);
		const matrix::Vector3f body_y(0.f, 1.f, 0.f);
		const matrix::Vector3f body_z(0.f, 0.f, 1.f);

		const float span = sqrtf(cfg.ar * cfg.area);
		const matrix::Vector3f vel_ld = v_B - body_y * v_B.dot(body_y);
		const float speed = vel_ld.norm();

		if (speed <= 1e-3f) {
			return out;
		}

		const matrix::Vector3f stability_x = vel_ld / speed;
		const matrix::Vector3f stability_y = body_y;
		const matrix::Vector3f stability_z = stability_x.cross(stability_y);

		const float alpha = atan2f(stability_x.dot(body_z), stability_x.dot(body_x));
		const float beta = atan2f(v_B.dot(body_y), v_B.dot(body_x));

		const float dyn_pres = 0.5f * cfg.rho * speed * speed;
		const float half_rho_vel = 0.5f * cfg.rho * speed;

		// SIH body rates are already FRD. Gazebo negates its native Y/Z rates
		// to obtain these same aerodynamic pitch/yaw conventions.
		const float rr = w_B(0);
		const float pr = w_B(1);
		const float yr = w_B(2);

		// Keep the dimensional rate factors exactly as AdvancedLiftDrag:
		// coefficient * (rate * reference_length / 2) * (rho * V / 2).
		// Dividing these factors by speed here would apply 1/V twice and
		// nearly remove aerodynamic damping.
		const float p_rate = rr * span * 0.5f;
		const float q_rate = pr * cfg.mac * 0.5f;
		const float r_rate = yr * span * 0.5f;

		// Stall blending (same sigmoid as Gazebo plugin)
		const float exp_pos = expf(-cfg.blend_m * (alpha - cfg.alpha_stall));
		const float exp_neg = expf(cfg.blend_m * (alpha + cfg.alpha_stall));
		const float sigma = (1.f + exp_pos + exp_neg) / ((1.f + exp_pos) * (1.f + exp_neg));

		// Differential elevons: PX4 +roll_cmd must roll right (match Gazebo Cell_ctrl signs)
		const float max_def_deg = max_def_rad * 180.f / M_PI_F;
		const float ctrl_deg[Config::NUM_CTRL] {
			-roll_cmd * max_def_deg,
			roll_cmd * max_def_deg,
			pitch_cmd * max_def_deg,
			yaw_cmd * max_def_deg
		};

		float cl_ctrl_tot = 0.f;
		float cd_ctrl_tot = 0.f;
		float cy_ctrl_tot = 0.f;
		float cell_ctrl_tot = 0.f;
		float cem_ctrl_tot = 0.f;
		float cen_ctrl_tot = 0.f;

		for (int i = 0; i < Config::NUM_CTRL; i++) {
			const float angle = ctrl_deg[i];
			const float dir = cfg.ctrl_dir[i];
			cl_ctrl_tot += angle * cfg.cl_ctrl[i] * dir;
			cd_ctrl_tot += angle * cfg.cd_ctrl[i] * dir;
			cy_ctrl_tot += angle * cfg.cy_ctrl[i] * dir;
			cell_ctrl_tot += angle * cfg.cell_ctrl[i] * dir;
			cem_ctrl_tot += angle * cfg.cem_ctrl[i] * dir;
			cen_ctrl_tot += angle * cfg.cen_ctrl[i] * dir;
		}

		const float k_ctrl = math::constrain(ctrl_eff_scale, 0.01f, 2.f);
		cl_ctrl_tot *= k_ctrl;
		cd_ctrl_tot *= k_ctrl;
		cy_ctrl_tot *= k_ctrl;
		cell_ctrl_tot *= k_ctrl;
		cem_ctrl_tot *= k_ctrl;
		cen_ctrl_tot *= k_ctrl;

		// Lift coefficient
		const float alpha_ratio = (fabsf(alpha) > 1e-6f) ? (alpha / fabsf(alpha)) : 1.f;
		const float cl_pre = cfg.cl0 + cfg.cla * alpha;
		const float cl_post = 2.f * alpha_ratio * sinf(alpha) * sinf(alpha) * cosf(alpha);
		float cl = (1.f - sigma) * cl_pre + sigma * cl_post;
		cl += cfg.clb * beta + cl_ctrl_tot;

		// Drag coefficient
		const float cd_fp = 2.f / (1.f + expf(cfg.cd_fp_k1 + cfg.cd_fp_k2 * fmaxf(cfg.ar, 1.f / cfg.ar)));
		const float cd_pre = cfg.cd0 + (cl * cl) / (M_PI_F * cfg.ar * cfg.eff);
		const float cd_post = fabsf(cd_fp * (0.5f - 0.5f * cosf(2.f * alpha)));
		float cd = (1.f - sigma) * cd_pre + sigma * cd_post;
		cd += cd_ctrl_tot;

		// Sideforce coefficient
		const float cy = cfg.cya * alpha + cfg.cyb * beta + cy_ctrl_tot;

		// Pitch moment coefficient (matches plugin lines, including Cemb assignment)
		float cem{0.f};

		if (alpha > cfg.alpha_stall) {
			cem = cfg.cem0 + (cfg.cema * cfg.alpha_stall + cfg.cema_stall * (alpha - cfg.alpha_stall));

		} else if (alpha < -cfg.alpha_stall) {
			cem = cfg.cem0 + (-cfg.cema * cfg.alpha_stall + cfg.cema_stall * (alpha + cfg.alpha_stall));

		} else {
			cem = cfg.cem0 + cfg.cema * alpha;
		}

		cem += cfg.cemb * beta;
		cem += cem_ctrl_tot;

		// Roll / yaw moment coefficients
		const float cell = cfg.cella * alpha + cfg.cellb * beta + cell_ctrl_tot;
		const float cen = cfg.cena * alpha + cfg.cenb * beta + cen_ctrl_tot;

		// Forces in body FRD [N]
		const matrix::Vector3f lift = (cl * dyn_pres
					       + cfg.clp * p_rate * half_rho_vel
					       + cfg.clq * q_rate * half_rho_vel
					       + cfg.clr * r_rate * half_rho_vel) * cfg.area * (-stability_z);

		const matrix::Vector3f drag = (cd * dyn_pres
					       + cfg.cdp * p_rate * half_rho_vel
					       + cfg.cdq * q_rate * half_rho_vel
					       + cfg.cdr * r_rate * half_rho_vel) * cfg.area * (-stability_x);

		const matrix::Vector3f sideforce = (cy * dyn_pres
						    + cfg.cyp * p_rate * half_rho_vel
						    + cfg.cyq * q_rate * half_rho_vel
						    + cfg.cyr * r_rate * half_rho_vel) * cfg.area * stability_y;

		// Moments about body origin (CG) [N·m]
		matrix::Vector3f pm = ((cem * dyn_pres)
				       + cfg.cemp * p_rate * half_rho_vel
				       + cfg.cemq * q_rate * half_rho_vel
				       + cfg.cemr * r_rate * half_rho_vel) * cfg.area * cfg.mac * body_y;

		matrix::Vector3f rm = ((cell * dyn_pres)
				       + cfg.cellp * p_rate * half_rho_vel
				       + cfg.cellq * q_rate * half_rho_vel
				       + cfg.cellr * r_rate * half_rho_vel) * cfg.area * span * body_x;

		matrix::Vector3f ym = ((cen * dyn_pres)
				       + cfg.cenp * p_rate * half_rho_vel
				       + cfg.cenq * q_rate * half_rho_vel
				       + cfg.cenr * r_rate * half_rho_vel) * cfg.area * span * body_z;

		// Differential thermal rolling moment (plugin main.cc)
		matrix::Vector3f thermal_roll_moment{};

		if (speed > 1e-3f) {
			const float dw_right = w_right_tip - w_cg;
			const float dw_left = w_left_tip - w_cg;
			const float d_alpha_right = dw_right / speed;
			const float d_alpha_left = dw_left / speed;
			const matrix::Vector3f lift_dir = -stability_z;
			const float dF_right = dyn_pres * (cfg.area / 2.f) * cfg.cla * d_alpha_right;
			const float dF_left = dyn_pres * (cfg.area / 2.f) * cfg.cla * d_alpha_left;
			const matrix::Vector3f right_arm = (span / 4.f) * body_y;
			const matrix::Vector3f left_arm = -(span / 4.f) * body_y;
			thermal_roll_moment = right_arm.cross(dF_right * lift_dir)
					      + left_arm.cross(dF_left * lift_dir);
		}

		const matrix::Vector3f force = lift + drag + sideforce;
		const matrix::Vector3f moment = pm + rm + ym + thermal_roll_moment + cfg.cp.cross(force);

		out.Fa_B = force;
		out.Ma_B = moment;
		return out;
	}
};
