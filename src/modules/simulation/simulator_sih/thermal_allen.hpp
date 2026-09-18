/****************************************************************************
 *
 * Allen (1997) convective thermal updraft, matching the Gazebo
 * AdvancedLiftDrag plugin (ComputeThermalW).
 *
 ****************************************************************************/

#pragma once

#include <math.h>
#include <stdint.h>

#include <lib/mathlib/mathlib.h>

namespace thermal_allen
{

static constexpr int MAX_THERMALS = 16;

struct Thermal {
	float lat;
	float lon;
	float zi;
	float wi;
	float lifetime;
	float birth_time;
};

struct Field {
	float sim_time_s{0.f};
	uint8_t count{0};
	bool lifecycle_enabled{false};
	Thermal thermals[MAX_THERMALS] {};
};

inline void allen_radial_constants(float r1r2, float &ka, float &kb, float &kc, float &kd)
{
	if (r1r2 < 0.1950f) {
		ka = 1.5352f; kb = 2.5826f; kc = -0.0113f; kd = 0.0008f;

	} else if (r1r2 < 0.3050f) {
		ka = 1.5265f; kb = 3.6054f; kc = -0.0176f; kd = 0.0005f;

	} else if (r1r2 < 0.4150f) {
		ka = 1.4866f; kb = 4.8354f; kc = -0.0320f; kd = 0.0001f;

	} else if (r1r2 < 0.5250f) {
		ka = 1.2042f; kb = 7.7904f; kc = 0.0848f; kd = 0.0001f;

	} else if (r1r2 < 0.6350f) {
		ka = 0.8816f; kb = 13.9720f; kc = 0.3404f; kd = 0.0001f;

	} else if (r1r2 < 0.7450f) {
		ka = 0.7067f; kb = 23.9940f; kc = 0.5689f; kd = 0.0002f;

	} else {
		ka = 0.6189f; kb = 42.7970f; kc = 0.7157f; kd = 0.0001f;
	}
}

inline float lifecycle_scale(const Thermal &th, float sim_time_s, bool enabled)
{
	if (!enabled || sim_time_s <= 0.f || th.lifetime <= 0.f) {
		return 1.f;
	}

	const float t = sim_time_s - th.birth_time;

	if (t < 0.f) {
		return 0.f;
	}

	if (t > th.lifetime) {
		return 0.f;
	}

	const float ratio = t / th.lifetime;

	if (ratio < 0.25f) {
		return ratio / 0.25f;
	}

	if (ratio < 0.75f) {
		return 1.f;
	}

	return math::max(0.f, 1.f - (ratio - 0.75f) / 0.25f);
}

/** Updraft w [m/s], positive upward. dist_xy is horizontal range to the core [m]. */
inline float updraft_w(float dist_xy, float alt_agl, float zi, float wi)
{
	if (zi < 1.f || wi <= 0.f || alt_agl < 0.f || alt_agl > zi) {
		return 0.f;
	}

	const float zzi = alt_agl / zi;
	const float rbar = 0.102f * powf(zzi, 1.f / 3.f) * (1.f - 0.25f * zzi) * zi;
	const float r2 = math::max(10.f, rbar);
	const float rr2 = dist_xy / r2;

	if (rr2 > 2.f) {
		return 0.f;
	}

	const float r1r2 = (r2 < 600.f) ? (0.0011f * r2 + 0.14f) : 0.8f;
	const float r1 = r1r2 * r2;
	const float w_bar = wi * powf(zzi, 1.f / 3.f) * (1.f - 1.1f * zzi);
	const float w_peak = (r2 > r1)
			     ? (3.f * w_bar * (r2 * r2 * r2 - r2 * r2 * r1)) / (r2 * r2 * r2 - r1 * r1 * r1)
			     : 0.f;

	float ka, kb, kc, kd;
	allen_radial_constants(r1r2, ka, kb, kc, kd);
	float ws = 1.f / (1.f + powf(fabsf(ka * (rr2 + kc)), kb)) + kd * rr2;
	ws = math::max(0.f, ws);

	const float w1 = (dist_xy > r1 && rr2 <= 2.f) ? (M_PI_F / 6.f) * sinf(M_PI_F * rr2) : 0.f;
	const float swd = (zzi > 0.5f && zzi < 0.9f) ? 2.5f * (zzi - 0.5f) : 0.f;
	const float w_d = (zzi > 0.5f && zzi < 0.9f) ? math::min(0.f, swd * w1) : 0.f;

	return ws * w_peak + w_d;
}

} // namespace thermal_allen
