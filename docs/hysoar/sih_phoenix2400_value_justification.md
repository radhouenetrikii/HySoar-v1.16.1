# Justification of SIH plant values (Phoenix 2400 + `SIH_CTRL_EFF`)

This note justifies every non-stock number used for the HySoar / SIH fixed-wing plant on Pixhawk and SITL. The goal is a **defensible engineering approximation**, not a wind-tunnel identification of the Volantex / Phoenix 2400.

**Sources**

1. Manufacturer / kit specs for Phoenix 2400 (span, length, AUW).
2. PX4 Gazebo `advanced_plane` (`Tools/simulation/gz/models/advanced_plane/model.sdf`) and AdvancedLiftDrag1 coefficients already ported into `advanced_liftdrag.hpp`.
3. Rigid-body scaling laws for inertia and aerodynamic moments.
4. Bench observation: with Phoenix geometry alone, Mission surface commands stayed ~1–4% while tracking ~40° bank; `SIH_CTRL_EFF` restores visible / plausible surface authority.

---

## 1. What is measured vs what is assumed

| Quantity | Status | Basis |
|----------|--------|--------|
| Wingspan \(b = 2.40\,\mathrm{m}\) | **Spec** | Phoenix 2400: 2400 mm |
| Overall length \(L = 1.132\,\mathrm{m}\) | **Spec** | Phoenix 2400: 1132 mm |
| Flying mass \(m = 1.16\,\mathrm{kg}\) | **Spec** | ~1160 g AUW |
| Aspect ratio \(\mathrm{AR} = 6.5\) | **Assumed** | Kept from validated `advanced_plane` (no published Phoenix AR) |
| Wing area \(S\) | **Derived** | \(S = b^2 / \mathrm{AR}\) |
| MAC \(\bar{c}\) | **Derived** | \(\bar{c} = S / b\) (rectangular-equivalent) |
| Aero force/moment coefficients (\(C_{L0}\), \(C_{L\alpha}\), …) | **Inherited** | Gazebo `advanced_plane` / AdvancedLiftDrag1 |
| Control derivatives (\(C_{\ell\delta}\), …) | **Inherited × scale** | Same baselines × `SIH_CTRL_EFF` |
| Inertia \(I_{xx}, I_{yy}, I_{zz}\) | **Scaled** | From `advanced_plane` inertias × mass × length² |
| CP location | **Scaled** | `advanced_plane` CP × MAC ratio |

Honest limitation: Phoenix 2400 is a foam powered-glider; `advanced_plane` is a small research FW model. Sharing nondimensional coefficients is an **engineering starting point** for SIH controller/mission bench work, not a claim of aerodynamic identity.

---

## 2. Geometry

### 2.1 Span, length, mass (from airframe)

\[
b = 2.40\,\mathrm{m},\quad L = 1.132\,\mathrm{m},\quad m = 1.16\,\mathrm{kg}
\]

Set in airframes as `SIH_MASS 1.16`. Span/MAC/area live in code (`sih.hpp`, `advanced_liftdrag.hpp`) because Allen wingtip sampling and AdvancedLiftDrag need them every step.

### 2.2 Aspect ratio (assumed)

Keep

\[
\mathrm{AR} = 6.5
\]

**Justification**

- Already used and flight-checked in PX4 SIH ↔ Gazebo `advanced_plane` work.
- Typical light powered-glider AR is often higher (≈8–12); keeping 6.5 is **conservative** (larger chord / area for a given span → stronger aero moments, slightly “stubbier” wing).
- Without a measured Phoenix wing area, changing AR would be another free parameter. Fix AR, derive \(S\) and \(\bar{c}\).

### 2.3 Wing area and MAC (derived)

\[
S = \frac{b^2}{\mathrm{AR}} = \frac{2.40^2}{6.5} = 0.88615\ldots \approx \mathbf{0.886\,\mathrm{m}^2}
\]

\[
\bar{c} = \frac{S}{b} = \frac{0.886}{2.40} \approx \mathbf{0.369\,\mathrm{m}}
\]

**Justification**

- For a rectangular wing, \(S = b\,\bar{c}\) and \(\mathrm{AR} = b/\bar{c} = b^2/S\). Using the same relations gives a **single consistent** \((b, S, \bar{c}, \mathrm{AR})\) tuple.
- Real Phoenix planform is tapered; treating \(\bar{c}\) as mean aerodynamic chord of an equivalent rectangle is standard for first-order 6-DoF models.

### 2.4 Geometric scale vs `advanced_plane`

| | `advanced_plane` | Phoenix SIH | Ratio |
|--|-----------------:|------------:|------:|
| Span \(b\) | 1.487 m | 2.40 m | 1.614 |
| Area \(S\) | 0.34 m² | 0.886 m² | 2.606 |
| MAC \(\bar{c}\) | 0.22 m | 0.369 m | 1.677 |
| AR | 6.5 | 6.5 | 1 |
| Mass | 1.0 kg | 1.16 kg | 1.16 |

Moment arms in AdvancedLiftDrag use \(S\) and \(b\) (or \(\bar{c}\)) explicitly, e.g. roll moment ~ \(q\,S\,b\,C_\ell\). So **growing the wing without reducing control derivatives over-powers the surfaces** (see §4).

### 2.5 Center of pressure / aerodynamic reference offset

`advanced_plane` used \(\mathbf{c}_p \approx (-0.12, 0, 0)\,\mathrm{m}\).

Scaled with MAC:

\[
c_{p,x} = -0.12 \times \frac{0.369}{0.22} \approx -0.201 \approx \mathbf{-0.20\,\mathrm{m}}
\]

**Justification:** keep the same nondimensional CP location along the chord when the reference chord grows. Fuselage length \(L\) was **not** used to invent a new aero model; it only documents the physical airframe and supports “length scale” narrative for inertia.

---

## 3. Mass properties (`SIH_MASS`, `SIH_Ixx/Iyy/Izz`)

### 3.1 Mass

\[
m = 1.16\,\mathrm{kg}\quad(\texttt{SIH\_MASS})
\]

Directly from flying-weight spec.

### 3.2 Inertia scaling

Baseline (Gazebo `advanced_plane`, 1 kg):

\[
I_{xx}^{(0)}=0.197563,\quad
I_{yy}^{(0)}=0.1458929,\quad
I_{zz}^{(0)}=0.1477\quad[\mathrm{kg\cdot m^2}]
\]

Rigid-body inertia scales approximately as

\[
I \sim m\,\ell^2
\]

Use **span** as the dominant length for a high-AR wing (roll/yaw especially):

\[
k_I = \frac{m}{m_0}\left(\frac{b}{b_0}\right)^2
= 1.16 \times \left(\frac{2.40}{1.487}\right)^2
\approx 1.16 \times 2.605 \approx \mathbf{3.022}
\]

\[
\begin{aligned}
I_{xx} &= 0.197563 \times 3.022 \approx \mathbf{0.597}\\
I_{yy} &= 0.1458929 \times 3.022 \approx \mathbf{0.441}\\
I_{zz} &= 0.1477 \times 3.022 \approx \mathbf{0.446}
\end{aligned}
\]

**Justification**

- No Phoenix CAD inertia was available.
- Scaling a known PX4 reference inertia by \(m(b/b_0)^2\) is a standard first-order approach.
- Using span (not fuselage length) for all three axes is a simplification: pitch inertia is more fuselage/chord driven; errors of tens of percent are acceptable for SIH controller benches, not for loads certification.

Products of inertia: `SIH_IXZ = 0` (same assumption as the scaled SITL airframe).

---

## 4. Aerodynamic coefficients

### 4.1 Baseline force / stability derivatives

All of \(C_{L0}\), \(C_{L\alpha}\), \(C_{D0}\), \(e\), stall blend, \(C_{Y\beta}\), \(C_{\ell\beta}\), rate dampers \(C_{\ell p}\), \(C_{mq}\), etc. are **copied from Gazebo `advanced_plane` AdvancedLiftDrag1**.

**Justification**

- Already implemented and sign-checked in this repo’s SIH port.
- Changing them without flight / CFD data would add unjustified free parameters.
- Nondimensional coeffs are applied with **Phoenix \(S\), \(b\), \(\bar{c}\)** so dimensional forces/moments grow with the larger wing.

### 4.2 Control derivatives and `SIH_CTRL_EFF`

Dimensional control moments scale roughly as

\[
M_{\mathrm{ctrl}} \propto q\,S\,b\,(C_{\ell\delta}\,\delta)\cdot k_{\mathrm{eff}}
\]

with \(k_{\mathrm{eff}} = \texttt{SIH\_CTRL\_EFF}\).

Geometric growth of the moment “lever” vs `advanced_plane`:

\[
\frac{(S b)_{\mathrm{Phx}}}{(S b)_{\mathrm{adv}}}
= \frac{0.886 \times 2.40}{0.34 \times 1.487}
\approx \frac{2.126}{0.506}
\approx \mathbf{4.20}
\]

So if \(k_{\mathrm{eff}}=1\), Phoenix SIH has ~**4.2×** the control moment of `advanced_plane` for the same normalized surface command \(\delta\in[-1,1]\), while inertia only grew ~**3.0×**. Net roll acceleration from controls would be **higher** than the already agile small model → **tiny** PX4 surface commands for a 40° bank (exactly what was observed on the bench).

**Choosing \(k_{\mathrm{eff}}\)**

| Goal | Formula / choice | Result |
|------|------------------|--------|
| Match `advanced_plane` control acceleration roughly | \(k_{\mathrm{eff}} \approx 3.02 / 4.20 \approx 0.72\) | Similar agility to small model |
| Weaker / more “trainer-like” + visible throws | **0.12 (default)** | ~\(0.12/0.72 \approx 1/6\) of that matched agility |
| First attempt (compile-time ×0.4 only) | 0.4 | Net ~\(0.4\times4.2/3.0 \approx 0.56\times\) still too strong → still tiny surfaces |

Default:

\[
\texttt{SIH\_CTRL\_EFF} = \mathbf{0.12}
\]

**Justification narrative for a report**

1. Start from published AdvancedLiftDrag control derivatives (traceable).
2. Apply geometric Phoenix \(S,b\) (traceable from span + assumed AR).
3. Introduce a single scalar \(k_{\mathrm{eff}}\) because **surface effectiveness was not identified** on the Phoenix and because geometric scaling otherwise overstates control power.
4. Set 0.12 so that:
   - Mission roll-in produces **visibly larger** `actuator_servos` (bench requirement for HySoar hardware-in-the-loop demos);
   - Attitude tracking remains usable (verified qualitatively after the change);
   - The parameter remains **tunable** without rebuilding (`param set SIH_CTRL_EFF …`).

Tune band for the report: **0.08–0.20**. Below ~0.08 risk sluggish / oscillatory under FW rate gains; above ~0.25 surfaces become small again in Mission.

`SIH_CTRL_EFF` multiplies **all** control-surface contributions (\(C_L, C_D, C_Y, C_\ell, C_m, C_n\) from δ) so the mix stays consistent.

---

## 5. Other SIH parameters used with this plant

| Param | Value | Justification |
|-------|------:|---------------|
| `SIH_VEHICLE_TYPE` | 1 | Fixed-wing |
| `SIH_T_MAX` | 20 N | Order-of-magnitude thrust for ~1.2 kg SIH takeoff (same ballpark as advanced_plane SITL); not an ESC thrust stand measurement |
| `SIH_KDV` | 0 | Parasite linear drag left to AdvancedLiftDrag \(C_D\); extra linear drag disabled |
| `SIH_ACT_OUT` | 1 | Bench only: allow real MAIN PWM under SIH (see `sih_real_pwm_lockdown.md`) |
| `SYS_HITL` | 2 | SIH on FMU |
| `ADV_PLANE_DEFL_MAX` | 0.78 rad (~45°) | Gazebo advanced_plane joint limit; maps \(\delta_{\mathrm{cmd}}\in[-1,1]\) to deflection |

Thrust is **not** claimed to match a specific Phoenix prop/ESC curve; it is sized so runway takeoff in SIH completes under TECS / RWTO.

---

## 6. What these values are *not*

- Not a Phoenix 2400 system-ID result.
- Not a guarantee that outdoor Phoenix gains equal SIH gains.
- Not a reason to expect large aileron **while holding** bank: with good tracking, \(\delta_a\to 0\) even on a real aircraft. Justification of “small hold deflection” is closed-loop control, independent of `SIH_CTRL_EFF`.

---

## 7. One-paragraph summary (for papers / reports)

The SIH fixed-wing plant uses Phoenix 2400 **span (2.40 m), length (1.132 m), and mass (1.16 kg)**. Wing **area (0.886 m²) and MAC (0.369 m)** follow from retaining the validated Gazebo `advanced_plane` **aspect ratio 6.5**. Nondimensional AdvancedLiftDrag coefficients are taken from that model; dimensional forces/moments use the Phoenix geometry. Inertia is scaled from `advanced_plane` by \(m(b/b_0)^2\). Because \(Sb\) grows by ~4.2× while inertia grows by ~3×, a single effectiveness parameter **`SIH_CTRL_EFF = 0.12`** scales control-surface derivatives so SIH does not overstate roll/pitch/yaw authority; the value was chosen so Mission surface commands become observable on the hardware bench while attitude tracking remains acceptable, and it can be retuned without changing the coefficient tables.

---

## 8. File map

| Item | Location |
|------|----------|
| Geometry + coeffs | `src/modules/simulation/simulator_sih/advanced_liftdrag.hpp` |
| Span constants (Allen tips) | `src/modules/simulation/simulator_sih/sih.hpp` |
| `SIH_CTRL_EFF` | `src/modules/simulation/simulator_sih/sih_params.c` |
| Mass / inertia / defaults | `ROMFS/.../1101_rc_plane_sih.hil`, `10041_sihsim_airplane` |
| Real PWM lockdown | `docs/hysoar/sih_real_pwm_lockdown.md` |
