# AutosoaringControl — uORB message published by a ROS2 companion-computer node
# and bridged into the FMU via XRCE-DDS (/fmu/in/autosoaring_control).
# The FMU fw_pos_control module reads this message to select the soaring mode.
#
# ── Mode encoding ──────────────────────────────────────────────────────────────
# soaring_mode  |  TECS         | Navigator       | Fields used
# ─────────────────────────────────────────────────────────────────────────────
# 0  OFF         | powered       | unchanged       | (none)
# 1  GLIDE_FIXED | gliding       | AUTO_MISSION    | airspeed_cmd
# 2  GLIDE_POLAR | gliding       | AUTO_MISSION    | (polar from params)
# 3  THERMAL_LOITER | gliding    | AUTO_LOITER     | loiter_*, bank_angle_cmd, loiter_clockwise
# 4  THERMAL_BANK   | gliding    | AUTO_LOITER     | bank_angle_cmd, loiter_clockwise (roll sign)
# ─────────────────────────────────────────────────────────────────────────────
# Fields not listed for a mode are silently ignored.

uint64  timestamp               # time since system start (microseconds)

# ── Mode command ───────────────────────────────────────────────────────────────
uint8   soaring_mode            # Active soaring mode (see constants below)

uint8 SOARING_OFF             = 0   # Normal powered flight
uint8 SOARING_GLIDE_FIXED     = 1   # Engine-off glide, fixed EAS = FW_GLIDE_AIRSPD (or airspeed_cmd)
uint8 SOARING_GLIDE_POLAR     = 2   # Engine-off glide, polar best-glide EAS = sqrt(FW_POLAR_B/FW_POLAR_A)
uint8 SOARING_THERMAL_LOITER  = 3   # Thermalling, radius-guided loiter via navigator
uint8 SOARING_THERMAL_BANK    = 4   # Thermalling, direct bank-angle command (companion controls centering)

# ── Loiter geometry — used only by SOARING_THERMAL_LOITER (3) ─────────────────
float32 loiter_radius_m         # Loiter radius [m]; NaN = derive minimum from bank_angle_cmd
float32 loiter_lat              # Thermal centre latitude  [deg]; NaN or 0 = use current position
float32 loiter_lon              # Thermal centre longitude [deg]; NaN or 0 = use current position

# ── Per-cycle optimisation — NaN means "use PX4 default parameter" ─────────────
float32 airspeed_cmd            # MacCready airspeed [m/s EAS]; used by SOARING_GLIDE_FIXED only
float32 bank_angle_cmd          # Bank angle magnitude [deg]; used by SOARING_THERMAL_LOITER and SOARING_THERMAL_BANK
bool    loiter_clockwise        # Turn direction: true = clockwise (right), false = CCW (left). Default: true

# ── Metadata ───────────────────────────────────────────────────────────────────
uint8   soaring_phase           # Companion-reported phase (for logging / GCS display)

uint8 SOARING_PHASE_CRUISE    = 0   # Normal powered cruise
uint8 SOARING_PHASE_GLIDING   = 1   # Engine-off glide along mission
uint8 SOARING_PHASE_SEARCHING = 2   # Searching for thermal (circling without confirmed lift)
uint8 SOARING_PHASE_CENTERING = 3   # Adjusting circle to centre on thermal core
uint8 SOARING_PHASE_THERMAL   = 4   # Confirmed thermal — climbing
