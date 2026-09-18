# AutosoaringStatus — uORB message published by fw_pos_control.
# Bridged to the companion via XRCE-DDS as /fmu/out/autosoaring_status.
# Provides FMU-side feedback: active mode, staleness events, and derived soaring phase.

uint64 timestamp               # time since system start (microseconds)

uint8 source                   # What caused this status emission (see constants below)
uint8 SOURCE_DDS_OFF              = 0   # Companion sent SOARING_OFF
uint8 SOURCE_STALENESS_WATCHDOG   = 1   # Companion silent >2 s — soaring forced OFF
uint8 SOURCE_PERIODIC             = 2   # Periodic heartbeat during active soaring (2 Hz)

uint8 soaring_mode_effective   # Mode stored in _autosoaring_control after the event (same encoding as AutosoaringControl.soaring_mode)

uint64 dds_inhibit_until_us    # Reserved — always 0 in DDS-only mode

# FMU-derived phase — what the FMU is actually doing this cycle (same encoding as AutosoaringControl.soaring_phase)
uint8 fmu_soaring_phase
uint8 SOARING_PHASE_CRUISE    = 0   # Powered cruise (soaring OFF or forbidden)
uint8 SOARING_PHASE_GLIDING   = 1   # Engine-off glide along mission (effective_glide)
uint8 SOARING_PHASE_SEARCHING = 2   # Thermal active but loiter not yet positioned (_soaring_last_lat==NAN)
uint8 SOARING_PHASE_THERMAL   = 3   # Thermal loiter active and loiter centre established
