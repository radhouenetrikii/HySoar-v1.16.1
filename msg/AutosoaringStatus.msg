# AutosoaringStatus — uORB message published by fw_pos_control so a ROS2 companion
# can observe FMU-side autosoaring decisions (CLI soar:off, DDS inhibit window, watchdog, active phase).
# Bridged to the companion via XRCE-DDS as /fmu/out/autosoaring_status when listed in dds_topics.yaml.

uint64 timestamp               # time since system start (microseconds)

uint8 source                   # What caused this status emission (see constants below)
uint8 SOURCE_CLI_SOARING_OFF      = 0   # Operator `soar:off` via VEHICLE_CMD_CUSTOM_0
uint8 SOURCE_STALENESS_WATCHDOG   = 1   # Companion silent >2 s — soaring forced OFF
uint8 SOURCE_PERIODIC             = 2   # Periodic heartbeat during active soaring (2 Hz)
uint8 SOURCE_CLI_SOARING_ON       = 3   # Operator `soar:glide/polar/thermal/bank` via VEHICLE_CMD_CUSTOM_0

uint8 soaring_mode_effective   # Mode stored in _autosoaring_control after the event (same encoding as AutosoaringControl.soaring_mode)

uint64 dds_inhibit_until_us    # Absolute time [us] until DDS "soaring on" is accepted again; 0 = no inhibit

# FMU-derived phase — what the FMU is actually doing this cycle (same encoding as AutosoaringControl.soaring_phase)
uint8 fmu_soaring_phase
uint8 SOARING_PHASE_CRUISE    = 0   # Powered cruise (soaring OFF or forbidden)
uint8 SOARING_PHASE_GLIDING   = 1   # Engine-off glide along mission (effective_glide)
uint8 SOARING_PHASE_SEARCHING = 2   # Thermal active but loiter not yet positioned (_soaring_last_lat==NAN)
uint8 SOARING_PHASE_THERMAL   = 3   # Thermal loiter active and loiter centre established
