# SihThermalField — Allen (1997) thermal list for Simulator in Hardware.
# Companion publishes the full active set on /fmu/in/sih_thermal_field (XRCE-DDS).
# SIH evaluates the updraft at the vehicle and subtracts it from air-relative velocity
# (same coupling as the Gazebo AdvancedLiftDrag plugin).

uint8 ORB_QUEUE_LENGTH = 2
uint8 MAX_THERMALS = 16

uint64 timestamp               # time since system start (microseconds)
float32 sim_time_s             # companion / sim time [s]; 0 = ignore lifecycle
uint8 count                    # valid entries in the arrays (0..MAX_THERMALS)
uint8 lifecycle_enabled        # 0 = constant wi; 1 = grow/stable/fade vs birth_time

float32[16] lat                # thermal centre latitude [deg]
float32[16] lon                # thermal centre longitude [deg]
float32[16] zi                 # convective mixing-layer height [m AGL]
float32[16] wi                 # convective velocity scale w* [m/s]
float32[16] lifetime           # thermal lifetime [s]
float32[16] birth_time         # sim time of birth [s]
