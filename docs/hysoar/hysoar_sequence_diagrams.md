# HySoar — system sequence diagrams (Mermaid)

These diagrams summarize the XRCE-DDS autosoaring path: `FixedwingPositionControl::Run()` (ingress + staleness), `FixedwingPositionControl::control_auto()` (safety gates, Commander/Navigator, TECS, mission restore, throttle ramp), and the companion link.

Render with [Mermaid Live Editor](https://mermaid.live), VS Code **Mermaid** preview, GitHub/GitLab Markdown, or:

```bash
npx @mermaid-js/mermaid-cli -i docs/hysoar/hysoar_system_sequence.mmd -o hysoar_system_sequence.svg
npx @mermaid-js/mermaid-cli -i docs/hysoar/hysoar_ceiling_rearm_sequence.mmd -o hysoar_ceiling_rearm_sequence.svg
```

---

## 1. End-to-end HySoar flow

```mermaid
sequenceDiagram
    autonumber
    participant Companion as ROS2 companion
    participant DDS as XRCE DDS uORB bridge
    participant FP as FixedwingPositionControl
    participant Commander
    participant Navigator
    participant TECS
    participant AC as Attitude controller

    Note over Companion,AC: Runs on LOCAL_POSITION ticks Run then control_auto

    Companion->>DDS: autosoaring_control
    DDS->>FP: uORB update

    activate FP

    FP->>FP: Staleness if no DDS for 2s to SOARING_OFF STALENESS status TECS glide off clear last_recv

    opt New DDS in thermal mode
        FP->>FP: Reset DO_REPOSITION debounce clear thermal lat lon cache NaN NaN record last thermal time
    end

    FP->>FP: control_auto start

    opt Pending restore and NAV_STATE AUTO_MISSION
        FP->>Commander: VEHICLE_CMD_MISSION_START seq
        FP->>FP: Clear pending invalidate latched seq
    end

    FP->>FP: FW_ALT_MIN floor latch ceiling hysteresis post-ceiling thermal re-arm gate

    FP->>FP: Compute effective_glide effective_thermal effective_thermal_bank

    alt Effective thermal re-arm ok ceiling cleared
        FP->>Navigator: Snapshot mission seq on first activation
        FP->>Commander: VEHICLE_CMD_DO_REPOSITION AUTO_LOITER
        Commander->>Navigator: Loiter centre radius altitude
        Navigator-->>FP: Setpoints mission uORB
    else Effective glide without thermal
        FP->>Commander: VEHICLE_CMD_DO_SET_MODE AUTO_MISSION debounced
        Commander->>Navigator: Mission mode
        Navigator-->>FP: Mission setpoints
    end

    FP->>TECS: set_gliding_mode_enabled glide_or_thermal
    FP->>TECS: Glide EAS polar sqrt c over a fixed cmd FW_GLIDE_AIRSPD
    FP->>FP: Thermal bank companion or FW_THERMAL_BANK

    alt Thermal off edge neither glide nor thermal
        FP->>Commander: DO_SET_MODE AUTO_MISSION clear cache clear forbid paths
        opt Valid resume seq
            FP->>FP: Arm deferred MISSION_START
        end
    else Glide-only off edge
        FP->>Commander: DO_SET_MODE AUTO_MISSION clear forbid no mission restore arm
    end

    FP->>DDS: autosoaring_status SOURCE_PERIODIC companion via DDS
    Note over DDS,Companion: Status returns on autosoaring_status topic

    alt effective_thermal_bank
        FP->>TECS: tecs_update_pitch_throttle altitude glide speed
        FP->>AC: Quaternion bank_sign TECS_pitch yaw
    else Normal AUTO idle position velocity loiter
        FP->>FP: NPFG lateral path
        FP->>TECS: Standard branch with glide flag
        FP->>AC: Attitude NPFG plus TECS
    end

    FP->>FP: FW_GLIDE_RAMP_T throttle ramp

    deactivate FP

    Note over Companion,FP: After ceiling hysteresis a newer thermal DDS than rearm timestamp required
```

---

## 2. Ceiling, hysteresis, and thermal re-arm

```mermaid
sequenceDiagram
    participant Companion
    participant FP as FixedwingPositionControl

    Companion->>FP: AutosoaringControl thermal remains requested

    FP->>FP: At FW_ALT_MAX set ceiling latch
    FP->>FP: effective_glide thermal suppressed by latch

    loop Altitude in hysteresis band above FW_ALT_MAX minus FW_ALT_HYST
        FP->>FP: Mission glide behavior
        Companion->>FP: Repeated thermal only does not re-open thermal alone
    end

    FP->>FP: Below band clears latch store rearm-after timestamp

    Companion->>FP: New thermal DDS sample after timestamp
    FP->>FP: Thermal allowed again thermal_rearmed
```

---

## Standalone `.mmd` files (CLI / imports)

| File | Content |
|------|---------|
| `hysoar_system_sequence.mmd` | Diagram 1 (full system) |
| `hysoar_ceiling_rearm_sequence.mmd` | Diagram 2 (ceiling + re-arm) |
