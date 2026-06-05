"""ipex_specs.py — IPEx flight-system parameters, real-data-sourced (no fabricated values).

Grounds the M3 energy/battery model (K2) in published IPEx numbers instead of arbitrary
coefficients. Every constant carries its provenance. Primary source:

  [SCHULER24] J.M. Schuler et al., "ISRU Pilot Excavator (IPEx) Technology Readiness Level 5
              Design Overview", AIAA AVIATION FORUM AND ASCEND 2024, 2024.
              NTRS 20240008162.  IPEx mass/speed/size/power were estimated by recreating the
              ConOps with the RASSOR 2 proof-of-concept, recording battery + actuator current
              draw, and applying a ~0.7 one-dimensional scaling factor (RASSOR 2 -> IPEx).
  [BATTERY]   Current flight build: 12S Li-ion pack, ~30 Ah (per project lead, 2026-06-02).
              Test-rig bus was swept at 47.6 / 53.2 / 58.8 V (a 14S range, 58.8 = 14*4.2);
              the 12S/30Ah figure here is the current pack, not the dynamometer rig.

[CALIB] tags mark values derived under a stated assumption (e.g. an operational speed the
paper reports only for an accelerated test). They are honest estimates from real inputs, not
fabricated; refine when the per-actuator current->torque equations (Figs 6, 20) are available.
"""
from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi


# ---- Published IPEx ConOps / sizing numbers [SCHULER24] -----------------------------------
ROVER_MASS_CLASS_KG = 30.0        # "30 kg-class excavator"
DRIVE_SPEED_MS = 0.30             # nominal driving speed 30 cm/s
DRUM_SPEED_RPM = 25.0             # bucket-drum operational rotation rate
REGOLITH_PER_CYCLE_KG = 30.0      # collect/store/deposit up to 30 kg/cycle (15 kg min threshold)
DIG_RATE_KG_PER_HR = 42.0         # demonstration excavation rate
TOTAL_REGOLITH_KG = (5000.0, 10000.0)   # moved over the mission
TRAVERSE_KM = 70.0                # total driving distance
MISSION_DAYS = 11.0
SCALE_FACTOR = 0.7                # 1-D RASSOR 2 -> IPEx scaling factor
BUS_VOLTAGE_TESTED_V = (47.6, 53.2, 58.8)

# Table 3 (Loads experienced during ConOps), wheel actuator, 80:1 gearbox.
WHEEL_GEAR_RATIO = 80.0
DRIVE_MOTOR_TORQUE_NM = 0.063     # mean motor-side load of the four driving cases (3a/3b/4a/4b)
DRIVE_MOTOR_SPEED_RPM = 1530.0    # motor speed for those driving cases
N_WHEELS = 4

# Table 7 note: "18.5 Nm is predicted excavation load on the moon" (arm actuator).
ARM_EXCAVATION_LOAD_NM = 18.5

# ---- Battery [BATTERY] --------------------------------------------------------------------
BATTERY_SERIES_CELLS = 12         # 12S -> ~44 V pack
BATTERY_CAPACITY_AH = 30.0        # ~30 Ah
LIION_NOMINAL_V_PER_CELL = 3.7    # standard Li-ion nominal: 12 * 3.7 = 44.4 V ~= 44 V
# NOTE: IPEx actuators were qualified at -35 C / +40 C (TC2). That lunar-grade thermal range is
# NOT met by off-the-shelf cells; pack energy/usable capacity degrades sharply at those extremes.
# This model uses nominal-temperature capacity; a thermal-derating factor is [CALIB] future work.


def battery_energy_j() -> float:
    """Usable pack energy at nominal voltage: 12S * 3.7 V * 30 Ah -> J (~4.79 MJ / 1332 Wh)."""
    v_nom = BATTERY_SERIES_CELLS * LIION_NOMINAL_V_PER_CELL
    return v_nom * BATTERY_CAPACITY_AH * 3600.0


def battery_energy_wh() -> float:
    return battery_energy_j() / 3600.0


def drive_power_w() -> float:
    """Whole-rover drive mechanical power from Table 3 driving cases: P = N * tau_motor * omega.

    Motor-side mechanical power (electrical draw is higher by 1/efficiency; the drivetrain
    efficiency is not given in [SCHULER24], so this is a lower bound on battery drain). [CALIB]
    """
    omega = DRIVE_MOTOR_SPEED_RPM * TWO_PI / 60.0
    return N_WHEELS * DRIVE_MOTOR_TORQUE_NM * omega


def drive_energy_per_m() -> float:
    """Joules per metre driven at the nominal 0.30 m/s: P_drive / v."""
    return drive_power_w() / DRIVE_SPEED_MS


def dig_power_w() -> float:
    """Excavation mechanical power: predicted arm load (18.5 N*m) at the drum's operational
    25 RPM. [CALIB] the paper reports 18.5 N*m at a 500 RPM accelerated-life speed; pairing it
    with the 25 RPM operational drum rate is the honest operational estimate."""
    omega = DRUM_SPEED_RPM * TWO_PI / 60.0
    return ARM_EXCAVATION_LOAD_NM * omega


def dig_energy_per_kg() -> float:
    """Joules per kg excavated: P_dig / (dig rate in kg/s)."""
    rate_kg_s = DIG_RATE_KG_PER_HR / 3600.0
    return dig_power_w() / rate_kg_s


# ---- Planner operational parameters (mission_planner build-sequencer) ----------------------
# NOT in [SCHULER24]: planner-level assumptions for the energy/battery build sequencer. The single
# source of truth for these knobs (mission_planner imports them; nothing is duplicated downstream).
SINTER_HEAD_POWER_W = 1000.0      # [SOURCED] domestic-microwave-class head ~0.8-1.0 kW (Lin et al. 2024);
#                                   NOT on the IPEx baseline (drum excavator, no sinter tool) -- sinter-EQUIPPED
#                                   variant only. constants.SINTER_ENABLED gates it (sourced-physics rationale).
RECHARGE_POWER_W = 700.0          # [CALIB] surface recharge power (no IPEx solar/charge spec)
BATTERY_RESERVE_FRAC = 0.10       # operational: hold >=10% pack reserve before forcing a recharge
# [ASSUMPTION] continuous idle / heater / avionics survival draw, NOT in [SCHULER24] and genuinely
# data-gated. Over a multi-day sortie this term is plausibly the DOMINANT energy cost, so the planner
# surfaces it as its OWN line, tagged [ASSUMPTION]. Default 0 W = "not modelled" (no silent inflation of
# the headline figures); set DUSTGYM_IDLE_POWER_W (or the constant) to fold in a survival load you can
# defend. Lunar-night heater loads for a small rover are tens-to-hundreds of W -- do NOT treat any value
# here as sourced. (mission_planner: survival_energy_J = IDLE_POWER_W * mission duration.)
IDLE_POWER_W = 0.0


def energy_model(cell_m: float, *, allowance_j: float | None = None,
                 allowance_factor: float | None = None,
                 planner_cost_j: float | None = None) -> dict:
    """Grounded coefficients for SkillMacroEnv's resource layer, in SI joules.

    travel_cost_per_cell = drive_energy_per_m * cell_m   (env distance is in cell units)
    dig_cost_per_kg      = dig_energy_per_kg()
    energy_budget        = allowance_j, or allowance_factor * planner_cost_j (a task slice),
                           else the full pack (battery_energy_j()).

    The per-unit costs are real-data-grounded for ANY map scale. The full pack only *binds*
    at mission scale (5 m LOLA cells, 70 km, 1e4 kg); at the 2 cm sandbox scale a single
    construction episode draws ~kJ of a ~4.7 MJ pack, so pass an allowance to make routing
    efficiency matter (allowance_factor ~1.3 of the grounded optimal-plan cost is a tight,
    binding budget).
    """
    if allowance_j is not None:
        budget = float(allowance_j)
    elif allowance_factor is not None and planner_cost_j is not None:
        budget = float(allowance_factor) * float(planner_cost_j)
    else:
        budget = battery_energy_j()
    return {
        "travel_cost_per_cell": drive_energy_per_m() * float(cell_m),
        "dig_cost_per_kg": dig_energy_per_kg(),
        "energy_budget": budget,
    }


def spec_record() -> dict:
    """JSON-dumpable provenance record of every real input + derived quantity."""
    return {
        "source": "Schuler et al., IPEx TRL-5 Design Overview, ASCEND 2024 (NTRS 20240008162)",
        "battery_source": "project lead 2026-06-02: 12S Li-ion, ~30 Ah",
        "published": {
            "mass_class_kg": ROVER_MASS_CLASS_KG, "drive_speed_ms": DRIVE_SPEED_MS,
            "drum_speed_rpm": DRUM_SPEED_RPM, "regolith_per_cycle_kg": REGOLITH_PER_CYCLE_KG,
            "dig_rate_kg_per_hr": DIG_RATE_KG_PER_HR, "total_regolith_kg": list(TOTAL_REGOLITH_KG),
            "traverse_km": TRAVERSE_KM, "mission_days": MISSION_DAYS,
            "scale_factor_rassor2_to_ipex": SCALE_FACTOR,
            "drive_motor_torque_nm": DRIVE_MOTOR_TORQUE_NM,
            "drive_motor_speed_rpm": DRIVE_MOTOR_SPEED_RPM, "n_wheels": N_WHEELS,
            "wheel_gear_ratio": WHEEL_GEAR_RATIO, "arm_excavation_load_nm": ARM_EXCAVATION_LOAD_NM,
            "bus_voltage_tested_v": list(BUS_VOLTAGE_TESTED_V),
            "battery_series_cells": BATTERY_SERIES_CELLS, "battery_capacity_ah": BATTERY_CAPACITY_AH,
        },
        "derived": {
            "battery_energy_wh": round(battery_energy_wh(), 1),
            "battery_energy_mj": round(battery_energy_j() / 1e6, 3),
            "drive_power_w": round(drive_power_w(), 2),
            "drive_energy_per_m_j": round(drive_energy_per_m(), 2),
            "dig_power_w": round(dig_power_w(), 2),
            "dig_energy_per_kg_j": round(dig_energy_per_kg(), 1),
        },
        "planner": {                                  # [CALIB]/operational, not [SCHULER24]
            "sinter_head_power_w": SINTER_HEAD_POWER_W,
            "recharge_power_w": RECHARGE_POWER_W,
            "battery_reserve_frac": BATTERY_RESERVE_FRAC,
        },
    }


# Externalized config overlay (PRD N15 / area O): apply DUSTGYM_<NAME> env vars + the
# DUSTGYM_CONFIG TOML to these published/planner constants. The energy/battery quantities are
# functions that read the constants live, so a 14S override (etc.) recomputes automatically.
from . import config as _config  # noqa: E402

_config.apply(globals())


if __name__ == "__main__":
    import json
    print(json.dumps(spec_record(), indent=2))
