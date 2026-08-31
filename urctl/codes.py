"""Numeric-code decoding — turn controller enums and bit words into names.

RTDE (and the Primary broadcast) report state as integers: ``robot_mode=7``,
``safety_status_bits=2049``. Every consumer of the toolkit (CLI, tools, MCP,
a GUI) wants the *names*, and the mapping is fixed by the e-Series controller,
so it lives here once. Sources: the UR RTDE guide's field documentation and
the e-Series URScript manual (``get_robot_status_bits`` and friends), spot-
verified against this repo's URSim 5.12.5 and a real UR10e on URControl 82.x
(``safety_status_bits=2049`` == NORMAL + 3-position-enabling bit, matching the
Dashboard's ``Safetymode: NORMAL``).
"""

from __future__ import annotations

# RTDE ``robot_mode`` (INT32). -1 only appears transiently at boot.
ROBOT_MODES: dict[int, str] = {
    -1: "NO_CONTROLLER",
    0: "DISCONNECTED",
    1: "CONFIRM_SAFETY",
    2: "BOOTING",
    3: "POWER_OFF",
    4: "POWER_ON",
    5: "IDLE",
    6: "BACKDRIVE",
    7: "RUNNING",
    8: "UPDATING_FIRMWARE",
}

# RTDE ``safety_mode`` (INT32) — the *coarse* safety state, same values the
# Dashboard's ``safetymode`` command names.
SAFETY_MODES: dict[int, str] = {
    1: "NORMAL",
    2: "REDUCED",
    3: "PROTECTIVE_STOP",
    4: "RECOVERY",
    5: "SAFEGUARD_STOP",
    6: "SYSTEM_EMERGENCY_STOP",
    7: "ROBOT_EMERGENCY_STOP",
    8: "VIOLATION",
    9: "FAULT",
    10: "VALIDATE_JOINT_ID",
    11: "UNDEFINED_SAFETY_MODE",
    12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
    13: "SYSTEM_THREE_POSITION_ENABLING_STOP",
}

# RTDE ``runtime_state`` (UINT32) — the program interpreter's state.
RUNTIME_STATES: dict[int, str] = {
    0: "STOPPING",
    1: "STOPPED",
    2: "PLAYING",
    3: "PAUSING",
    4: "PAUSED",
    5: "RESUMING",
    6: "RETRACTING",
}

# RTDE ``joint_mode`` (VECTOR6INT32) — per-joint drive state.
JOINT_MODES: dict[int, str] = {
    235: "SHUTTING_DOWN",
    236: "PART_D_CALIBRATION",
    237: "BACKDRIVE",
    238: "POWER_OFF",
    240: "READY_FOR_POWER_OFF",
    245: "NOT_RESPONDING",
    246: "MOTOR_INITIALISATION",
    247: "BOOTING",
    248: "PART_D_CALIBRATION_ERROR",
    249: "BOOTLOADER",
    250: "CALIBRATION",
    252: "FAULT",
    253: "RUNNING",
    255: "IDLE",
}

# RTDE ``tool_mode`` (UINT32) — tool-connector state; same value space as the
# joint modes (the tool board is a joint-class device on the same bus).
TOOL_MODES: dict[int, str] = JOINT_MODES

# ``safety_status_bits`` (UINT32) — one flag per bit. Bit 11 is set in normal
# operation on e-Series (the 3-position enabling device is "not active" state
# encoding); a plain healthy robot reads 2049 = NORMAL_MODE | bit 11.
SAFETY_STATUS_BITS: list[str] = [
    "normal_mode",  # bit 0
    "reduced_mode",
    "protective_stopped",
    "recovery_mode",
    "safeguard_stopped",
    "system_emergency_stopped",
    "robot_emergency_stopped",
    "emergency_stopped",
    "violation",
    "fault",
    "stopped_due_to_safety",  # bit 10
    "three_position_enabling",  # bit 11
]

# ``robot_status_bits`` (UINT32).
ROBOT_STATUS_BITS: list[str] = [
    "power_on",  # bit 0
    "program_running",
    "teach_button_pressed",
    "power_button_pressed",
]


def name_for(table: dict[int, str], code: object) -> str | None:
    """Look up ``code`` in ``table``; None-safe, unknown codes come back as
    ``"UNKNOWN(<code>)"`` so a new firmware enum value is visible, not lost."""
    if code is None:
        return None
    try:
        key = int(code)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return table.get(key, f"UNKNOWN({key})")


def shape_rtde_sample(raw: dict, *, deep: bool = False, unavailable: list[str] | None = None) -> dict:
    """Map a raw RTDE output sample (controller field names) to the toolkit's
    friendly, decoded shape. The single source of truth for that mapping —
    used by :meth:`urctl.Robot.rtde_state` and the GUI's telemetry stream."""
    result = {
        "joints": raw.get("actual_q"),
        "joint_velocities": raw.get("actual_qd"),
        "tcp": raw.get("actual_TCP_pose"),
        "tcp_speed": raw.get("actual_TCP_speed"),
        "tcp_force": raw.get("actual_TCP_force"),
        "safety_status": raw.get("safety_status_bits"),
        "safety_flags": decode_bits(SAFETY_STATUS_BITS, raw.get("safety_status_bits")),
        "runtime_state": raw.get("runtime_state"),
        "runtime_state_name": name_for(RUNTIME_STATES, raw.get("runtime_state")),
        "rtde_robot_mode": raw.get("robot_mode"),
        "robot_mode_name": name_for(ROBOT_MODES, raw.get("robot_mode")),
        "digital_inputs": raw.get("actual_digital_input_bits"),
        "digital_outputs": raw.get("actual_digital_output_bits"),
        "timestamp": raw.get("timestamp"),
    }
    if deep:
        jm = raw.get("joint_mode")
        result.update(
            {
                "safety_mode": raw.get("safety_mode"),
                "safety_mode_name": name_for(SAFETY_MODES, raw.get("safety_mode")),
                "robot_status_flags": decode_bits(ROBOT_STATUS_BITS, raw.get("robot_status_bits")),
                "joint_targets": raw.get("target_q"),
                "joint_currents": raw.get("actual_current"),
                "joint_temperatures": raw.get("joint_temperatures"),
                "joint_voltages": raw.get("actual_joint_voltages"),
                "joint_modes": jm,
                "joint_mode_names": ([name_for(JOINT_MODES, m) for m in jm] if jm else None),
                "elbow_position": raw.get("elbow_position"),
                "speed_scaling": raw.get("speed_scaling"),
                "target_speed_fraction": raw.get("target_speed_fraction"),
                "robot_voltage": raw.get("actual_robot_voltage"),
                "robot_current": raw.get("actual_robot_current"),
                "main_voltage": raw.get("actual_main_voltage"),
                "analog_in": [raw.get("standard_analog_input0"), raw.get("standard_analog_input1")],
                "analog_out": [raw.get("standard_analog_output0"), raw.get("standard_analog_output1")],
                "tool_analog_in": [raw.get("tool_analog_input0"), raw.get("tool_analog_input1")],
                "tool_output_voltage": raw.get("tool_output_voltage"),
                "tool_current": raw.get("tool_output_current"),
                "tool_temperature": raw.get("tool_temperature"),
                "tool_mode_name": name_for(TOOL_MODES, raw.get("tool_mode")),
                "program_execution_time": raw.get("actual_execution_time"),
                "unavailable_fields": list(unavailable or []),
            }
        )
    return result


def decode_bits(names: list[str], word: object) -> dict[str, bool] | None:
    """Expand a bit word into ``{flag_name: bool}``. Bits beyond the named list
    are reported as ``bit<N>`` only when set, so nothing is silently dropped."""
    if word is None:
        return None
    try:
        value = int(word)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    out: dict[str, bool] = {name: bool(value >> i & 1) for i, name in enumerate(names)}
    extra = value >> len(names)
    i = len(names)
    while extra:
        if extra & 1:
            out[f"bit{i}"] = True
        extra >>= 1
        i += 1
    return out
