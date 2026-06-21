"""
Phase 4: lift and transport.

Reuses the validated grasp acquisition from tactile_grasp.py, then adds
smooth, bounded wrist motion (lift, then horizontal transport) while
continuously regulating fingertip force via the same closed-loop logic
used during the stationary hold. Does not rely on fixed closure alone --
forces are monitored and corrected throughout.
"""
import mujoco
import numpy as np

from tactile_grasp import (
    get_ids, acquire_grasp, TARGET_FORCE, FORCE_TOLERANCE, MAX_CLOSURE_STEP,
    PHYSICS_MIN_FORCE,
)

# Tighter regulation band during lift/transport than the stationary-hold
# tolerance in tactile_grasp.py (which remains unchanged, preserving the
# validated Phase 3 5/5 gate). Dynamic motion benefits from holding closer
# to the target to minimize gradual force-driven sag/slip.
DYNAMIC_FORCE_TOLERANCE = 0.15

LIFT_HEIGHT = 0.04          # m, 40mm -- within the 30-50mm conservative range
TRANSPORT_DISTANCE = 0.12   # m, 120mm -- within the 100-150mm conservative range
HOLD_AIRBORNE_STEPS = 1200   # >=1.2s at timestep=0.001
HOLD_TRANSPORT_STEPS = 1200  # >=1.2s
WRIST_MOVE_STEPS = 1500      # steps over which a wrist motion ramps, for smoothness
SETTLE_AFTER_MOVE_STEPS = 300

TABLE_TOP_Z = 0.0            # table surface world z (table geom centered at z=-0.02, half 0.02 -> top=0.0)
NO_TABLE_CONTACT_HEIGHT = 0.005  # payload bottom must clear this much above table top


class LiftTransportResult:
    def __init__(self):
        self.success = False
        self.failure_reason = None
        self.grasp_ok = False
        self.min_force_lift = (None, None)
        self.min_force_transport = (None, None)
        self.peak_force = 0.0
        self.max_force_asymmetry = 0.0
        self.max_relative_slip = 0.0
        self.lift_height_achieved = 0.0
        self.transport_error = None
        self.airborne_duration_s = 0.0
        self.unexpected_contacts = set()
        self.actuator_saturated = False
        self.nan_detected = False
        self.table_contact_during_airborne = False


def smooth_ramp(start_val, end_val, n_steps, step_idx):
    """Smooth (cosine-eased) interpolation -- avoids instantaneous command
    jumps; velocity is zero at both endpoints."""
    t = min(step_idx / n_steps, 1.0)
    eased = 0.5 - 0.5 * np.cos(np.pi * t)
    return start_val + (end_val - start_val) * eased


def regulate_and_step(m, d, ids, wrist_x_target, wrist_z_target, wrist_yaw_target,
                       lf_cmd, rf_cmd, result, payload_pos_ref, wrist_to_payload_ref):
    """One control step: regulate finger force, apply wrist targets, step
    physics, update tracked metrics. Returns updated (lf_cmd, rf_cmd)."""
    left_force = float(d.sensordata[0])
    right_force = float(d.sensordata[1])
    result.peak_force = max(result.peak_force, left_force, right_force)

    if left_force < TARGET_FORCE - DYNAMIC_FORCE_TOLERANCE:
        lf_cmd = max(lf_cmd - MAX_CLOSURE_STEP, ids["lf_range"][0])
    elif left_force > TARGET_FORCE + DYNAMIC_FORCE_TOLERANCE:
        lf_cmd = min(lf_cmd + MAX_CLOSURE_STEP, ids["lf_range"][1])
    if right_force < TARGET_FORCE - DYNAMIC_FORCE_TOLERANCE:
        rf_cmd = min(rf_cmd + MAX_CLOSURE_STEP, ids["rf_range"][1])
    elif right_force > TARGET_FORCE + DYNAMIC_FORCE_TOLERANCE:
        rf_cmd = max(rf_cmd - MAX_CLOSURE_STEP, ids["rf_range"][0])

    if lf_cmd <= ids["lf_range"][0] + 1e-9 or rf_cmd >= ids["rf_range"][1] - 1e-9:
        result.actuator_saturated = True

    d.ctrl[ids["act_x"]] = wrist_x_target
    d.ctrl[ids["act_z"]] = wrist_z_target
    d.ctrl[ids["act_yaw"]] = wrist_yaw_target
    d.ctrl[ids["act_lf"]] = lf_cmd
    d.ctrl[ids["act_rf"]] = rf_cmd
    mujoco.mj_step(m, d)

    if np.any(np.isnan(d.qpos)) or np.any(np.isinf(d.qpos)):
        result.nan_detected = True

    for i in range(d.ncon):
        g1 = m.geom(d.contact[i].geom1).name
        g2 = m.geom(d.contact[i].geom2).name
        pair = tuple(sorted([g1, g2]))
        if pair == ("payload_geom", "table"):
            # Only count this as an "airborne table contact" problem if the
            # payload has clearly cleared the table (well above its resting
            # height). Contact at/near resting height during the initial
            # lift-off transition is expected, not a failure.
            payload_z = d.xpos[ids["payload_id"]][2]
            if payload_z > 0.0178 + 0.01:  # 1cm above resting height
                result.table_contact_during_airborne = True
        elif pair not in (("left_finger_geom", "payload_geom"),
                           ("payload_geom", "right_finger_geom")):
            result.unexpected_contacts.add(pair)

    # Relative slip: how far the payload has moved relative to the wrist's
    # rigid frame (not the finger midpoint, which moves independently as
    # the closure regulator adjusts lf_cmd/rf_cmd -- using the wrist frame
    # isolates true payload-vs-gripper slip from finger-closure noise).
    wrist_pos = d.xpos[m.body("wrist").id]
    payload_pos = d.xpos[ids["payload_id"]]
    rel_pos = payload_pos - wrist_pos
    slip = float(np.linalg.norm(rel_pos - wrist_to_payload_ref))
    result.max_relative_slip = max(result.max_relative_slip, slip)

    return lf_cmd, rf_cmd, left_force, right_force


def run_lift_transport_trial(payload_offset_xy=(0.0, 0.0)):
    m = mujoco.MjModel.from_xml_path("scene.xml")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    ids = get_ids(m)

    payload_jid = m.joint("payload_free").id
    payload_qpos_adr = m.jnt_qposadr[payload_jid]
    d.qpos[payload_qpos_adr] += payload_offset_xy[0]
    d.qpos[payload_qpos_adr + 1] += payload_offset_xy[1]
    mujoco.mj_forward(m, d)

    for _ in range(500):
        d.ctrl[ids["act_x"]] = 0.0
        d.ctrl[ids["act_z"]] = 0.0
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]
        d.ctrl[ids["act_rf"]] = ids["rf_range"][0]
        mujoco.mj_step(m, d)

    result = LiftTransportResult()

    # --- Acquire grasp (reused, validated logic) ---
    grasp_out, lf_cmd, rf_cmd = acquire_grasp(m, d, ids, wrist_x_target=0.0, wrist_z_target=0.0)
    result.grasp_ok = grasp_out["success"]
    result.actuator_saturated = grasp_out["actuator_saturated"]
    result.nan_detected = grasp_out["nan_detected"]
    result.unexpected_contacts |= grasp_out["unexpected_contacts"]

    if not grasp_out["success"] or grasp_out["nan_detected"]:
        result.failure_reason = grasp_out["failure_reason"] or "grasp_failed"
        return result

    left_force = float(d.sensordata[0])
    right_force = float(d.sensordata[1])
    if left_force < PHYSICS_MIN_FORCE or right_force < PHYSICS_MIN_FORCE:
        result.failure_reason = f"force_below_physics_floor_before_lift (L={left_force:.3f} R={right_force:.3f})"
        return result

    # Reference: payload position relative to fingertip midpoint, captured
    # right after grasp acquisition (before any lift motion) -- used as the
    # zero-slip baseline.
    wrist_pos = d.xpos[m.body("wrist").id]
    wrist_to_payload_ref = (d.xpos[ids["payload_id"]] - wrist_pos).copy()

    wrist_x_at_grasp = 0.0
    wrist_z_at_grasp = 0.0

    # --- Lift: smooth ramp from current z to current z + LIFT_HEIGHT ---
    lift_min_left = float("inf")
    lift_min_right = float("inf")
    for step in range(WRIST_MOVE_STEPS):
        z_target = smooth_ramp(wrist_z_at_grasp, wrist_z_at_grasp + LIFT_HEIGHT, WRIST_MOVE_STEPS, step)
        lf_cmd, rf_cmd, lf_force, rf_force = regulate_and_step(
            m, d, ids, wrist_x_at_grasp, z_target, 0.0, lf_cmd, rf_cmd,
            result, None, wrist_to_payload_ref,
        )
        lift_min_left = min(lift_min_left, lf_force)
        lift_min_right = min(lift_min_right, rf_force)
        if result.nan_detected:
            result.failure_reason = "nan_detected_during_lift"
            return result

    result.min_force_lift = (round(lift_min_left, 4), round(lift_min_right, 4))

    # --- Hold airborne ---
    for _ in range(HOLD_AIRBORNE_STEPS):
        lf_cmd, rf_cmd, lf_force, rf_force = regulate_and_step(
            m, d, ids, wrist_x_at_grasp, wrist_z_at_grasp + LIFT_HEIGHT, 0.0,
            lf_cmd, rf_cmd, result, None, wrist_to_payload_ref,
        )
        lift_min_left = min(lift_min_left, lf_force)
        lift_min_right = min(lift_min_right, rf_force)
        if result.nan_detected:
            result.failure_reason = "nan_detected_during_airborne_hold"
            return result

    result.min_force_lift = (round(lift_min_left, 4), round(lift_min_right, 4))
    result.airborne_duration_s += HOLD_AIRBORNE_STEPS * 0.001

    payload_z_airborne = d.xpos[ids["payload_id"]][2]
    result.lift_height_achieved = float(payload_z_airborne - 0.0178)  # vs original resting z

    # --- Transport horizontally ---
    transport_min_left = float("inf")
    transport_min_right = float("inf")
    for step in range(WRIST_MOVE_STEPS):
        x_target = smooth_ramp(wrist_x_at_grasp, wrist_x_at_grasp + TRANSPORT_DISTANCE,
                                WRIST_MOVE_STEPS, step)
        lf_cmd, rf_cmd, lf_force, rf_force = regulate_and_step(
            m, d, ids, x_target, wrist_z_at_grasp + LIFT_HEIGHT, 0.0,
            lf_cmd, rf_cmd, result, None, wrist_to_payload_ref,
        )
        transport_min_left = min(transport_min_left, lf_force)
        transport_min_right = min(transport_min_right, rf_force)
        if result.nan_detected:
            result.failure_reason = "nan_detected_during_transport"
            return result
    result.airborne_duration_s += WRIST_MOVE_STEPS * 0.001

    # --- Hold at transport destination ---
    for _ in range(HOLD_TRANSPORT_STEPS):
        lf_cmd, rf_cmd, lf_force, rf_force = regulate_and_step(
            m, d, ids, wrist_x_at_grasp + TRANSPORT_DISTANCE, wrist_z_at_grasp + LIFT_HEIGHT, 0.0,
            lf_cmd, rf_cmd, result, None, wrist_to_payload_ref,
        )
        transport_min_left = min(transport_min_left, lf_force)
        transport_min_right = min(transport_min_right, rf_force)
        if result.nan_detected:
            result.failure_reason = "nan_detected_during_transport_hold"
            return result
    result.airborne_duration_s += HOLD_TRANSPORT_STEPS * 0.001

    result.min_force_transport = (round(transport_min_left, 4), round(transport_min_right, 4))

    payload_xy_transported = d.xpos[ids["payload_id"]][:2].copy()
    expected_payload_x = -0.35 + payload_offset_xy[0] + TRANSPORT_DISTANCE
    result.transport_error = float(abs(payload_xy_transported[0] - expected_payload_x))

    # --- Lower back down and release (no uncontrolled drop) ---
    for step in range(WRIST_MOVE_STEPS):
        z_target = smooth_ramp(wrist_z_at_grasp + LIFT_HEIGHT, wrist_z_at_grasp, WRIST_MOVE_STEPS, step)
        lf_cmd, rf_cmd, lf_force, rf_force = regulate_and_step(
            m, d, ids, wrist_x_at_grasp + TRANSPORT_DISTANCE, z_target, 0.0,
            lf_cmd, rf_cmd, result, None, wrist_to_payload_ref,
        )
        if result.nan_detected:
            result.failure_reason = "nan_detected_during_lower"
            return result

    for _ in range(1000):
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]
        d.ctrl[ids["act_rf"]] = ids["rf_range"][0]
        mujoco.mj_step(m, d)
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.failure_reason = "nan_detected_during_release"
            return result

    # --- Final asymmetry over the whole transport-hold window already
    # captured via min forces; compute a representative asymmetry from the
    # final held forces. ---
    final_left = float(d.sensordata[0])
    final_right = float(d.sensordata[1])
    result.max_force_asymmetry = float(abs(result.min_force_transport[0] - result.min_force_transport[1]))

    # --- Success criteria ---
    if result.nan_detected:
        result.failure_reason = "nan_detected"
    elif result.table_contact_during_airborne:
        result.failure_reason = "table_contact_during_airborne_hold"
    elif min(result.min_force_lift) < PHYSICS_MIN_FORCE:
        result.failure_reason = f"force_below_physics_floor_during_lift {result.min_force_lift}"
    elif min(result.min_force_transport) < PHYSICS_MIN_FORCE:
        result.failure_reason = f"force_below_physics_floor_during_transport {result.min_force_transport}"
    elif len(result.unexpected_contacts) > 0:
        result.failure_reason = f"unexpected_contacts {result.unexpected_contacts}"
    elif result.lift_height_achieved < 0.025:  # must clearly leave the table
        result.failure_reason = f"insufficient_lift_height ({result.lift_height_achieved:.4f}m)"
    elif result.transport_error > 0.01:  # 1cm tolerance
        result.failure_reason = f"transport_error_exceeded ({result.transport_error:.4f}m)"
    elif result.max_relative_slip > 0.005:  # 5mm relative-slip tolerance (natural, not injected)
        result.failure_reason = f"excessive_relative_slip ({result.max_relative_slip:.4f}m)"
    else:
        result.success = True

    return result
