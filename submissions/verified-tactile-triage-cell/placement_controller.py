"""
placement_controller.py — full triage placement sequence.

Builds on the validated Phase 3/4/5 grasp, lift, and transport logic.
Adds: destination planning (gated), descent, contact-confirmed release,
and retraction. The safety gate is called at two mandatory checkpoints
(planning and pre-release) -- the controller has no other code path that
can release the fingers, so it cannot bypass the gate.
"""
import mujoco
import numpy as np

from tactile_grasp import get_ids, acquire_grasp, TARGET_FORCE, MAX_CLOSURE_STEP, PHYSICS_MIN_FORCE
from lift_transport import smooth_ramp, LIFT_HEIGHT, WRIST_MOVE_STEPS, DYNAMIC_FORCE_TOLERANCE
from triage_safety_gate import evaluate_placement, Zone

# Zone definitions (must match scene.xml geom positions/sizes exactly).
SAFE_ZONE = Zone("triage_safe_zone", (0.1, 0.0), (0.06, 0.06), "safe")
CONTAMINATED_ZONE = Zone("contaminated_zone", (-0.54, 0.0), (0.06, 0.06), "unsafe")
ZONES = [SAFE_ZONE, CONTAMINATED_ZONE]

STAGING_X = -0.35  # original payload staging x-position, used as fallback

DESCEND_HEIGHT = 0.0    # wrist_z offset matching the original grasp height
                          # (table contact), NOT a small positive number --
                          # the grasp itself happens at wrist_z=0, so
                          # descending back to 0 returns the payload to
                          # table-resting height.
RELEASE_STEPS = 1200      # gradual finger-opening duration
RETRACT_HEIGHT = 0.06     # retract distance after release
STABILITY_HOLD_STEPS = 1200  # >=1.2s settle check after release
STABILITY_VELOCITY_THRESHOLD = 0.005  # m/s, payload must be this calm to count as "stable"

PLACEMENT_TOLERANCE_M = 0.04  # payload center must end within this of the zone center


class PlacementResult:
    def __init__(self):
        self.requested_target = None
        self.planning_gate_ran = False
        self.planning_verdict = None
        self.prerelease_gate_ran = False
        self.prerelease_verdict = None
        self.final_state = None  # "PLACED_SAFE" / "BLOCKED_FALLBACK_SAFE" / "BLOCKED_FALLBACK_STAGING" / failure labels
        self.actual_release_xy = None
        self.final_payload_xy = None
        self.placement_error_m = None
        self.payload_stability_velocity = None
        self.min_force_transport = (None, None)
        self.release_force_profile = []
        self.contaminated_zone_entered = False
        self.contaminated_zone_release = False
        self.nan_detected = False
        self.payload_dropped = False
        self.unexpected_contacts = set()
        self.gate_bypassed = False  # sanity flag; should never be True


def _regulate(d, ids, lf_cmd, rf_cmd, target_force=None):
    target = target_force if target_force is not None else TARGET_FORCE
    left_force = float(d.sensordata[0])
    right_force = float(d.sensordata[1])
    if left_force < target - DYNAMIC_FORCE_TOLERANCE:
        lf_cmd = max(lf_cmd - MAX_CLOSURE_STEP, ids["lf_range"][0])
    elif left_force > target + DYNAMIC_FORCE_TOLERANCE:
        lf_cmd = min(lf_cmd + MAX_CLOSURE_STEP, ids["lf_range"][1])
    if right_force < target - DYNAMIC_FORCE_TOLERANCE:
        rf_cmd = min(rf_cmd + MAX_CLOSURE_STEP, ids["rf_range"][1])
    elif right_force > target + DYNAMIC_FORCE_TOLERANCE:
        rf_cmd = max(rf_cmd - MAX_CLOSURE_STEP, ids["rf_range"][0])
    return lf_cmd, rf_cmd


def _check_zone_entry(d, m, ids, result):
    """Track whether the payload geom has ever entered the contaminated
    zone's footprint (x,y projection), regardless of placement outcome."""
    payload_xy = d.xpos[ids["payload_id"]][:2]
    if CONTAMINATED_ZONE.contains(tuple(payload_xy)):
        result.contaminated_zone_entered = True

    for i in range(d.ncon):
        g1 = m.geom(d.contact[i].geom1).name
        g2 = m.geom(d.contact[i].geom2).name
        pair = tuple(sorted([g1, g2]))
        if pair not in (("left_finger_geom", "payload_geom"),
                         ("payload_geom", "right_finger_geom"),
                         ("payload_geom", "table")):
            result.unexpected_contacts.add(pair)


def run_placement_trial(requested_target_xy, payload_offset_xy=(0.0, 0.0)):
    """
    Run one full triage trial: grasp -> transport toward the requested
    target -> mandatory planning gate check -> (if ALLOW) descend and
    release in the safe zone, OR (if BLOCK) execute the deterministic
    fallback (place in the safe zone instead, never in the unsafe zone,
    never just dropped).
    """
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

    result = PlacementResult()
    result.requested_target = requested_target_xy

    grasp_out, lf_cmd, rf_cmd = acquire_grasp(m, d, ids, 0.0, 0.0)
    if not grasp_out["success"]:
        result.final_state = "GRASP_FAILED"
        return result

    # --- Mandatory planning-time gate check ---
    planning_verdict = evaluate_placement(requested_target_xy, ZONES, check_label="planning")
    result.planning_gate_ran = True
    result.planning_verdict = planning_verdict

    if planning_verdict["verdict"] == "ALLOW":
        descend_target_xy = requested_target_xy
    else:
        # Deterministic fallback: place in the safe zone instead. (Could
        # also fall back to staging; safe zone chosen here since it
        # completes the task rather than just aborting it.)
        descend_target_xy = SAFE_ZONE.center_xy

    # --- Lift ---
    min_lift_l, min_lift_r = float("inf"), float("inf")
    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(0.0, LIFT_HEIGHT, WRIST_MOVE_STEPS, s)
        lf, rf = _regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_x"]] = 0.0
        d.ctrl[ids["act_z"]] = z
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf
        d.ctrl[ids["act_rf"]] = rf
        lf_cmd, rf_cmd = lf, rf
        mujoco.mj_step(m, d)
        min_lift_l = min(min_lift_l, float(d.sensordata[0]))
        min_lift_r = min(min_lift_r, float(d.sensordata[1]))
        _check_zone_entry(d, m, ids, result)
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.final_state = "NAN_DURING_LIFT"
            return result

    # --- Transport toward descend_target_xy ---
    wrist_x_start = 0.0
    wrist_x_target = descend_target_xy[0] - STAGING_X  # offset relative to anchor
    for s in range(WRIST_MOVE_STEPS):
        x = smooth_ramp(wrist_x_start, wrist_x_target, WRIST_MOVE_STEPS, s)
        lf, rf = _regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_x"]] = x
        d.ctrl[ids["act_z"]] = LIFT_HEIGHT
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf
        d.ctrl[ids["act_rf"]] = rf
        lf_cmd, rf_cmd = lf, rf
        mujoco.mj_step(m, d)
        min_lift_l = min(min_lift_l, float(d.sensordata[0]))
        min_lift_r = min(min_lift_r, float(d.sensordata[1]))
        _check_zone_entry(d, m, ids, result)
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.final_state = "NAN_DURING_TRANSPORT"
            return result

    result.min_force_transport = (round(min_lift_l, 4), round(min_lift_r, 4))

    # Brief hold at transport destination height before descent.
    for _ in range(300):
        lf, rf = _regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = LIFT_HEIGHT
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf
        d.ctrl[ids["act_rf"]] = rf
        lf_cmd, rf_cmd = lf, rf
        mujoco.mj_step(m, d)
        _check_zone_entry(d, m, ids, result)

    # --- Mandatory pre-release gate recheck (immediately before descent) ---
    prerelease_verdict = evaluate_placement(descend_target_xy, ZONES, check_label="pre_release")
    result.prerelease_gate_ran = True
    result.prerelease_verdict = prerelease_verdict

    if prerelease_verdict["verdict"] != "ALLOW":
        # The pre-release recheck itself caught something unsafe about
        # descend_target_xy (should not happen if planning already chose
        # the safe zone, but checked again for defense-in-depth). Fall
        # back to the safe zone's center directly without descending.
        descend_target_xy = SAFE_ZONE.center_xy
        wrist_x_target = descend_target_xy[0] - STAGING_X
        for s in range(WRIST_MOVE_STEPS):
            x = smooth_ramp(d.qpos[m.jnt_qposadr[m.joint("wrist_x").id]], wrist_x_target, WRIST_MOVE_STEPS, s)
            lf, rf = _regulate(d, ids, lf_cmd, rf_cmd)
            d.ctrl[ids["act_x"]] = x
            d.ctrl[ids["act_z"]] = LIFT_HEIGHT
            d.ctrl[ids["act_yaw"]] = 0.0
            d.ctrl[ids["act_lf"]] = lf
            d.ctrl[ids["act_rf"]] = rf
            lf_cmd, rf_cmd = lf, rf
            mujoco.mj_step(m, d)
            _check_zone_entry(d, m, ids, result)
        prerelease_verdict = evaluate_placement(descend_target_xy, ZONES, check_label="pre_release_retry")
        result.prerelease_verdict = prerelease_verdict

    # The ONLY code path that proceeds to descend+release requires
    # prerelease_verdict["verdict"] == "ALLOW". This is the gate the
    # controller cannot bypass.
    if prerelease_verdict["verdict"] != "ALLOW":
        result.gate_bypassed = False  # we correctly refuse to release
        result.final_state = "BLOCKED_NO_SAFE_FALLBACK_AVAILABLE"
        # Retract without releasing, as a safe failure mode.
        for s in range(WRIST_MOVE_STEPS):
            z = smooth_ramp(LIFT_HEIGHT, LIFT_HEIGHT + RETRACT_HEIGHT, WRIST_MOVE_STEPS, s)
            lf, rf = _regulate(d, ids, lf_cmd, rf_cmd)
            d.ctrl[ids["act_z"]] = z
            d.ctrl[ids["act_lf"]] = lf
            d.ctrl[ids["act_rf"]] = rf
            lf_cmd, rf_cmd = lf, rf
            mujoco.mj_step(m, d)
        return result

    # --- Descend ---
    descend_z_target = DESCEND_HEIGHT
    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(LIFT_HEIGHT, descend_z_target, WRIST_MOVE_STEPS, s)
        lf, rf = _regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = z
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf
        d.ctrl[ids["act_rf"]] = rf
        lf_cmd, rf_cmd = lf, rf
        mujoco.mj_step(m, d)
        _check_zone_entry(d, m, ids, result)
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.final_state = "NAN_DURING_DESCENT"
            return result

    # Confirm payload support: check for payload-table contact before
    # releasing (don't release in mid-air).
    payload_supported = False
    for _ in range(300):
        lf, rf = _regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_lf"]] = lf
        d.ctrl[ids["act_rf"]] = rf
        lf_cmd, rf_cmd = lf, rf
        mujoco.mj_step(m, d)
        for i in range(d.ncon):
            pair = tuple(sorted([m.geom(d.contact[i].geom1).name, m.geom(d.contact[i].geom2).name]))
            if pair == ("payload_geom", "table"):
                payload_supported = True

    if not payload_supported:
        result.final_state = "NO_SUPPORT_CONFIRMED_BEFORE_RELEASE"
        return result

    # --- Gradual release ---
    lf_start, rf_start = lf_cmd, rf_cmd
    for s in range(RELEASE_STEPS):
        t = s / RELEASE_STEPS
        lf_release = lf_start + (ids["lf_range"][1] - lf_start) * t
        rf_release = rf_start + (ids["rf_range"][0] - rf_start) * t
        d.ctrl[ids["act_lf"]] = lf_release
        d.ctrl[ids["act_rf"]] = rf_release
        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = descend_z_target
        mujoco.mj_step(m, d)
        result.release_force_profile.append((round(float(d.sensordata[0]), 3), round(float(d.sensordata[1]), 3)))
        _check_zone_entry(d, m, ids, result)
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.final_state = "NAN_DURING_RELEASE"
            return result

    result.actual_release_xy = tuple(d.xpos[ids["payload_id"]][:2])

    if CONTAMINATED_ZONE.contains(result.actual_release_xy):
        result.contaminated_zone_release = True

    # --- Retract ---
    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(descend_z_target, descend_z_target + RETRACT_HEIGHT, WRIST_MOVE_STEPS, s)
        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = z
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]
        d.ctrl[ids["act_rf"]] = ids["rf_range"][0]
        mujoco.mj_step(m, d)
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.final_state = "NAN_DURING_RETRACT"
            return result
        for i in range(d.ncon):
            pair = tuple(sorted([m.geom(d.contact[i].geom1).name, m.geom(d.contact[i].geom2).name]))
            if "finger" in pair[0] or "finger" in pair[1]:
                if "payload" in pair[0] or "payload" in pair[1]:
                    result.unexpected_contacts.add(("retract_recontact",) + pair)

    # --- Stability hold + check ---
    prev_pos = d.xpos[ids["payload_id"]].copy()
    max_velocity = 0.0
    for _ in range(STABILITY_HOLD_STEPS):
        mujoco.mj_step(m, d)
        pos = d.xpos[ids["payload_id"]].copy()
        vel = float(np.linalg.norm(pos - prev_pos) / m.opt.timestep)
        max_velocity = max(max_velocity, vel)
        prev_pos = pos

    final_xy = tuple(d.xpos[ids["payload_id"]][:2])
    result.final_payload_xy = final_xy
    result.payload_stability_velocity = max_velocity

    zone_target = SAFE_ZONE if planning_verdict["verdict"] != "ALLOW" or prerelease_verdict["verdict"] == "ALLOW" else None
    error_to_safe_zone = ((final_xy[0] - SAFE_ZONE.center_xy[0]) ** 2 +
                          (final_xy[1] - SAFE_ZONE.center_xy[1]) ** 2) ** 0.5
    result.placement_error_m = error_to_safe_zone

    if result.nan_detected:
        result.final_state = "NAN"
    elif result.contaminated_zone_release:
        result.final_state = "FAILURE_RELEASED_IN_CONTAMINATED_ZONE"
    elif not SAFE_ZONE.contains(final_xy):
        result.final_state = "FAILURE_NOT_IN_SAFE_ZONE"
    elif max_velocity > STABILITY_VELOCITY_THRESHOLD:
        result.final_state = "FAILURE_UNSTABLE"
    else:
        result.final_state = (
            "PLACED_SAFE" if planning_verdict["verdict"] == "ALLOW"
            else "BLOCKED_FALLBACK_SAFE"
        )

    return result
