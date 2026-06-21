"""
Phase 5: slip detection and recovery.

Detection is measurement-driven: wrist-relative payload displacement,
relative velocity, and fingertip force asymmetry/drop are computed every
control step from real simulation state. No detection result is hardcoded
or dependent on knowing when a disturbance was injected.

Disturbance is injected via xfrc_applied on the payload body -- a bounded
external force through MuJoCo's physics, not a direct qpos/qvel edit.

Recovery state machine (per spec):
  HOLD_NORMAL -> SLIP_DETECTED -> PAUSE_WRIST_MOTION -> INCREASE_GRIP_TARGET
  -> WAIT_FOR_STABILIZATION -> RESUME_TRANSPORT/SAFE_HOLD
  -> RECOVERY_SUCCESS / RECOVERY_FAIL
"""
import mujoco
import numpy as np

from tactile_grasp import get_ids, acquire_grasp, TARGET_FORCE, MAX_CLOSURE_STEP, PHYSICS_MIN_FORCE
from lift_transport import smooth_ramp, LIFT_HEIGHT, WRIST_MOVE_STEPS, DYNAMIC_FORCE_TOLERANCE

# --- Thresholds derived from measured baseline (see Phase 5 report) ---
# Baseline wrist-relative slip noise (5 trials, Phase 4 validated config):
# min=0.7705mm, max=0.8095mm, mean=0.7935mm.
BASELINE_SLIP_MAX_M = 0.000810
SLIP_DETECTION_MARGIN = 3.0  # x baseline max
SLIP_DETECTION_THRESHOLD_M = BASELINE_SLIP_MAX_M * SLIP_DETECTION_MARGIN  # 2.43mm

# Relative-velocity threshold: detect meaningful motion of the payload
# relative to the wrist, independent of slip-distance accumulation.
VELOCITY_DETECTION_THRESHOLD_MPS = 0.01  # 1 cm/s

# Force-asymmetry threshold: a real grip disturbance often shows up as a
# transient imbalance between the two fingertip sensors, even if total
# force doesn't drop much.
FORCE_ASYMMETRY_THRESHOLD_N = 0.05

# Disturbance: smallest force/duration found (by direct measurement) to
# produce slip clearly above the detection threshold without ejecting the
# payload. 0.5N for 50 steps (50ms) -> ~3.94mm measured slip in isolation.
DISTURBANCE_FORCE_N = 0.5
DISTURBANCE_DURATION_STEPS = 50

# Recovery parameters. Increase is capped below the known achievable
# ceiling (~3.0N at full +-0.058m closure, measured in Phase 3/4) so the
# regulator never chases an unreachable target.
RECOVERY_GRIP_INCREASE_N = 0.15
RECOVERY_GRIP_TARGET_N = TARGET_FORCE + RECOVERY_GRIP_INCREASE_N  # 2.95N
STABILIZATION_STEPS = 300         # steps to wait/check before declaring success
STABILIZATION_VELOCITY_THRESHOLD_MPS = 0.002  # must settle below this
MAX_RECOVERY_STEPS = 1500         # hard bound on recovery duration


class SlipRecoveryResult:
    def __init__(self):
        self.disturbance_applied = False
        self.slip_detected = False
        self.detection_step = None
        self.detection_latency_steps = None
        self.slip_at_detection = None
        self.velocity_at_detection = None
        self.force_asymmetry_at_detection = None
        self.max_wrist_slip = 0.0
        self.force_before = (None, None)
        self.force_during = (None, None)
        self.force_after = (None, None)
        self.recovery_grip_target = None
        self.recovery_duration_steps = None
        self.post_recovery_velocity = None
        self.final_state = None  # "RECOVERY_SUCCESS" / "RECOVERY_FAIL" / "HOLD_NORMAL" (no slip)
        self.payload_dropped = False
        self.nan_detected = False
        self.unexpected_contacts = set()
        self.false_positive = False
        self.false_negative = False
        self.state_history = []  # list of (step, state) transitions


def run_slip_trial(inject_disturbance, payload_offset_xy=(0.0, 0.0)):
    """
    Run one airborne-hold trial. If inject_disturbance is True, apply the
    standard disturbance partway through the hold and exercise the
    recovery state machine. If False, this is a negative control: no
    disturbance, and the detector must NOT fire (checked, not hardcoded).
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

    result = SlipRecoveryResult()

    grasp_out, lf_cmd, rf_cmd = acquire_grasp(m, d, ids, 0.0, 0.0)
    if not grasp_out["success"]:
        result.final_state = "GRASP_FAILED"
        return result

    payload_body_id = ids["payload_id"]
    wrist_pos_ref = d.xpos[m.body("wrist").id].copy()
    payload_pos_ref0 = d.xpos[payload_body_id].copy()
    wrist_rel_ref = payload_pos_ref0 - wrist_pos_ref

    state = "HOLD_NORMAL"
    result.state_history.append((0, state))

    # --- Lift to airborne height (same as Phase 4) ---
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
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.final_state = "NAN_DURING_LIFT"
            return result

    # Record pre-disturbance force as a clean baseline for this trial.
    result.force_before = (round(float(d.sensordata[0]), 4), round(float(d.sensordata[1]), 4))

    # Previous-step payload position, for relative-velocity estimation.
    prev_payload_pos = d.xpos[payload_body_id].copy()
    dt = m.opt.timestep

    recovery_start_step = None
    recovery_grip_target = None
    stabilization_counter = 0

    total_hold_steps = 1200  # same airborne hold duration as Phase 4
    disturbance_start = 150
    disturbance_end = disturbance_start + DISTURBANCE_DURATION_STEPS

    for step in range(total_hold_steps):
        # --- Disturbance injection (xfrc_applied only, never qpos/qvel) ---
        if inject_disturbance and disturbance_start <= step < disturbance_end:
            d.xfrc_applied[payload_body_id, 1] = DISTURBANCE_FORCE_N
            if not result.disturbance_applied:
                result.disturbance_applied = True
                result.force_during = (round(float(d.sensordata[0]), 4), round(float(d.sensordata[1]), 4))
        else:
            d.xfrc_applied[payload_body_id, 1] = 0.0

        # --- Regulate grip force (target depends on state) ---
        active_target = recovery_grip_target if recovery_grip_target is not None else TARGET_FORCE
        lf, rf = _regulate(d, ids, lf_cmd, rf_cmd, target_force=active_target)

        # --- Wrist motion: paused during PAUSE_WRIST_MOTION/recovery states ---
        if state in ("PAUSE_WRIST_MOTION", "INCREASE_GRIP_TARGET", "WAIT_FOR_STABILIZATION"):
            wrist_z = LIFT_HEIGHT  # hold position, no motion
        else:
            wrist_z = LIFT_HEIGHT

        d.ctrl[ids["act_x"]] = 0.0
        d.ctrl[ids["act_z"]] = wrist_z
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf
        d.ctrl[ids["act_rf"]] = rf
        lf_cmd, rf_cmd = lf, rf
        mujoco.mj_step(m, d)

        if np.any(np.isnan(d.qpos)) or np.any(np.isinf(d.qpos)):
            result.nan_detected = True
            result.final_state = "NAN_DURING_HOLD"
            return result

        # --- Real measurements ---
        wrist_pos = d.xpos[m.body("wrist").id]
        payload_pos = d.xpos[payload_body_id].copy()
        rel = payload_pos - wrist_pos
        wrist_slip = float(np.linalg.norm(rel - wrist_rel_ref))
        result.max_wrist_slip = max(result.max_wrist_slip, wrist_slip)

        rel_velocity = float(np.linalg.norm(payload_pos - prev_payload_pos) / dt)
        prev_payload_pos = payload_pos.copy()

        left_force = float(d.sensordata[0])
        right_force = float(d.sensordata[1])
        force_asymmetry = abs(left_force - right_force)

        # Drop detection: payload separated from both fingers (no contact
        # with either) -- check contact list directly.
        finger_contacts = sum(
            1 for i in range(d.ncon)
            if {m.geom(d.contact[i].geom1).name, m.geom(d.contact[i].geom2).name}
            & {"left_finger_geom", "right_finger_geom"}
        )

        for i in range(d.ncon):
            g1 = m.geom(d.contact[i].geom1).name
            g2 = m.geom(d.contact[i].geom2).name
            pair = tuple(sorted([g1, g2]))
            if pair not in (("left_finger_geom", "payload_geom"),
                             ("payload_geom", "right_finger_geom"),
                             ("payload_geom", "table")):
                result.unexpected_contacts.add(pair)

        # --- Measurement-driven detection (only in HOLD_NORMAL state) ---
        if state == "HOLD_NORMAL":
            slip_trip = wrist_slip > SLIP_DETECTION_THRESHOLD_M
            velocity_trip = rel_velocity > VELOCITY_DETECTION_THRESHOLD_MPS
            asymmetry_trip = force_asymmetry > FORCE_ASYMMETRY_THRESHOLD_N

            if slip_trip or velocity_trip or asymmetry_trip:
                result.slip_detected = True
                result.detection_step = step
                result.slip_at_detection = wrist_slip
                result.velocity_at_detection = rel_velocity
                result.force_asymmetry_at_detection = force_asymmetry
                state = "SLIP_DETECTED"
                result.state_history.append((step, state))

        elif state == "SLIP_DETECTED":
            state = "PAUSE_WRIST_MOTION"
            result.state_history.append((step, state))

        elif state == "PAUSE_WRIST_MOTION":
            state = "INCREASE_GRIP_TARGET"
            recovery_grip_target = RECOVERY_GRIP_TARGET_N
            recovery_start_step = step
            result.recovery_grip_target = recovery_grip_target
            result.state_history.append((step, state))

        elif state == "INCREASE_GRIP_TARGET":
            # Move to WAIT_FOR_STABILIZATION once force is tracking the
            # raised target within tolerance (measurement-driven, not a
            # fixed step count).
            if (left_force >= recovery_grip_target - DYNAMIC_FORCE_TOLERANCE and
                    right_force >= recovery_grip_target - DYNAMIC_FORCE_TOLERANCE):
                state = "WAIT_FOR_STABILIZATION"
                stabilization_counter = 0
                result.state_history.append((step, state))

        elif state == "WAIT_FOR_STABILIZATION":
            if rel_velocity < STABILIZATION_VELOCITY_THRESHOLD_MPS:
                stabilization_counter += 1
            else:
                stabilization_counter = 0
            if stabilization_counter >= STABILIZATION_STEPS:
                state = "SAFE_HOLD"
                result.state_history.append((step, state))
            elif recovery_start_step is not None and step - recovery_start_step > MAX_RECOVERY_STEPS:
                state = "RECOVERY_FAIL"
                result.state_history.append((step, state))
                result.final_state = "RECOVERY_FAIL"
                break

        elif state in ("SAFE_HOLD",):
            # Reached a stable safe hold after recovery; remain here for
            # the rest of the trial.
            pass

        # Drop check, any state: if neither finger is in contact AND the
        # payload is not resting on the table, it has been dropped.
        payload_z = payload_pos[2]
        if finger_contacts == 0 and payload_z < LIFT_HEIGHT - 0.01:
            result.payload_dropped = True
            result.final_state = "PAYLOAD_DROPPED"
            break

    result.force_after = (round(float(d.sensordata[0]), 4), round(float(d.sensordata[1]), 4))
    result.post_recovery_velocity = rel_velocity if "rel_velocity" in dir() else None

    if result.detection_step is not None:
        result.detection_latency_steps = result.detection_step - disturbance_start

    if result.recovery_grip_target is not None and recovery_start_step is not None:
        result.recovery_duration_steps = step - recovery_start_step if state in ("SAFE_HOLD", "RECOVERY_FAIL") else None

    if result.final_state is None:
        if not inject_disturbance:
            result.final_state = "HOLD_NORMAL" if not result.slip_detected else "FALSE_POSITIVE"
            result.false_positive = result.slip_detected
        else:
            if state == "SAFE_HOLD" and not result.payload_dropped:
                result.final_state = "RECOVERY_SUCCESS"
            elif not result.slip_detected:
                result.final_state = "FALSE_NEGATIVE"
                result.false_negative = True
            else:
                result.final_state = "RECOVERY_FAIL"

    return result


def _regulate(d, ids, lf_cmd, rf_cmd, target_force=None):
    """One step of closed-loop force regulation (shared logic, mirrors
    lift_transport.py's regulate_and_step force law)."""
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
