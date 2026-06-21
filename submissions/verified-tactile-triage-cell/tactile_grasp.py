"""
Closed-loop tactile grasp controller.

Finger closure is driven by real fingertip touch-sensor feedback, not a
fixed scripted position. Each finger independently increments its closure
command in small steps until its own touch sensor reports force at or
above the per-finger target, then holds. This is a simple proportional
force-tracking loop, not a position trajectory.

TARGET_FORCE derivation (Phase 3, physics-based, not arbitrary):
  payload mass m = 0.05 kg (read from scene.xml)
  friction coefficient mu = 0.9 (read from scene.xml, MuJoCo min-combine rule)
  g = 9.81 m/s^2
  Static-lift requirement per finger: F_min = (m * g) / (2 * mu) = 0.2725 N
  Safety factor: 2.0x (covers dynamic loading during lift/transport and
    contact-solver/friction-model uncertainty)
  Required per-finger force with margin: 0.2725 * 2.0 = 0.545 N

Phase 4 correction (measured, not theoretical):
  The analytical F_min above assumes the touch sensor's reported per-finger
  force IS the relevant normal force for the friction calculation. Direct
  inspection of mj_contactForce during Phase 4 lift/transport testing
  showed each finger-payload contact is actually FOUR separate contact
  points (box-box corner contacts), and the friction force actually being
  applied at the original TARGET_FORCE=2.35N was only marginally sufficient
  to offset payload weight during sustained airborne holding -- measured
  slip grew at a constant ~0.0013 m/s even with rock-steady, non-decaying
  contact force, confirming genuine (if slow) kinetic slip rather than a
  control bug. Raising TARGET_FORCE (and the finger joint range enough to
  reach it without saturating the closed-loop regulator) eliminates this.
  Finger range extended from the original +-0.05 to +-0.058 (8mm total,
  validated: penetration stays sub-millimeter, no NaN, no instability at
  the new limit -- see Phase 4 notes). TARGET_FORCE raised to 2.8N
  (achievable without saturating at the new range, comfortably below the
  ~3.0N ceiling at full closure).
"""
import mujoco
import numpy as np

TARGET_FORCE = 2.8           # N, per-finger target -- revised in Phase 4, see derivation above
FORCE_TOLERANCE = 0.4         # N, band around target; keeps regulator from
                               # fighting against the hard joint-range limit
MAX_CLOSURE_STEP = 0.0006    # m, max finger position change per control step
CONTACT_TIMEOUT_STEPS = 4000  # safety bound on how long to wait for contact
HOLD_STEPS = 2200             # >=2.2s at timestep=0.001, hold period after grasp

# Physics-derived minimum (static-lift requirement with 2.0x safety factor).
# Used directly in success criteria so pass/fail is traceable to the
# derivation, not just to the (slightly more generous) operating target.
PAYLOAD_MASS_KG = 0.05
FRICTION_COEF = 0.9
GRAVITY = 9.81
SAFETY_FACTOR = 2.0
PHYSICS_MIN_FORCE = (PAYLOAD_MASS_KG * GRAVITY) / (2 * FRICTION_COEF) * SAFETY_FACTOR  # 0.545 N


class GraspResult:
    def __init__(self):
        self.success = False
        self.failure_reason = None
        self.contact_acquired_step = None
        self.left_peak_force = 0.0
        self.right_peak_force = 0.0
        self.left_steady_force = 0.0
        self.right_steady_force = 0.0
        self.max_force_asymmetry = 0.0
        self.payload_max_displacement = 0.0
        self.actuator_saturated = False
        self.nan_detected = False
        self.unexpected_contacts = set()
        self.left_force_history = []
        self.right_force_history = []
        self.payload_pos_history = []


def get_ids(m):
    return {
        "act_x": m.actuator("act_wrist_x").id,
        "act_z": m.actuator("act_wrist_z").id,
        "act_yaw": m.actuator("act_wrist_yaw").id,
        "act_lf": m.actuator("act_left_finger").id,
        "act_rf": m.actuator("act_right_finger").id,
        "jnt_x_adr": m.jnt_qposadr[m.joint("wrist_x").id],
        "jnt_z_adr": m.jnt_qposadr[m.joint("wrist_z").id],
        "jnt_yaw_adr": m.jnt_qposadr[m.joint("wrist_yaw").id],
        "jnt_lf_adr": m.jnt_qposadr[m.joint("left_finger_close").id],
        "jnt_rf_adr": m.jnt_qposadr[m.joint("right_finger_close").id],
        "lf_range": m.jnt_range[m.joint("left_finger_close").id].copy(),
        "rf_range": m.jnt_range[m.joint("right_finger_close").id].copy(),
        "payload_id": m.body("payload").id,
    }


def move_wrist_to_pregrasp(m, d, ids, wrist_x_target, wrist_z_target, n_steps=2000):
    """Move wrist to a pre-grasp pose (fingers open, positioned around the
    payload) before any grasp force control begins."""
    for _ in range(n_steps):
        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = wrist_z_target
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]  # fully open (left: range max = 0)
        d.ctrl[ids["act_rf"]] = ids["rf_range"][0]  # fully open (right: range min = 0)
        mujoco.mj_step(m, d)


def acquire_grasp(m, d, ids, wrist_x_target, wrist_z_target, wrist_yaw_target=0.0):
    """
    Run closed-loop tactile closure until bilateral target force is
    reached (or timeout). Mutates d in place via mj_step. Returns a dict
    with the same fields a GraspResult would track for this phase, plus
    the final (lf_cmd, rf_cmd) so callers can continue regulating force
    afterward (e.g. during lift/transport).

    This is the exact acquisition logic used by run_tactile_grasp_trial,
    extracted so it can be reused by lift/transport without duplication.
    Behavior is unchanged from the original inline version.
    """
    payload_pos_at_settle = d.xpos[ids["payload_id"]].copy()

    out = {
        "left_peak_force": 0.0,
        "right_peak_force": 0.0,
        "payload_max_displacement": 0.0,
        "actuator_saturated": False,
        "nan_detected": False,
        "unexpected_contacts": set(),
        "contact_acquired_step": None,
        "success": False,
        "failure_reason": None,
    }

    lf_cmd = ids["lf_range"][1]
    rf_cmd = ids["rf_range"][0]
    bilateral_contact_step = None

    for step in range(CONTACT_TIMEOUT_STEPS):
        left_force = float(d.sensordata[0])
        right_force = float(d.sensordata[1])
        out["left_peak_force"] = max(out["left_peak_force"], left_force)
        out["right_peak_force"] = max(out["right_peak_force"], right_force)

        if left_force < TARGET_FORCE:
            lf_cmd = max(lf_cmd - MAX_CLOSURE_STEP, ids["lf_range"][0])
        if right_force < TARGET_FORCE:
            rf_cmd = min(rf_cmd + MAX_CLOSURE_STEP, ids["rf_range"][1])

        if lf_cmd <= ids["lf_range"][0] + 1e-9 or rf_cmd >= ids["rf_range"][1] - 1e-9:
            out["actuator_saturated"] = True

        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = wrist_z_target
        d.ctrl[ids["act_yaw"]] = wrist_yaw_target
        d.ctrl[ids["act_lf"]] = lf_cmd
        d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)

        if np.any(np.isnan(d.qpos)) or np.any(np.isinf(d.qpos)):
            out["nan_detected"] = True
            out["failure_reason"] = "nan_detected"
            return out, lf_cmd, rf_cmd

        pos = d.xpos[ids["payload_id"]].copy()
        disp = float(np.linalg.norm(pos - payload_pos_at_settle))
        out["payload_max_displacement"] = max(out["payload_max_displacement"], disp)

        for i in range(d.ncon):
            g1 = m.geom(d.contact[i].geom1).name
            g2 = m.geom(d.contact[i].geom2).name
            pair = tuple(sorted([g1, g2]))
            if pair not in (("payload_geom", "table"),
                             ("left_finger_geom", "payload_geom"),
                             ("payload_geom", "right_finger_geom")):
                out["unexpected_contacts"].add(pair)

        if (left_force >= TARGET_FORCE - FORCE_TOLERANCE and
                right_force >= TARGET_FORCE - FORCE_TOLERANCE and
                bilateral_contact_step is None):
            bilateral_contact_step = step

        if bilateral_contact_step is not None and step >= bilateral_contact_step + 50:
            break
    else:
        out["failure_reason"] = "contact_not_acquired_in_time"
        return out, lf_cmd, rf_cmd

    if bilateral_contact_step is None:
        out["failure_reason"] = "contact_not_acquired_in_time"
        return out, lf_cmd, rf_cmd

    out["contact_acquired_step"] = bilateral_contact_step
    out["success"] = True
    return out, lf_cmd, rf_cmd


def run_tactile_grasp_trial(payload_offset_xy=(0.0, 0.0), wrist_x_target=0.0,
                             wrist_z_target=0.0, n_settle_steps=500):
    """
    Run one full grasp trial: settle, move to pre-grasp, close fingers
    under tactile feedback until bilateral target force is reached, hold,
    then open and reset. Returns a GraspResult.

    payload_offset_xy: small (dx, dy) perturbation applied to the payload's
    starting position, to test grasp robustness beyond one exact pose.
    """
    m = mujoco.MjModel.from_xml_path("scene.xml")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    ids = get_ids(m)

    # Apply payload position perturbation before settling.
    payload_jid = m.joint("payload_free").id
    payload_qpos_adr = m.jnt_qposadr[payload_jid]
    d.qpos[payload_qpos_adr] += payload_offset_xy[0]
    d.qpos[payload_qpos_adr + 1] += payload_offset_xy[1]
    mujoco.mj_forward(m, d)

    for _ in range(n_settle_steps):
        # Hold wrist/fingers at neutral-open during initial settle.
        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = wrist_z_target
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]
        d.ctrl[ids["act_rf"]] = ids["rf_range"][0]
        mujoco.mj_step(m, d)

    payload_pos_at_settle = d.xpos[ids["payload_id"]].copy()

    result = GraspResult()

    # Closed-loop tactile closure: increment each finger's closure command
    # toward the payload only while its own sensor force is below target.
    # Left finger: open=range[1]=0, closed=range[0]=-0.05 (closing decreases qpos).
    # Right finger: open=range[0]=0, closed=range[1]=0.05 (closing increases qpos).
    lf_cmd = ids["lf_range"][1]
    rf_cmd = ids["rf_range"][0]
    bilateral_contact_step = None

    for step in range(CONTACT_TIMEOUT_STEPS):
        left_force = float(d.sensordata[0])
        right_force = float(d.sensordata[1])
        result.left_force_history.append(left_force)
        result.right_force_history.append(right_force)
        result.left_peak_force = max(result.left_peak_force, left_force)
        result.right_peak_force = max(result.right_peak_force, right_force)

        if left_force < TARGET_FORCE:
            lf_cmd = max(lf_cmd - MAX_CLOSURE_STEP, ids["lf_range"][0])
        if right_force < TARGET_FORCE:
            rf_cmd = min(rf_cmd + MAX_CLOSURE_STEP, ids["rf_range"][1])

        # Detect actuator saturation (commanded at joint limit).
        if lf_cmd <= ids["lf_range"][0] + 1e-9 or rf_cmd >= ids["rf_range"][1] - 1e-9:
            result.actuator_saturated = True

        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = wrist_z_target
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf_cmd
        d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)

        if np.any(np.isnan(d.qpos)) or np.any(np.isinf(d.qpos)):
            result.nan_detected = True
            result.failure_reason = "nan_detected"
            return result

        pos = d.xpos[ids["payload_id"]].copy()
        result.payload_pos_history.append(pos.copy())
        disp = float(np.linalg.norm(pos - payload_pos_at_settle))
        result.payload_max_displacement = max(result.payload_max_displacement, disp)

        for i in range(d.ncon):
            g1 = m.geom(d.contact[i].geom1).name
            g2 = m.geom(d.contact[i].geom2).name
            pair = tuple(sorted([g1, g2]))
            if pair not in (("payload_geom", "table"),
                             ("left_finger_geom", "payload_geom"),
                             ("payload_geom", "right_finger_geom")):
                result.unexpected_contacts.add(pair)

        if (left_force >= TARGET_FORCE - FORCE_TOLERANCE and
                right_force >= TARGET_FORCE - FORCE_TOLERANCE and
                bilateral_contact_step is None):
            bilateral_contact_step = step

        if bilateral_contact_step is not None and step >= bilateral_contact_step + 50:
            # Bilateral contact achieved and stable for a short confirmation
            # window -- proceed to hold phase.
            break
    else:
        result.failure_reason = "contact_not_acquired_in_time"
        return result

    if bilateral_contact_step is None:
        result.failure_reason = "contact_not_acquired_in_time"
        return result

    result.contact_acquired_step = bilateral_contact_step

    # Hold phase: maintain current closure commands, keep regulating force
    # with the same proportional loop, track steady-state force.
    hold_left_forces = []
    hold_right_forces = []
    for step in range(HOLD_STEPS):
        left_force = float(d.sensordata[0])
        right_force = float(d.sensordata[1])
        hold_left_forces.append(left_force)
        hold_right_forces.append(right_force)
        result.left_peak_force = max(result.left_peak_force, left_force)
        result.right_peak_force = max(result.right_peak_force, right_force)

        if left_force < TARGET_FORCE - FORCE_TOLERANCE:
            lf_cmd = max(lf_cmd - MAX_CLOSURE_STEP, ids["lf_range"][0])
        elif left_force > TARGET_FORCE + FORCE_TOLERANCE:
            lf_cmd = min(lf_cmd + MAX_CLOSURE_STEP, ids["lf_range"][1])
        if right_force < TARGET_FORCE - FORCE_TOLERANCE:
            rf_cmd = min(rf_cmd + MAX_CLOSURE_STEP, ids["rf_range"][1])
        elif right_force > TARGET_FORCE + FORCE_TOLERANCE:
            rf_cmd = max(rf_cmd - MAX_CLOSURE_STEP, ids["rf_range"][0])

        d.ctrl[ids["act_x"]] = wrist_x_target
        d.ctrl[ids["act_z"]] = wrist_z_target
        d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf_cmd
        d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)

        if np.any(np.isnan(d.qpos)) or np.any(np.isinf(d.qpos)):
            result.nan_detected = True
            result.failure_reason = "nan_detected_during_hold"
            return result

        pos = d.xpos[ids["payload_id"]].copy()
        disp = float(np.linalg.norm(pos - payload_pos_at_settle))
        result.payload_max_displacement = max(result.payload_max_displacement, disp)

        for i in range(d.ncon):
            g1 = m.geom(d.contact[i].geom1).name
            g2 = m.geom(d.contact[i].geom2).name
            pair = tuple(sorted([g1, g2]))
            if pair not in (("payload_geom", "table"),
                             ("left_finger_geom", "payload_geom"),
                             ("payload_geom", "right_finger_geom")):
                result.unexpected_contacts.add(pair)

    result.left_steady_force = float(np.mean(hold_left_forces[-200:]))
    result.right_steady_force = float(np.mean(hold_right_forces[-200:]))
    result.max_force_asymmetry = float(abs(result.left_steady_force - result.right_steady_force))

    # Release: open fingers cleanly, reset.
    for _ in range(1000):
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]
        d.ctrl[ids["act_rf"]] = ids["rf_range"][0]
        mujoco.mj_step(m, d)
        if np.any(np.isnan(d.qpos)):
            result.nan_detected = True
            result.failure_reason = "nan_detected_during_release"
            return result

    final_left_force = float(d.sensordata[0])
    final_right_force = float(d.sensordata[1])

    # Success criteria check.
    if result.payload_max_displacement > 0.010:  # 1cm tolerance for grasp/hold phase
        result.failure_reason = f"payload_displacement_exceeded ({result.payload_max_displacement:.4f}m)"
    elif result.max_force_asymmetry > 1.0:
        result.failure_reason = f"force_asymmetry_exceeded ({result.max_force_asymmetry:.3f}N)"
    elif len(result.unexpected_contacts) > 0:
        result.failure_reason = f"unexpected_contacts ({result.unexpected_contacts})"
    elif result.left_steady_force < PHYSICS_MIN_FORCE or \
            result.right_steady_force < PHYSICS_MIN_FORCE:
        result.failure_reason = (
            f"steady_state_force_below_physics_minimum "
            f"(L={result.left_steady_force:.3f}N R={result.right_steady_force:.3f}N, "
            f"required={PHYSICS_MIN_FORCE:.3f}N)"
        )
    else:
        result.success = True

    return result
