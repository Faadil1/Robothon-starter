"""Phase 2: independent actuator validation. No grasp control, no contact tuning."""
import mujoco
import numpy as np

TOLERANCE = 0.003  # 3mm / 3mrad tolerance for position-tracking accuracy

m = mujoco.MjModel.from_xml_path("scene.xml")


def fresh_settled_data():
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(500):
        mujoco.mj_step(m, d)
    return d


def joint_qpos(d, joint_name):
    jid = m.joint(joint_name).id
    adr = m.jnt_qposadr[jid]
    return d.qpos[adr]


def joint_range(joint_name):
    jid = m.joint(joint_name).id
    return m.jnt_range[jid]


def check_nan(d):
    return bool(np.any(np.isnan(d.qpos)) or np.any(np.isnan(d.qvel)) or np.any(np.isinf(d.qpos)))


def check_oscillation(history, window=200, threshold=0.0005):
    """Check if the tail of a position history is still oscillating
    (stddev over the last `window` samples above threshold)."""
    if len(history) < window:
        return False
    tail = np.array(history[-window:])
    return float(np.std(tail)) > threshold


def list_contacts(d):
    return [(m.geom(d.contact[i].geom1).name, m.geom(d.contact[i].geom2).name)
            for i in range(d.ncon)]


def run_single_actuator_test(act_name, joint_name, target_offset, n_steps=1500):
    d = fresh_settled_data()
    payload_id = m.body("payload").id
    payload_before = d.xpos[payload_id].copy()

    act_id = m.actuator(act_name).id
    history = []
    for _ in range(n_steps):
        d.ctrl[act_id] = target_offset
        mujoco.mj_step(m, d)
        history.append(float(joint_qpos(d, joint_name)))

    measured = joint_qpos(d, joint_name)
    error = abs(measured - target_offset)
    jrange = joint_range(joint_name)
    within_range = jrange[0] - 1e-6 <= measured <= jrange[1] + 1e-6
    oscillating = check_oscillation(history)
    nan_detected = check_nan(d)
    payload_after = d.xpos[payload_id].copy()
    payload_disp = float(np.linalg.norm(payload_after - payload_before))
    contacts = list_contacts(d)

    # Return to neutral
    for _ in range(n_steps):
        d.ctrl[act_id] = 0.0
        mujoco.mj_step(m, d)
    neutral_error = abs(joint_qpos(d, joint_name) - 0.0)

    return {
        "actuator": act_name,
        "target": target_offset,
        "measured": float(measured),
        "error": float(error),
        "within_range": within_range,
        "oscillating": oscillating,
        "nan_detected": nan_detected,
        "payload_displacement": payload_disp,
        "contacts": contacts,
        "neutral_return_error": float(neutral_error),
        "touch_L": float(d.sensordata[0]),
        "touch_R": float(d.sensordata[1]),
    }


if __name__ == "__main__":
    tests = [
        ("act_wrist_x", "wrist_x", 0.3),
        ("act_wrist_z", "wrist_z", 0.1),
        ("act_wrist_yaw", "wrist_yaw", 0.3),
        # Finger targets corrected for Phase 8: the original +-0.03 targets
        # were set before later geometry refinements (Phase 4/6) narrowed
        # the open-finger-to-payload gap to 0.020m -- at +-0.03 the fingers
        # now contact the payload, which this test was never designed to
        # handle (it asserts zero payload displacement). +-0.012 stays
        # safely inside the verified 0.020m no-contact margin while still
        # exercising real, substantial actuator travel (0.012m out of the
        # 0.058m full range) to genuinely validate the actuator.
        ("act_left_finger", "left_finger_close", -0.012),
        ("act_right_finger", "right_finger_close", 0.012),
    ]

    all_pass = True
    for act_name, joint_name, target in tests:
        r = run_single_actuator_test(act_name, joint_name, target)
        status = "PASS" if (r["error"] <= TOLERANCE and not r["nan_detected"]
                             and r["within_range"] and not r["oscillating"]
                             and r["payload_displacement"] < 0.001
                             and r["neutral_return_error"] <= TOLERANCE) else "FAIL"
        if status == "FAIL":
            all_pass = False
        print(f"[{status}] {act_name}: target={target}, measured={r['measured']:.5f}, "
              f"error={r['error']:.5f}, within_range={r['within_range']}, "
              f"oscillating={r['oscillating']}, nan={r['nan_detected']}, "
              f"payload_disp={r['payload_displacement']:.6f}, "
              f"neutral_return_error={r['neutral_return_error']:.5f}, "
              f"touch_L={r['touch_L']:.4f}, touch_R={r['touch_R']:.4f}, "
              f"contacts={r['contacts']}")

    print()
    print("ALL INDIVIDUAL ACTUATOR TESTS:", "PASS" if all_pass else "FAIL")
