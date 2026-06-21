"""Phase 2b: combined motion sequence test. Open/close fingers + wrist
horizontal/vertical/yaw motion in one sequence, then return to neutral.
No grasp control, no contact force tuning."""
import mujoco
import numpy as np

TOLERANCE = 0.003

m = mujoco.MjModel.from_xml_path("scene.xml")
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
for _ in range(500):
    mujoco.mj_step(m, d)

payload_id = m.body("payload").id
payload_before = d.xpos[payload_id].copy()

act = {
    "x": m.actuator("act_wrist_x").id,
    "z": m.actuator("act_wrist_z").id,
    "yaw": m.actuator("act_wrist_yaw").id,
    "lf": m.actuator("act_left_finger").id,
    "rf": m.actuator("act_right_finger").id,
}
jnt = {
    "x": (m.joint("wrist_x").id, m.jnt_qposadr[m.joint("wrist_x").id]),
    "z": (m.joint("wrist_z").id, m.jnt_qposadr[m.joint("wrist_z").id]),
    "yaw": (m.joint("wrist_yaw").id, m.jnt_qposadr[m.joint("wrist_yaw").id]),
    "lf": (m.joint("left_finger_close").id, m.jnt_qposadr[m.joint("left_finger_close").id]),
    "rf": (m.joint("right_finger_close").id, m.jnt_qposadr[m.joint("right_finger_close").id]),
}

sequence = [
    # Finger targets corrected for Phase 8 (see test_phase2_actuators.py
    # for the full explanation): +-0.04 exceeded the verified 0.020m
    # no-contact margin at current geometry, causing the fingers to drag
    # the payload during the subsequent x/z/yaw motion and triggering a
    # solver instability warning. +-0.012 stays safely inside that margin.
    {"lf": -0.012, "rf": 0.012},
    {"x": 0.15},
    {"z": 0.08},
    {"yaw": 0.2},
    {"yaw": 0.0},
    {"z": 0.0},
    {"x": 0.0},
    {"lf": 0.0, "rf": 0.0},
]

history = {"x": [], "z": [], "yaw": [], "lf": [], "rf": []}
nan_detected = False
unexpected_contacts = set()

ctrl_state = {"x": 0.0, "z": 0.0, "yaw": 0.0, "lf": 0.0, "rf": 0.0}

for stage in sequence:
    ctrl_state.update(stage)
    for _ in range(1200):
        d.ctrl[act["x"]] = ctrl_state["x"]
        d.ctrl[act["z"]] = ctrl_state["z"]
        d.ctrl[act["yaw"]] = ctrl_state["yaw"]
        d.ctrl[act["lf"]] = ctrl_state["lf"]
        d.ctrl[act["rf"]] = ctrl_state["rf"]
        mujoco.mj_step(m, d)

        if np.any(np.isnan(d.qpos)) or np.any(np.isinf(d.qpos)):
            nan_detected = True

        for k, (jid, adr) in jnt.items():
            history[k].append(float(d.qpos[adr]))

        for i in range(d.ncon):
            g1 = m.geom(d.contact[i].geom1).name
            g2 = m.geom(d.contact[i].geom2).name
            pair = tuple(sorted([g1, g2]))
            if pair != ("payload_geom", "table"):
                unexpected_contacts.add(pair)

final_errors = {}
for k, (jid, adr) in jnt.items():
    final_errors[k] = abs(float(d.qpos[adr]) - 0.0)

payload_after = d.xpos[payload_id].copy()
payload_disp = float(np.linalg.norm(payload_after - payload_before))

print("=== Combined sequence test ===")
print("Final neutral-return errors:", {k: round(v, 5) for k, v in final_errors.items()})
print("Max error:", max(final_errors.values()))
print("NaN detected:", nan_detected)
print("Payload displacement:", payload_disp)
print("Unexpected contacts:", unexpected_contacts if unexpected_contacts else "none")
print("Touch L/R at end:", d.sensordata[0], d.sensordata[1])

all_pass = (
    max(final_errors.values()) <= TOLERANCE
    and not nan_detected
    and payload_disp < 0.001
    and len(unexpected_contacts) == 0
)
print()
print("COMBINED SEQUENCE TEST:", "PASS" if all_pass else "FAIL")
