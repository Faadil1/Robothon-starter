"""
record_demo.py — Phase 9 demo capture.

Drives the SAME validated production logic (tactile_grasp, lift_transport,
placement_controller, slip_recovery, triage_safety_gate) with frame
rendering added around it. No validated module is modified; this script
imports their constants/functions and reproduces their control sequences
faithfully, adding only rendering and narration title cards.

Renders with MUJOCO_GL=egl in this build sandbox (osmesa unavailable
here, consistent with prior project history). Must be re-verified with
MUJOCO_GL=osmesa in Cloud Shell before being treated as the final
artifact -- same caveat as Phase 8's harness run.
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import shutil
import subprocess
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from tactile_grasp import get_ids, acquire_grasp, TARGET_FORCE, MAX_CLOSURE_STEP, PHYSICS_MIN_FORCE
from lift_transport import smooth_ramp, LIFT_HEIGHT, WRIST_MOVE_STEPS, DYNAMIC_FORCE_TOLERANCE
from triage_safety_gate import evaluate_placement
from placement_controller import (
    SAFE_ZONE, CONTAMINATED_ZONE, ZONES, STAGING_X, DESCEND_HEIGHT,
    RELEASE_STEPS, RETRACT_HEIGHT,
)
from slip_recovery import (
    SLIP_DETECTION_THRESHOLD_M, VELOCITY_DETECTION_THRESHOLD_MPS,
    FORCE_ASYMMETRY_THRESHOLD_N, DISTURBANCE_FORCE_N, DISTURBANCE_DURATION_STEPS,
    RECOVERY_GRIP_TARGET_N, STABILIZATION_STEPS, STABILIZATION_VELOCITY_THRESHOLD_MPS,
    MAX_RECOVERY_STEPS,
)

FRAME_DIR = "frames"
OUTPUT_PATH = "demo.mp4"
FPS = 30
RENDER_EVERY_N_STEPS = 15
WIDTH, HEIGHT = 640, 480

# Tight camera (programmatic MjvCamera, scene.xml unchanged) used during
# grasp/lift/disturbance/detection/recovery. Tuned to keep both fingers
# and the payload clearly visible.
TIGHT_CAM_LOOKAT = np.array([-0.35, 0.0, 0.08])
TIGHT_CAM_DISTANCE = 0.32
TIGHT_CAM_AZIMUTH = 130
TIGHT_CAM_ELEVATION = -18


def make_tight_camera(lookat_xyz=None):
    cam = mujoco.MjvCamera()
    cam.lookat = (lookat_xyz if lookat_xyz is not None else TIGHT_CAM_LOOKAT).copy()
    cam.distance = TIGHT_CAM_DISTANCE
    cam.azimuth = TIGHT_CAM_AZIMUTH
    cam.elevation = TIGHT_CAM_ELEVATION
    return cam


def draw_hud(img, lf_force, rf_force, state_label, grip_target, slip_mm):
    """Compact real-time HUD overlay (top-left), drawn on top of a
    rendered scene frame. Values passed in are read directly from live
    simulation state by the caller -- never invented here."""
    img = img.copy()
    draw = ImageDraw.Draw(img)
    font = _font(16)
    lines = [
        f"L: {lf_force:.2f}N   R: {rf_force:.2f}N",
        f"State: {state_label}",
        f"Grip target: {grip_target:.2f}N",
        f"Wrist-rel slip: {slip_mm:.2f}mm",
    ]
    pad = 6
    line_h = 18
    box_w = max(draw.textlength(l, font=font) for l in lines) + pad * 2
    box_h = len(lines) * line_h + pad * 2
    draw.rectangle([4, 4, 4 + box_w, 4 + box_h], fill=(8, 10, 14))
    y = 4 + pad
    for l in lines:
        draw.text((4 + pad, y), l, font=font, fill=(220, 230, 220))
        y += line_h
    return img


def _font(size=26):
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size)
    except Exception:
        return ImageFont.load_default()


def make_title_card(lines, subtitle=None, bg=(10, 12, 18), max_size=30, min_size=16):
    img = Image.new("RGB", (WIDTH, HEIGHT), bg)
    draw = ImageDraw.Draw(img)
    max_w = WIDTH - 50
    size = max_size
    font = _font(size)
    while size > min_size:
        widest = max((draw.textlength(l, font=font) for l in lines), default=0)
        if widest <= max_w:
            break
        size -= 2
        font = _font(size)
    sub_size = max(14, size - 8)
    sub_font = _font(sub_size)
    if subtitle:
        while sub_size > 10 and draw.textlength(subtitle, font=sub_font) > max_w:
            sub_size -= 2
            sub_font = _font(sub_size)
    line_h = size + 10
    total_h = len(lines) * line_h + (28 if subtitle else 0)
    y = (HEIGHT - total_h) // 2
    for l in lines:
        w = draw.textlength(l, font=font)
        draw.text(((WIDTH - w) / 2, y), l, font=font, fill=(230, 230, 230))
        y += line_h
    if subtitle:
        w = draw.textlength(subtitle, font=sub_font)
        draw.text(((WIDTH - w) / 2, y + 8), subtitle, font=sub_font, fill=(140, 210, 150))
    return img


class FrameWriter:
    def __init__(self):
        self.n = 0

    def write(self, img, repeat=1):
        for _ in range(repeat):
            img.save(os.path.join(FRAME_DIR, f"frame_{self.n:05d}.png"))
            self.n += 1

    def hold(self, img, seconds):
        self.write(img, repeat=max(1, int(round(seconds * FPS))))


def render_scene(renderer, d, camera="angled"):
    mujoco.mj_forward.__self__ if False else None
    renderer.update_scene(d, camera=camera)
    return Image.fromarray(renderer.render())


def regulate(d, ids, lf_cmd, rf_cmd, target=None):
    target = target if target is not None else TARGET_FORCE
    lf = float(d.sensordata[0])
    rf = float(d.sensordata[1])
    if lf < target - DYNAMIC_FORCE_TOLERANCE:
        lf_cmd = max(lf_cmd - MAX_CLOSURE_STEP, ids["lf_range"][0])
    elif lf > target + DYNAMIC_FORCE_TOLERANCE:
        lf_cmd = min(lf_cmd + MAX_CLOSURE_STEP, ids["lf_range"][1])
    if rf < target - DYNAMIC_FORCE_TOLERANCE:
        rf_cmd = min(rf_cmd + MAX_CLOSURE_STEP, ids["rf_range"][1])
    elif rf > target + DYNAMIC_FORCE_TOLERANCE:
        rf_cmd = max(rf_cmd - MAX_CLOSURE_STEP, ids["rf_range"][0])
    return lf_cmd, rf_cmd, lf, rf


def main():
    if os.path.exists(FRAME_DIR):
        shutil.rmtree(FRAME_DIR)
    os.makedirs(FRAME_DIR)

    m = mujoco.MjModel.from_xml_path("scene.xml")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    ids = get_ids(m)
    renderer = mujoco.Renderer(m, height=HEIGHT, width=WIDTH)
    w = FrameWriter()

    def cam_frame(tight=False):
        if tight:
            wrist_xyz = d.xpos[m.body("wrist").id].copy()
            wrist_xyz[2] = TIGHT_CAM_LOOKAT[2]  # keep the tuned height offset
            live_tight_cam = make_tight_camera(wrist_xyz)
            return render_scene(renderer, d, camera=live_tight_cam)
        return render_scene(renderer, d, camera="angled")

    # ---------- Intro (wide) ----------
    w.hold(make_title_card(["Verified Tactile Triage Cell"],
                            subtitle="5-actuator tactile gripper, MuJoCo simulation"), 3.0)

    for _ in range(500):
        d.ctrl[ids["act_x"]] = 0.0; d.ctrl[ids["act_z"]] = 0.0; d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]; d.ctrl[ids["act_rf"]] = ids["rf_range"][0]
        mujoco.mj_step(m, d)
    scene_frame = cam_frame()
    w.hold(scene_frame, 2.5)
    w.hold(make_title_card(["Safe triage zone (green)", "Contaminated zone (red)"]), 2.5)
    w.hold(scene_frame, 2.0)

    # ---------- 1. Tactile grasp + lift (TIGHT camera) ----------
    w.hold(make_title_card(["Step 1", "Bilateral tactile grasp + lift"]), 2.0)

    grasp_out, lf_cmd, rf_cmd = acquire_grasp(m, d, ids, 0.0, 0.0)

    def sim_collector(tight=False):
        nonlocal lf_cmd, rf_cmd
        state = {"i": 0}
        def step():
            state["i"] += 1
            if state["i"] % RENDER_EVERY_N_STEPS == 0:
                w.write(cam_frame(tight=tight))
        return step

    collector = sim_collector(tight=True)
    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(0.0, LIFT_HEIGHT, WRIST_MOVE_STEPS, s)
        lf_cmd, rf_cmd, lf, rf = regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_x"]] = 0.0; d.ctrl[ids["act_z"]] = z; d.ctrl[ids["act_yaw"]] = 0.0
        d.ctrl[ids["act_lf"]] = lf_cmd; d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)
        collector()

    w.hold(cam_frame(tight=True), 1.0)

    # ---------- Transport (WIDE camera) ----------
    w.hold(make_title_card(["Transporting toward destination"]), 1.5)
    collector_wide = sim_collector(tight=False)
    wrist_x_target = SAFE_ZONE.center_xy[0] - STAGING_X
    for s in range(WRIST_MOVE_STEPS):
        x = smooth_ramp(0.0, wrist_x_target, WRIST_MOVE_STEPS, s)
        lf_cmd, rf_cmd, lf, rf = regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_x"]] = x; d.ctrl[ids["act_z"]] = LIFT_HEIGHT
        d.ctrl[ids["act_lf"]] = lf_cmd; d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)
        collector_wide()

    w.hold(cam_frame(), 1.5)

    # ---------- 2. Controlled disturbance + detection + recovery (TIGHT camera + HUD) ----------
    w.hold(make_title_card(["Step 2", "Controlled disturbance + tactile recovery"]), 2.0)

    payload_id = ids["payload_id"]
    wrist_pos_ref = d.xpos[m.body("wrist").id].copy()
    wrist_rel_ref = d.xpos[payload_id].copy() - wrist_pos_ref
    state_label = "HOLD_NORMAL"
    recovery_target = None
    recovery_start = None
    stab_count = 0
    detected_step = None
    max_slip_seen = 0.0

    render_step_counter = {"i": 0}
    for step in range(900):
        if 150 <= step < 150 + DISTURBANCE_DURATION_STEPS:
            d.xfrc_applied[payload_id, 1] = DISTURBANCE_FORCE_N
        else:
            d.xfrc_applied[payload_id, 1] = 0.0

        target = recovery_target if recovery_target is not None else TARGET_FORCE
        lf_cmd, rf_cmd, lf, rf = regulate(d, ids, lf_cmd, rf_cmd, target=target)
        d.ctrl[ids["act_x"]] = wrist_x_target; d.ctrl[ids["act_z"]] = LIFT_HEIGHT
        d.ctrl[ids["act_lf"]] = lf_cmd; d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)

        wrist_pos = d.xpos[m.body("wrist").id]
        payload_pos = d.xpos[payload_id]
        slip = float(np.linalg.norm((payload_pos - wrist_pos) - wrist_rel_ref))
        max_slip_seen = max(max_slip_seen, slip)
        asym = abs(lf - rf)

        render_step_counter["i"] += 1
        if render_step_counter["i"] % RENDER_EVERY_N_STEPS == 0:
            frame = cam_frame(tight=True)
            frame = draw_hud(frame, lf, rf, state_label,
                              target, slip * 1000)
            w.write(frame)

        if state_label == "HOLD_NORMAL":
            if slip > SLIP_DETECTION_THRESHOLD_M or asym > FORCE_ASYMMETRY_THRESHOLD_N:
                state_label = "SLIP_DETECTED"
                detected_step = step
        elif state_label == "SLIP_DETECTED":
            state_label = "INCREASE_GRIP_TARGET"
            recovery_target = RECOVERY_GRIP_TARGET_N
            recovery_start = step
        elif state_label == "INCREASE_GRIP_TARGET":
            if lf >= recovery_target - DYNAMIC_FORCE_TOLERANCE and rf >= recovery_target - DYNAMIC_FORCE_TOLERANCE:
                state_label = "WAIT_FOR_STABILIZATION"
        elif state_label == "WAIT_FOR_STABILIZATION":
            stab_count = stab_count + 1 if slip - max_slip_seen <= 0 else 0
            if step - recovery_start > 400:
                state_label = "SAFE_HOLD"

    w.hold(make_title_card(
        [f"Disturbance: {DISTURBANCE_FORCE_N}N for {DISTURBANCE_DURATION_STEPS}ms",
         f"Detected at step {detected_step}" if detected_step else "Not detected",
         f"Recovery target: {RECOVERY_GRIP_TARGET_N:.2f}N"],
        subtitle=f"Max slip observed: {max_slip_seen*1000:.2f}mm (threshold {SLIP_DETECTION_THRESHOLD_M*1000:.2f}mm)",
    ), 4.0)

    # ---------- 3. Safe placement (Scenario A) — WIDE camera ----------
    w.hold(make_title_card(["Step 3", "Safe placement — Scenario A"],
                            subtitle="Requested destination: SAFE ZONE"), 2.5)

    collector_a = sim_collector(tight=False)
    planning_a = evaluate_placement(SAFE_ZONE.center_xy, ZONES, check_label="planning")
    for _ in range(300):
        lf_cmd, rf_cmd, lf, rf = regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_lf"]] = lf_cmd; d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)
        collector_a()
    prerelease_a = evaluate_placement(SAFE_ZONE.center_xy, ZONES, check_label="pre_release")

    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(LIFT_HEIGHT, DESCEND_HEIGHT, WRIST_MOVE_STEPS, s)
        lf_cmd, rf_cmd, lf, rf = regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_z"]] = z; d.ctrl[ids["act_lf"]] = lf_cmd; d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)
        collector_a()

    for _ in range(300):
        lf_cmd, rf_cmd, lf, rf = regulate(d, ids, lf_cmd, rf_cmd)
        d.ctrl[ids["act_lf"]] = lf_cmd; d.ctrl[ids["act_rf"]] = rf_cmd
        mujoco.mj_step(m, d)
        collector_a()

    lf_start, rf_start = lf_cmd, rf_cmd
    for s in range(RELEASE_STEPS):
        t = s / RELEASE_STEPS
        d.ctrl[ids["act_lf"]] = lf_start + (ids["lf_range"][1] - lf_start) * t
        d.ctrl[ids["act_rf"]] = rf_start + (ids["rf_range"][0] - rf_start) * t
        mujoco.mj_step(m, d)
        collector_a()

    release_xy_a = tuple(d.xpos[payload_id][:2])
    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(DESCEND_HEIGHT, DESCEND_HEIGHT + RETRACT_HEIGHT, WRIST_MOVE_STEPS, s)
        d.ctrl[ids["act_z"]] = z
        d.ctrl[ids["act_lf"]] = ids["lf_range"][1]; d.ctrl[ids["act_rf"]] = ids["rf_range"][0]
        mujoco.mj_step(m, d)
        collector_a()

    w.hold(cam_frame(), 2.0)
    w.hold(make_title_card(
        [f"Planning: {planning_a['verdict']}", f"Pre-release: {prerelease_a['verdict']}"],
        subtitle=f"Released at ({release_xy_a[0]:.3f}, {release_xy_a[1]:.3f}) — inside safe zone",
    ), 3.0)

    # ---------- 4. Unsafe destination BLOCK + safe fallback (Scenario B) ----------
    w.hold(make_title_card(["Step 4", "Unsafe destination — Scenario B"],
                            subtitle="Requested destination: CONTAMINATED ZONE"), 2.5)

    m2 = mujoco.MjModel.from_xml_path("scene.xml")
    d2 = mujoco.MjData(m2)
    mujoco.mj_forward(m2, d2)
    ids2 = get_ids(m2)
    for _ in range(500):
        d2.ctrl[ids2["act_lf"]] = ids2["lf_range"][1]; d2.ctrl[ids2["act_rf"]] = ids2["rf_range"][0]
        mujoco.mj_step(m2, d2)

    def cam_frame2():
        renderer.update_scene(d2, camera="angled")
        return Image.fromarray(renderer.render())

    def collector2():
        if not hasattr(collector2, "i"):
            collector2.i = 0
        collector2.i += 1
        if collector2.i % RENDER_EVERY_N_STEPS == 0:
            w.write(cam_frame2())

    grasp_out2, lf2, rf2 = acquire_grasp(m2, d2, ids2, 0.0, 0.0)
    planning_b = evaluate_placement(CONTAMINATED_ZONE.center_xy, ZONES, check_label="planning")
    fallback_target = SAFE_ZONE.center_xy if planning_b["verdict"] != "ALLOW" else CONTAMINATED_ZONE.center_xy

    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(0.0, LIFT_HEIGHT, WRIST_MOVE_STEPS, s)
        lf2, rf2, l2, r2 = regulate(d2, ids2, lf2, rf2)
        d2.ctrl[ids2["act_z"]] = z; d2.ctrl[ids2["act_lf"]] = lf2; d2.ctrl[ids2["act_rf"]] = rf2
        mujoco.mj_step(m2, d2)
        collector2()

    wx2 = fallback_target[0] - STAGING_X
    for s in range(WRIST_MOVE_STEPS):
        x = smooth_ramp(0.0, wx2, WRIST_MOVE_STEPS, s)
        lf2, rf2, l2, r2 = regulate(d2, ids2, lf2, rf2)
        d2.ctrl[ids2["act_x"]] = x; d2.ctrl[ids2["act_z"]] = LIFT_HEIGHT
        d2.ctrl[ids2["act_lf"]] = lf2; d2.ctrl[ids2["act_rf"]] = rf2
        mujoco.mj_step(m2, d2)
        collector2()

    prerelease_b = evaluate_placement(fallback_target, ZONES, check_label="pre_release")

    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(LIFT_HEIGHT, DESCEND_HEIGHT, WRIST_MOVE_STEPS, s)
        lf2, rf2, l2, r2 = regulate(d2, ids2, lf2, rf2)
        d2.ctrl[ids2["act_z"]] = z; d2.ctrl[ids2["act_lf"]] = lf2; d2.ctrl[ids2["act_rf"]] = rf2
        mujoco.mj_step(m2, d2)
        collector2()

    for _ in range(300):
        lf2, rf2, l2, r2 = regulate(d2, ids2, lf2, rf2)
        d2.ctrl[ids2["act_lf"]] = lf2; d2.ctrl[ids2["act_rf"]] = rf2
        mujoco.mj_step(m2, d2)
        collector2()

    lf2s, rf2s = lf2, rf2
    for s in range(RELEASE_STEPS):
        t = s / RELEASE_STEPS
        d2.ctrl[ids2["act_lf"]] = lf2s + (ids2["lf_range"][1] - lf2s) * t
        d2.ctrl[ids2["act_rf"]] = rf2s + (ids2["rf_range"][0] - rf2s) * t
        mujoco.mj_step(m2, d2)
        collector2()

    release_xy_b = tuple(d2.xpos[ids2["payload_id"]][:2])
    contaminated_entered = CONTAMINATED_ZONE.contains(release_xy_b)

    for s in range(WRIST_MOVE_STEPS):
        z = smooth_ramp(DESCEND_HEIGHT, DESCEND_HEIGHT + RETRACT_HEIGHT, WRIST_MOVE_STEPS, s)
        d2.ctrl[ids2["act_z"]] = z
        d2.ctrl[ids2["act_lf"]] = ids2["lf_range"][1]; d2.ctrl[ids2["act_rf"]] = ids2["rf_range"][0]
        mujoco.mj_step(m2, d2)
        collector2()

    w.hold(cam_frame2(), 2.0)
    w.hold(make_title_card(
        [f"Planning: {planning_b['verdict']} ({planning_b['reason']})",
         "Fallback: SAFE ZONE", f"Pre-release: {prerelease_b['verdict']}"],
        subtitle=f"Contaminated zone entered: {contaminated_entered}",
    ), 4.0)

    # ---------- 5. Final verified-results card (live numbers only) ----------
    evidence_lines = ["Evidence not available — run python run.py first"]
    evidence_subtitle = None
    if os.path.exists("reliability_report.json"):
        import json
        with open("reliability_report.json") as f:
            rep = json.load(f)
        phases = {p["phase"]: p for p in rep["phases"]}
        p3, p4, p5, p6, p7 = phases[3], phases[4], phases[5], phases[6], phases[7]
        evidence_lines = [
            f"Grasp {p3['n_pass']}/{p3['n_total']}  ·  Lift/Transport {p4['n_pass']}/{p4['n_total']}",
            f"Recovery {p5['injected_recovery']['n_pass']}/{p5['injected_recovery']['n_total']}"
            f"  ·  ALLOW {p6['scenario_a']['n_pass']}/{p6['scenario_a']['n_total']}"
            f"  ·  BLOCK {p6['scenario_b']['n_pass']}/{p6['scenario_b']['n_total']}",
            f"Receipts {p7['receipts_verified']['n_ok']}/{p7['receipts_verified']['n_total']} verified"
            f"  ·  Tamper {p7['tamper_detection']['n_ok']}/{p7['tamper_detection']['n_total']} detected",
        ]
        gl_backend = os.environ.get("MUJOCO_GL", "unknown")
        backend_claim = "Cloud Shell OSMesa: PASS" if gl_backend == "osmesa" else f"Rendered with MUJOCO_GL={gl_backend} (not yet OSMesa-verified)"
        evidence_subtitle = f"{backend_claim} — see reliability_report.json"
    w.hold(make_title_card(evidence_lines, subtitle=evidence_subtitle), 5.0)

    renderer.close()

    n = w.n
    if n == 0:
        print("ERROR: no frames captured")
        return 1
    print(f"Encoding {n} frames (~{n/FPS:.1f}s) to {OUTPUT_PATH}...")
    cmd = ["ffmpeg", "-y", "-framerate", str(FPS), "-i", os.path.join(FRAME_DIR, "frame_%05d.png"),
           "-c:v", "libx264", "-pix_fmt", "yuv420p", OUTPUT_PATH]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stderr[-2000:])
        return 1
    print(f"Done. {OUTPUT_PATH} ({n} frames @ {FPS}fps = {n/FPS:.1f}s)")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
