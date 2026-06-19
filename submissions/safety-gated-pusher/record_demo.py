"""
record_demo.py — produce a demo video by running the actual submitted code.

This does NOT stage or fake anything: it calls the same run_episode()
function used by run.py (unmodified), with a frame_collector attached to
capture frames during real simulation steps. Title cards, hold-frames, and
text overlays are added around the genuine simulation footage to make the
task narrative legible — no simulation frame's pixel content is altered,
and no scenario's outcome is staged or pre-determined.

V2 changes vs V1:
  - Extended runtime (~60-90s vs ~10.7s) via intro/title cards, longer holds
    on start/end states, and an explicit hazard-zone establishing shot.
  - Text overlay cards narrate each beat (intro, scene, hazard/goal zones,
    Scenario A, receipt verification, Scenario B, final blocked state,
    summary) without altering any underlying physics or control code.
  - Receipt verification result is rendered as an on-screen text card,
    using the real signed receipts.jsonl entries produced by run_episode().

Usage:
    python record_demo.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import shutil
import subprocess
import mujoco
from PIL import Image, ImageDraw, ImageFont

from episode_runner import run_episode
from receipt_logger import log_episode, log_receipt, verify_event

FRAME_DIR = "frames"
OUTPUT_PATH = "demo.mp4"
FPS = 30
RENDER_EVERY_N_STEPS = 3  # capture every 3rd physics step during live sim

WIDTH, HEIGHT = 640, 480

# Hold durations (seconds) for static/title frames -- tunable without
# touching any simulation logic.
HOLD = {
    "intro": 4.5,
    "scene_establish": 4.5,
    "hazard_establish": 4.5,
    "goal_establish": 4.0,
    "scenario_a_label": 3.0,
    "scenario_a_start_hold": 2.5,
    "scenario_a_end_hold": 4.5,
    "receipt_a_card": 4.5,
    "scenario_b_label": 3.0,
    "scenario_b_start_hold": 2.5,
    "scenario_b_end_hold": 4.5,
    "receipt_b_card": 4.5,
    "summary_card": 5.5,
}


def _font(size=28):
    try:
        return ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size
        )
    except Exception:
        return ImageFont.load_default()


def make_title_card(lines, subtitle=None, bg=(12, 14, 20), max_title_size=32, min_title_size=18):
    """Render a simple text title card as a PIL Image (no simulation
    content involved -- purely a narration frame). Font size auto-shrinks
    so the longest line always fits within the frame width with margin,
    preventing text overflow/cutoff regardless of how long a line is."""
    img = Image.new("RGB", (WIDTH, HEIGHT), bg)
    draw = ImageDraw.Draw(img)
    max_text_width = WIDTH - 60  # leave margin on both sides

    title_size = max_title_size
    title_font = _font(title_size)
    while title_size > min_title_size:
        widest = max((draw.textlength(line, font=title_font) for line in lines), default=0)
        if widest <= max_text_width:
            break
        title_size -= 2
        title_font = _font(title_size)

    sub_font = _font(max(16, title_size - 10))
    if subtitle is not None:
        while title_size > min_title_size and draw.textlength(subtitle, font=sub_font) > max_text_width:
            title_size -= 2
            sub_font = _font(max(16, title_size - 10))

    line_height = title_size + 10
    total_h = len(lines) * line_height + (30 if subtitle else 0)
    y = (HEIGHT - total_h) // 2
    for line in lines:
        w = draw.textlength(line, font=title_font)
        draw.text(((WIDTH - w) / 2, y), line, font=title_font, fill=(230, 230, 230))
        y += line_height

    if subtitle:
        w = draw.textlength(subtitle, font=sub_font)
        draw.text(((WIDTH - w) / 2, y + 10), subtitle, font=sub_font, fill=(150, 200, 150))

    return img


class FrameWriter:
    def __init__(self):
        self.n = 0

    def write_image(self, img: Image.Image, repeat=1):
        for _ in range(repeat):
            img.save(os.path.join(FRAME_DIR, f"frame_{self.n:05d}.png"))
            self.n += 1

    def hold_seconds(self, img: Image.Image, seconds: float):
        n_frames = max(1, int(round(seconds * FPS)))
        self.write_image(img, repeat=n_frames)


def make_sim_frame_collector(renderer, writer: FrameWriter, camera="angled"):
    """Same approach as V1: captures every Nth real physics-step frame.
    No change to simulation stepping, control, or gate logic."""
    state = {"step": 0}

    def collector(model, data, _renderer):
        state["step"] += 1
        if state["step"] % RENDER_EVERY_N_STEPS != 0:
            return
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        Image.fromarray(pixels).save(os.path.join(FRAME_DIR, f"frame_{writer.n:05d}.png"))
        writer.n += 1

    return collector


def render_static_scene(renderer, model, data, camera="angled"):
    """Render one static frame of the current (unstepped) scene state --
    used for establishing shots before any motion occurs."""
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=camera)
    pixels = renderer.render()
    return Image.fromarray(pixels)


def main():
    if os.path.exists(FRAME_DIR):
        shutil.rmtree(FRAME_DIR)
    os.makedirs(FRAME_DIR, exist_ok=True)

    writer = FrameWriter()
    model = mujoco.MjModel.from_xml_path("scene.xml")
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)

    # --- Intro title card ---
    writer.hold_seconds(
        make_title_card(
            ["Safety-Gated Pusher", "Hazard-Aware Rescue Task"],
            subtitle="Robothon 2026 — MuJoCo simulation",
        ),
        HOLD["intro"],
    )

    # --- Scene establishing shot (fresh, unstepped scene) ---
    fresh_data = mujoco.MjData(model)
    scene_frame = render_static_scene(renderer, model, fresh_data)
    writer.hold_seconds(scene_frame, HOLD["scene_establish"])

    # --- Hazard zone establishing card + shot ---
    writer.hold_seconds(
        make_title_card(["Hazard / No-Go Zone"], subtitle="Red zone — unsafe for object transit"),
        2.0,
    )
    writer.hold_seconds(scene_frame, HOLD["hazard_establish"])

    # --- Goal / rescue zone establishing card + shot ---
    writer.hold_seconds(
        make_title_card(["Rescue / Goal Zone"], subtitle="Green zone — safe delivery target"),
        2.0,
    )
    writer.hold_seconds(scene_frame, HOLD["goal_establish"])

    # =========================================================
    # SCENARIO A — ALLOW
    # =========================================================
    writer.hold_seconds(
        make_title_card(["Scenario A", "Planned push: clear of hazard"],
                         subtitle="Expected verdict: ALLOW"),
        HOLD["scenario_a_label"],
    )

    pre_a_data = mujoco.MjData(model)
    pre_a_frame = render_static_scene(renderer, model, pre_a_data)
    writer.hold_seconds(pre_a_frame, HOLD["scenario_a_start_hold"])

    collector_a = make_sim_frame_collector(renderer, writer)
    print("[record_demo] Running Scenario A (ALLOW) with frame capture...")
    event_a, model_a, data_a = run_episode("A", target_xy=(0.55, 0.45), frame_collector=collector_a)
    print(f"  verdict: {event_a['verdict']}, frames so far: {writer.n}")

    final_a_frame = render_static_scene(renderer, model_a, data_a)
    writer.hold_seconds(final_a_frame, HOLD["scenario_a_end_hold"])

    log_episode(event_a)
    signed_a = log_receipt(event_a)
    verified_a = verify_event(signed_a)
    writer.hold_seconds(
        make_title_card(
            [f"Verdict: {event_a['verdict']}", "Receipt signed (Ed25519)"],
            subtitle=f"Independently verified: {'PASS' if verified_a else 'FAIL'}",
        ),
        HOLD["receipt_a_card"],
    )

    # =========================================================
    # SCENARIO B — BLOCK
    # =========================================================
    writer.hold_seconds(
        make_title_card(["Scenario B", "Planned push: crosses hazard zone"],
                         subtitle="Expected verdict: BLOCK"),
        HOLD["scenario_b_label"],
    )

    pre_b_data = mujoco.MjData(model)
    pre_b_frame = render_static_scene(renderer, model, pre_b_data)
    writer.hold_seconds(pre_b_frame, HOLD["scenario_b_start_hold"])

    collector_b = make_sim_frame_collector(renderer, writer)
    print("[record_demo] Running Scenario B (BLOCK) with frame capture...")
    event_b, model_b, data_b = run_episode("B", target_xy=(0.4, -0.25), frame_collector=collector_b)
    print(f"  verdict: {event_b['verdict']}, frames so far: {writer.n}")

    final_b_frame = render_static_scene(renderer, model_b, data_b)
    writer.hold_seconds(final_b_frame, HOLD["scenario_b_end_hold"])

    log_episode(event_b)
    signed_b = log_receipt(event_b)
    verified_b = verify_event(signed_b)
    writer.hold_seconds(
        make_title_card(
            [f"Verdict: {event_b['verdict']}", "Receipt signed (Ed25519)"],
            subtitle=f"Independently verified: {'PASS' if verified_b else 'FAIL'}",
        ),
        HOLD["receipt_b_card"],
    )

    # --- Summary card (exact required wording) ---
    writer.hold_seconds(
        make_title_card(
            ["Safe actions execute.", "Unsafe actions are blocked.",
             "Every decision is signed and verifiable."],
            subtitle="Safety-Gated Pusher — Hazard-Aware Rescue Task",
        ),
        HOLD["summary_card"],
    )

    # --- Optional closing card: Human-AI collaboration credit ---
    writer.hold_seconds(
        make_title_card(
            ["Built with Human-AI collaboration:", "Claude Code implementation, ChatGPT audit,",
             "Cloud Shell reproducibility, human go/no-go decisions."],
        ),
        HOLD["summary_card"],
    )

    renderer.close()

    n_frames = writer.n
    if n_frames == 0:
        print("[record_demo] ERROR: no frames captured")
        return 1

    duration_est = n_frames / FPS
    print(f"[record_demo] Encoding {n_frames} frames (~{duration_est:.1f}s) to {OUTPUT_PATH}...")
    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(FPS),
        "-i", os.path.join(FRAME_DIR, "frame_%05d.png"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p",
        OUTPUT_PATH,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print("[record_demo] ffmpeg failed:")
        print(result.stderr[-2000:])
        return 1

    print(f"[record_demo] Done. {OUTPUT_PATH} written ({n_frames} frames @ {FPS}fps "
          f"= {n_frames/FPS:.1f}s).")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
