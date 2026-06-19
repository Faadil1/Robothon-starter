"""
record_demo.py — produce a demo video by running the actual submitted code.

This does NOT stage or fake anything: it calls the same run_episode()
function used by run.py, with a frame_collector attached to capture frames
during real simulation steps. Frames are written to a temp directory and
encoded into demo.mp4 via ffmpeg.

Usage:
    python record_demo.py
"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import shutil
import subprocess
import mujoco

from episode_runner import run_episode

FRAME_DIR = "frames"
OUTPUT_PATH = "demo.mp4"
FPS = 30
RENDER_EVERY_N_STEPS = 3  # capture every 3rd physics step to keep frame count sane


def make_frame_collector(renderer, frame_counter, camera="angled"):
    state = {"step": 0}

    def collector(model, data, _renderer):
        state["step"] += 1
        if state["step"] % RENDER_EVERY_N_STEPS != 0:
            return
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        idx = frame_counter["n"]
        frame_counter["n"] += 1
        from PIL import Image
        Image.fromarray(pixels).save(os.path.join(FRAME_DIR, f"frame_{idx:05d}.png"))

    return collector


def main():
    if os.path.exists(FRAME_DIR):
        shutil.rmtree(FRAME_DIR)
    os.makedirs(FRAME_DIR, exist_ok=True)

    model = mujoco.MjModel.from_xml_path("scene.xml")
    renderer = mujoco.Renderer(model, height=480, width=640)
    frame_counter = {"n": 0}
    collector = make_frame_collector(renderer, frame_counter)

    print("[record_demo] Running Scenario A (ALLOW) with frame capture...")
    event_a, _, _ = run_episode("A", target_xy=(0.55, 0.45), frame_collector=collector)
    print(f"  verdict: {event_a['verdict']}, frames so far: {frame_counter['n']}")

    print("[record_demo] Running Scenario B (BLOCK) with frame capture...")
    event_b, _, _ = run_episode("B", target_xy=(0.4, -0.25), frame_collector=collector)
    print(f"  verdict: {event_b['verdict']}, frames so far: {frame_counter['n']}")

    renderer.close()

    n_frames = frame_counter["n"]
    if n_frames == 0:
        print("[record_demo] ERROR: no frames captured")
        return 1

    print(f"[record_demo] Encoding {n_frames} frames to {OUTPUT_PATH}...")
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
