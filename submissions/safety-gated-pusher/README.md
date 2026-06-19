# Safety-Gated Pusher — Robothon 2026

A MuJoCo simulation where a deterministic pusher controller plans a push
toward a target. A safety gate checks the planned path against a marked
no-go zone *before* any motion is executed:

- **ALLOW** — path is clear → the push executes, object moves to target.
- **BLOCK** — path crosses the no-go zone → the push never executes, object
  stays exactly where it started.

Every verdict (ALLOW/BLOCK) is signed with Ed25519 and logged, as a thin
evidence layer over the simulated decision — not the centerpiece of the
project. The simulation is the deliverable; the receipt is supporting
evidence that the verdict was real and unaltered.

## What this is NOT

- Not a dashboard or UI project. No web frontend exists.
- Not reinforcement learning. The controller is fully deterministic.
- Not multi-object. One pusher, one object, one goal zone, one no-go zone.

## Requirements

- Python 3.12 (tested)
- `pip install mujoco numpy cryptography pillow` (`pillow` only needed for
  `record_demo.py`; `run.py` itself has no rendering dependency)
- `ffmpeg` on PATH (only needed for `record_demo.py`)

## Run instructions

```bash
# Clean run — runs both scenarios, writes logs/episodes.jsonl and
# logs/receipts.jsonl, verifies all receipts, exits 0 on success.
rm -rf logs keys
python run.py
```

Expected output: Scenario A reports verdict `ALLOW` with the object ending
inside the goal zone; Scenario B reports verdict `BLOCK` with the object's
end position identical to its start position. Final line: `RESULT: PASS`.

### Verify receipts independently

```bash
python receipt_logger.py verify logs/receipts.jsonl
```

Exits 0 if all receipts verify, non-zero otherwise. Tampering with any
signed field (e.g. editing `verdict` in the log file by hand) will cause
verification to fail for that line — this was tested during development.

### Reproduce the demo video

```bash
python record_demo.py
```

This re-runs both scenarios through the same `run_episode()` function used
by `run.py`, with a frame collector attached, and encodes the captured
frames into `demo.mp4` via ffmpeg. Requires `MUJOCO_GL=egl` for headless
rendering (set automatically by the script; override if your environment
needs a different backend, e.g. `MUJOCO_GL=glfw` for an on-screen window).

## Architecture

| File | Responsibility |
|---|---|
| `scene.xml` | MuJoCo MJCF: table, pusher, object, goal zone, no-go zone |
| `controller.py` | Deterministic straight-line push planner (no RL) |
| `safety_gate.py` | Pure geometric check: does the planned path cross the no-go zone? |
| `episode_runner.py` | Orchestrates one episode: plan → gate → execute or halt |
| `receipt_logger.py` | Ed25519 signing/verification of episode verdicts |
| `run.py` | CLI entrypoint: runs both scenarios, asserts correctness, exits 0/1 |
| `record_demo.py` | Captures real frames during execution, encodes `demo.mp4` |

## Safety rule

The no-go zone is a fixed rectangular region on the table (rendered in red).
The safety gate checks every waypoint of the planned push path against this
region using simple point-in-rectangle geometry. If any waypoint falls
inside the no-go zone, the verdict is `BLOCK` and the push is never
executed — no pusher motion occurs, no force is applied to the object.

This rule was chosen because it is purely geometric (no force/velocity
threshold tuning required), deterministic, and visually obvious in the
recorded demo: viewers can see the red zone and see that Scenario B's
planned path runs through it.

## Demo capture checklist

- [ ] Run `rm -rf logs keys frames demo.mp4` for a clean slate before
      recording, so the receipt log starts empty.
- [ ] Run `python record_demo.py` and confirm it exits 0.
- [ ] Confirm `demo.mp4` exists and plays both scenarios in sequence.
- [ ] Confirm visually: object moves into the green zone in Scenario A;
      object does not move at all in Scenario B.
- [ ] Do not edit/cut the resulting video — it should be presented as
      produced, consistent with the "video must be produced by running
      the submitted code" requirement.
- [ ] Optionally narrate over the video afterward (audio only), but do not
      alter the visual frames.

## Known limitations / honest caveats

- The `uuid` in `registration.json` was generated locally for this build;
  it has not been verified against whatever registration process the
  actual Robothon-starter repository expects. Confirm before submitting.
- Joint damping/actuator gains in `scene.xml` were tuned empirically to
  make a single push reliably reach the goal zone in this exact scene
  geometry — they are not general-purpose values for arbitrary pusher
  tasks.
- `run.py` appends to existing log files rather than overwriting; run
  `rm -rf logs keys` between clean test runs to avoid accumulating entries
  across runs.
