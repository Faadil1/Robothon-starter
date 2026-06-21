# DEMO_SCRIPT.md

`demo.mp4` — 79.0s, 640×480, 30fps, h264. Produced by `record_demo.py`
actually executing the production control logic (not staged/edited).
Timestamps below measured directly from the final rendered video's frame
content, not estimated from source code.

**Rendering note:** the final artifact was rendered in Google Cloud Shell
with `MUJOCO_GL=osmesa` and `PYOPENGL_PLATFORM=osmesa`. The render completed
successfully with 2370 frames and exit code 0.

| Time | Content |
|---|---|
| 0:00–0:03 | Title card |
| 0:03–0:06 | Scene establishing shot (wide) |
| 0:06–0:08 | Zone label card (safe/contaminated) |
| 0:08–0:10 | Scene hold (wide) |
| 0:10–0:12 | "Step 1" label |
| 0:12–0:16 | **Tactile grasp + lift (TIGHT camera)** — closed-loop force-driven finger closure, smooth wrist lift |
| 0:16–0:18 | "Transporting toward destination" label |
| 0:18–0:22.7 | **Transport (WIDE camera)** — horizontal move toward the safe zone |
| 0:22.7–0:24.7 | "Step 2" label |
| 0:24.7–0:26.7 | **Controlled disturbance + detection + recovery (TIGHT camera + live HUD)** — gripper and payload remain in frame throughout; HUD shows real L/R fingertip force, controller state, grip target, wrist-relative slip in mm. State visibly transitions HOLD_NORMAL → INCREASE_GRIP_TARGET → WAIT_FOR_STABILIZATION |
| 0:26.7–0:30 | Disturbance/recovery results card (disturbance magnitude, detection step, recovery target, max slip vs. threshold) |
| 0:30–0:33 | "Step 3" label |
| 0:33–0:46 | **Safe placement, Scenario A (WIDE camera)** — descent, contact-confirmed release, retraction |
| 0:46–0:51 | Scenario A results card (planning/pre-release verdicts, release position) |
| 0:51–0:53 | "Step 4" label |
| 0:53–0:70 | **Unsafe destination block, Scenario B (WIDE camera)** — planning gate BLOCK, deterministic fallback to safe zone, pre-release gate ALLOW for fallback |
| 0:70–0:74 | Scenario B results card (BLOCK reason, fallback, contaminated-zone-entered=False) |
| 0:74–0:79 | Final evidence card: live numbers read from `reliability_report.json` (Grasp 5/5, Lift/Transport 5/5, Recovery 3/3, ALLOW 5/5, BLOCK 5/5, Receipts 4/4 verified, Tamper 6/6 detected), with an explicit, non-overclaiming note on which GL backend produced this render |

All on-screen numeric values are taken directly from the running
simulation or `reliability_report.json` at capture time, not pre-written.

## Confirmed during inspection
- Gripper and payload remain visible and in-frame for the entire
  disturbance/detection/recovery segment (tight camera now follows the
  wrist's live position rather than a fixed staging coordinate — this
  was a real bug, found and fixed this session).
- HUD text is fully readable against its background box throughout.
- Evidence-card subtitle no longer clips off-screen (title-card renderer
  now shrinks the subtitle font independently of the main lines).
- Zero black/blank frames found in sampling across the full timeline.

## Final rendering verification
The final `demo.mp4` was regenerated in Google Cloud Shell using OSMesa.

Independent media verification:
- codec: H.264
- resolution: 640×480
- frame rate: 30 fps
- frames: 2370
- duration: 79.0 seconds
- render exit code: 0

The final evidence card visibly confirms `Cloud Shell OSMesa: PASS`.
