# Verified Tactile Triage Cell

A 5-actuator tactile gripper in MuJoCo that uses real fingertip force
feedback to grasp, lift, transport, recover from a controlled disturbance,
and place a payload through a safety-gated triage workflow.

The controller blocks placement in a designated contaminated zone,
falls back deterministically to a safe zone, and emits Ed25519-signed,
independently verifiable receipts.

## What this demonstrates

- **Closed-loop tactile grasping**: finger closure is driven by live
  touch-sensor readings against a physics-derived force target.
- **Measured slip-risk detection and recovery**: a bounded external
  disturbance is applied through `xfrc_applied`; recovery uses live
  force-asymmetry, displacement, and stabilization signals.
- **Mandatory safety gating**: `triage_safety_gate.py` is called at
  planning and pre-release checkpoints. Unknown or malformed destinations
  default to `BLOCK`.
- **Tamper-evident evidence**: placement and recovery events are signed
  with Ed25519 and can be checked using only the public key.

## Run the validation

```bash
pip install -r requirements.txt
python run.py
```

The command exits with code 0 only when all mandatory phases pass and
writes `reliability_report.json`.

## Reproduce the demo

```bash
MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python record_demo.py
```

The final demo was rendered in Google Cloud Shell with OSMesa:

- codec: H.264
- resolution: 640×480
- frame rate: 30 fps
- frames: 2370
- duration: 79.0 seconds
- render exit code: 0

The footage is produced by executing the production control logic, not
by staging or manually animating outcomes. See `DEMO_SCRIPT.md`.

## Validated results

- Grasp: 5/5
- Lift/Transport: 5/5
- Slip negative controls: 3/3 with zero false positives
- Injected recovery: 3/3
- Safe placement (`ALLOW`): 5/5
- Unsafe-destination block (`BLOCK`): 5/5
- Malformed/unknown destinations: 6/6 correctly blocked
- Signed receipts independently verified: 4/4
- Tamper tests correctly rejected: 6/6
- Final consolidated regression: Phases 1–7 PASS, exit code 0

See `reliability_report.json`, `FINAL_REGRESSION.txt`, and
`CLOUD_SHELL_DEMO_FINAL.txt` for the recorded evidence.

## Architecture

| File | Role |
|---|---|
| `scene.xml` | MuJoCo scene with 5 actuators, 2 tactile sensors, payload, and safety zones |
| `tactile_grasp.py` | Physics-derived force target and closed-loop grasp acquisition |
| `lift_transport.py` | Lift and transport control |
| `slip_recovery.py` | Disturbance injection, detection, and recovery state machine |
| `triage_safety_gate.py` | Pure `ALLOW`/`BLOCK` decision function |
| `placement_controller.py` | Placement sequence with planning and pre-release checks |
| `triage_receipt_logger.py` | Ed25519 receipt signing |
| `verify_receipts.py` | Public-key-only independent verification |
| `reliability_harness.py` | Consolidated validation harness |
| `run.py` | One-command entry point |
| `record_demo.py` | Reproducible OSMesa demo capture |
| `DEMO_SCRIPT.md` | Timestamped video breakdown |
| `HUMAN_AI_COLLABORATION.md` | Human/AI role separation and audit history |

Detailed parameters, thresholds, and decisions are documented in
`PROJECT_STATE.md` and `DECISION_LOG.md`.

## Security

The private signing key is intentionally excluded from the project and
final archive. Only `keys/triage_receipt_public.pem` is distributed.

## Status

Private prototype. Not submitted anywhere. Submission remains under
human control and depends on contest eligibility and entry rules.
