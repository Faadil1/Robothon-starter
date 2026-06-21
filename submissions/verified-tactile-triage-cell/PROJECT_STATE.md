# PROJECT_STATE.md — Verified Tactile Triage Cell (Private Prototype)

Status: Phase 6 PASS. NOT submitted. NOT connected to PR #146.

## File inventory (production, in /home/claude/triage_cell/)
- `scene.xml` — PRODUCTION MuJoCo scene. Single source of truth for all geometry/physics.
- `tactile_grasp.py` — grasp acquisition, force constants, `acquire_grasp()`.
- `lift_transport.py` — lift/transport control, `regulate_and_step()`.
- `slip_recovery.py` — Phase 5 disturbance injection + recovery state machine.
- `triage_safety_gate.py` — pure ALLOW/BLOCK decision function (no MuJoCo dependency).
- `placement_controller.py` — full Scenario A/B placement sequence, calls the gate twice (mandatory).
- `test_phase2_actuators.py`, `test_phase2_combined.py` — Phase 2 tests. **STALE as of this session**: finger-closure targets now contact the payload at current production geometry; confirmed not a production defect (Phase 3/4/5/6 pass cleanly).
- `test_phase3_reliability.py` — Phase 3 grasp 5/5 gate.
- `test_phase4_lift_transport.py` — Phase 4 lift/transport 5/5 gate (new this session, previously undocumented as a file).
- `test_phase5_slip_recovery.py` — Phase 5 gate.
- `test_phase6_placement_gate.py` — Phase 6 gate.
- `phase3/4/5/6_reliability_report.json` — generated reports.
- `solver_experiment/` — NOT PRODUCTION. Contains scene_variant_A/B/C.xml from the Phase 4 solver A/B/C test. Variant B's settings were copied into production `scene.xml`; this folder is reference-only.

**Resolved this session:** `test_phase4_lift_transport.py` created and confirmed 5/5 against production `scene.xml`.

## Production MuJoCo settings (scene.xml `<option>`)
```
timestep=0.001, integrator=implicitfast, cone=elliptic, solver=Newton,
tolerance=1e-10, impratio=10, noslip_iterations=0
```
Selected via controlled A/B/C experiment (see DECISION_LOG.md). Do not revert to pyramidal cone/default tolerance — this caused 13.65mm slip (solver artifact, not physical friction deficit).

## Mechanical/control parameters
- Finger joints: `left_finger_close` range [-0.058, 0.0], `right_finger_close` range [0.0, 0.058] (slide, axis y)
- Wrist joints: `wrist_x` range [-0.5, 0.6] (anchor x=-0.35 → world reach **[-0.85, 0.25]**), `wrist_z` range [-0.05, 0.4], `wrist_yaw` range [-0.6, 0.6] rad (compiler angle=radian)
- Actuators (position, kp): wrist_x=300, wrist_z=2500, wrist_yaw=2000, left/right finger=80
- Sensors: `left_fingertip_touch`, `right_fingertip_touch` (touch, site size 0.016×0.016×0.052 — intentionally larger than the 0.012×0.012×0.05 finger geom to avoid the site-boundary contact-miss bug found in Phase 1)
- Payload mass: 0.05 kg; finger/payload friction: 0.9 (all geoms, symmetric)
- `TARGET_FORCE = 2.8N`, `FORCE_TOLERANCE = 0.4N` (stationary), `DYNAMIC_FORCE_TOLERANCE = 0.15N` (lift/transport/recovery)
- `PHYSICS_MIN_FORCE = 0.545N` (derived: m·g/(2μ)×2.0 safety factor)
- Achievable force ceiling at full closure: ~3.0N — do not set targets above ~2.9N without re-validating

## Phase 5 thresholds
- Baseline slip noise (measured, post-solver-fix): 0.770–0.810mm
- `SLIP_DETECTION_THRESHOLD_M = 2.43mm` (3× baseline max)
- `VELOCITY_DETECTION_THRESHOLD_MPS = 0.01`
- `FORCE_ASYMMETRY_THRESHOLD_N = 0.05`
- Disturbance: 0.5N for 50 steps via `xfrc_applied` (never qpos/qvel)
- Recovery grip target: 2.95N (capped below the 3.0N ceiling)

## Zone coordinates (Phase 6, production)
- `SAFE_ZONE`: center (0.1, 0.0), half-extent (0.06, 0.06)
- `CONTAMINATED_ZONE`: center (-0.54, 0.0), half-extent (0.06, 0.06)
- Both reachable (within wrist's [-0.85, 0.25] world-x range and table's [-0.6, 0.6]), 0.52m separation, non-colliding (contype/conaffinity=0)

## Phase 1–6 measured results
See TEST_MATRIX.md for the authoritative, current table.

## Known limitations
- No y-axis wrist actuator — all zones/targets must lie on y≈0.
- Phase 4 has no standalone test file (see Gap above).
- `RECOVERY_GRIP_TARGET_N` (2.95N) is close to the ~3.0N ceiling; any future TARGET_FORCE increase must re-check this margin.
- Receipt/signing layer (Phase 7) not yet implemented — no cryptographic evidence exists yet for any phase.
