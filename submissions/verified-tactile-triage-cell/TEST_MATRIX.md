# TEST_MATRIX.md — Verified Tactile Triage Cell

| Phase | Test command | Latest result | Status | Report |
|---|---|---|---|---|
| 1 | `python3 -c "..."` (scene load + 2000-step settle check, inline) | payload settles z≈0.0179, 0 NaN | PASS | none (inline) |
| 2 | `python3 test_phase2_actuators.py` | 5/5 actuators within tolerance (finger targets corrected to ±0.012m, see DECISION_LOG.md) | **PASS (repaired Phase 8)** | none (stdout) |
| 2 | `python3 test_phase2_combined.py` | combined sequence, max error 0.00235, zero unexpected contacts | **PASS (repaired Phase 8)** | none (stdout) |
| 3 | `python3 test_phase3_reliability.py` | 5/5, forces 2.84–2.87N | PASS | `phase3_reliability_report.json` |
| 4 | `python3 test_phase4_lift_transport.py` | 5/5, slip 0.77–0.81mm, force 2.82–2.87N | **PASS (now permanent)** | `phase4_reliability_report.json` |
| 4 (solver A/B/C) | `python3 solver_experiment/run_solver_experiment.py` | B selected: slip 13.65mm→0.67mm | DECISION RECORDED | inline (see DECISION_LOG.md) |
| 5 | `python3 test_phase5_slip_recovery.py` | neg control 3/3, recovery 3/3 | PASS | `phase5_reliability_report.json` |
| 6 | `python3 test_phase6_placement_gate.py` | malformed 6/6, Scenario A 5/5, Scenario B 5/5 | PASS | `phase6_reliability_report.json` |
| 7 | inline (build+sign receipts for 1 Scenario A, 1 Scenario B, 1 slip-recovery, 1 negative-control) + `python3 triage_receipt_logger.py verify logs/triage_receipts.jsonl` | 4/4 verified; 6/6 tamper tests correctly fail verification | PASS | `logs/triage_receipts.jsonl` |
| 8 (harness) | `python3 run.py` | All 7 phases PASS, exit 0 | PASS | `reliability_report.json` |
| 8 (osmesa, this sandbox) | `MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python3 run.py` | **FAIL — exit 1**, `libosmesa6` not installed in this sandbox (confirmed via `ldconfig -p`); import-time crash in mujoco's GL context loader, not a harness defect (same harness exits 0 without forced osmesa) | **BLOCKED — sandbox library gap, must re-verify in Cloud Shell** | n/a (crash before report write) |

## Phase 2 test repair (Phase 8)
Previously flagged as stale: `test_phase2_actuators.py` and `test_phase2_combined.py` used finger-closure target values (±0.03 to ±0.04) that, at geometry from Phase 4/6 onward, actually contacted the payload. Measured the real no-contact margin (0.020m) and retargeted both tests to ±0.012m — verified zero finger-payload contact, zero instability, zero payload displacement. Production `scene.xml` was not modified. Full root-cause detail preserved in DECISION_LOG.md.

## Regression requirement
Every phase change must re-run **all prior phase tests** before being accepted. This was followed throughout — see DECISION_LOG.md for specific regression checkpoints.

## To reproduce from a clean checkout
```bash
cd /home/claude/triage_cell
python3 -c "import mujoco; m=mujoco.MjModel.from_xml_path('scene.xml'); print('scene OK')"
python3 test_phase3_reliability.py
python3 test_phase4_lift_transport.py
python3 test_phase5_slip_recovery.py
python3 test_phase6_placement_gate.py
```
