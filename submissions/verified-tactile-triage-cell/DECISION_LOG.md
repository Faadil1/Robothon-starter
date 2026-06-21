# DECISION_LOG.md

## Solver A/B/C decision (Phase 4)
**Issue:** measured wrist-relative slip of 13.65mm during airborne hold, far above the 5mm gate.
**Decision:** ran a controlled A (baseline pyramidal/default) vs B (elliptic/Newton/1e-10/impratio10) vs C (B+noslip) experiment. B reduced slip to 0.67mm with no side effects; C achieved 0.35mm but introduced a new 0.036N force asymmetry. Per the rule "do not choose Noslip if B already passes," **B was selected**. Applied to production `scene.xml`. Phases 1 and 3 re-confirmed clean after the change.

## Phase 5 terminology/arithmetic correction
**Issue:** prior report claimed 2.91mm slip was "3.4× the 2.43mm threshold" — arithmetic error (correct ratio: 1.20×). Prior report also implied zero-latency *displacement*-based slip detection.
**Correction:** traced data showed the force-asymmetry channel fired at latency 0 (real, physically caused by the disturbance), while displacement at that same step was only 0.018mm — far below the 2.43mm displacement threshold. Two distinct signals (`disturbance_or_slip_risk_detected` vs `displacement_slip_detected`) were explicitly separated going forward; no further claims conflate them.

## Phase 6 reachability decision
**Issue:** originally-specified zone coordinates (y=±0.3, then x=0.4/x=0.15) were geometrically unreachable — the wrist has no y-actuator and `wrist_x`'s true world reach is only [-0.85, 0.25] (anchor -0.35 + range [-0.5,0.6]), not the full table extent.
**Decision:** repositioned both zones to verified-reachable x-axis coordinates (safe: x=0.1, contaminated: x=-0.54), placed on opposite sides of the staging position so normal deliveries never cross the contaminated zone. Verified against both table bounds and wrist reach before re-testing.

## Phase 6 safety-gate design decision
**Decision:** the gate (`evaluate_placement`) was built as a pure function with zero MuJoCo/controller dependency, specifically so the controller has exactly one code path to release authorization — it cannot construct an alternate, weaker check. Default-deny applied to all malformed/ambiguous/unknown inputs (verified via 6 unit tests, all BLOCK).

## Bug found and fixed: variable-naming collision (Phase 6)
**Issue:** `min_force_transport` reported negative values (e.g. -0.0564N), which is physically impossible for a touch sensor.
**Root cause:** code was taking `min()` over the regulator's *commanded joint position* (`lf`/`rf`, which can be legitimately negative for the left finger's closing range) instead of the actual sensor force reading. Fixed by reading `d.sensordata[0]`/`[1]` directly for this metric. Re-verified positive, consistent values after the fix.

## Standing human decisions carried forward
- BUILD PRIVATE ONLY (never push to PR #146 without explicit instruction).
- 5–6hr hard checkpoint for go/no-go on each major phase — all phases cleared this with substantial margin (Phase 6 ended at ~115 minutes total).
- No hidden or excluded failed trials at any gate, ever.

## Phase 8: Phase 2 test debt repair
**Issue:** `test_phase2_actuators.py` and `test_phase2_combined.py` used finger-closure targets (±0.03, ±0.04) that were valid when first written but became stale after later geometry refinements (Phase 4/6) narrowed the open-finger-to-payload gap. Measured precisely: the actual no-contact margin at current production geometry is **0.020m**. The old ±0.03/±0.04 targets exceeded this margin, causing real finger-payload contact, payload dragging, and (in the combined test) a genuine MuJoCo solver instability warning (QACC).
**Decision:** retargeted both tests to ±0.012m — verified to stay inside the 0.020m no-contact margin (confirmed via direct contact-list inspection: only `table↔payload_geom` contacts remain, zero finger-payload contacts) while still exercising substantial real actuator travel (0.012m of the joints' 0.058m total range), so the tests remain a genuine validation of actuator function, not a token pass. **Production `scene.xml` geometry was not modified to make these tests pass** — only the test target values changed. Both tests now PASS cleanly (zero payload displacement, zero instability, zero unexpected contacts). The original failure mode (stale targets, instability warning, 8cm payload displacement) remains documented here and in the session transcript as historical evidence; it was not deleted, only superseded.
