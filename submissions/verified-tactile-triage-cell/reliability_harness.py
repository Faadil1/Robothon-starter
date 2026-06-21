"""
reliability_harness.py — consolidated production validation.

Executes every mandatory gate against PRODUCTION scene.xml (never a
solver_experiment variant) and aggregates results into a single
reliability_report.json. Exits with a clear pass/fail summary; does not
weaken any threshold or hide any failing trial.
"""
import os
import sys
import json
import uuid
import datetime

SCENE_PATH = "scene.xml"


def assert_production_scene():
    """Hard guard: refuse to run against anything other than the real
    production scene.xml in this directory."""
    if not os.path.exists(SCENE_PATH):
        raise FileNotFoundError(f"Production scene not found at {SCENE_PATH}")
    abs_path = os.path.abspath(SCENE_PATH)
    if "solver_experiment" in abs_path:
        raise RuntimeError("Refusing to run: scene path resolves into solver_experiment/, not production.")
    return abs_path


def run_phase1():
    import mujoco
    import numpy as np
    m = mujoco.MjModel.from_xml_path(SCENE_PATH)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    for _ in range(2000):
        mujoco.mj_step(m, d)
    payload_z = float(d.xpos[m.body("payload").id][2])
    nan_detected = bool(np.any(np.isnan(d.qpos)))
    passed = (not nan_detected) and abs(payload_z - 0.0178) < 0.005
    return {
        "phase": 1, "name": "scene_load_and_stability",
        "passed": passed, "payload_z": payload_z, "nan_detected": nan_detected,
    }


def run_phase2():
    import subprocess
    r1 = subprocess.run([sys.executable, "test_phase2_actuators.py"], capture_output=True, text=True)
    r2 = subprocess.run([sys.executable, "test_phase2_combined.py"], capture_output=True, text=True)
    p1 = "ALL INDIVIDUAL ACTUATOR TESTS: PASS" in r1.stdout
    p2 = "COMBINED SEQUENCE TEST: PASS" in r2.stdout
    return {
        "phase": 2, "name": "actuator_validation",
        "passed": p1 and p2,
        "individual_actuators_passed": p1, "combined_sequence_passed": p2,
        "stdout_individual": r1.stdout[-2000:], "stdout_combined": r2.stdout[-2000:],
    }


def run_phase3():
    from tactile_grasp import run_tactile_grasp_trial
    offsets = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0), (0.0, 0.001), (0.0, -0.001)]
    trials = []
    for offset in offsets:
        r = run_tactile_grasp_trial(payload_offset_xy=offset)
        trials.append({"offset": offset, "success": r.success, "failure_reason": r.failure_reason})
    n_pass = sum(1 for t in trials if t["success"])
    return {"phase": 3, "name": "grasp_5_of_5", "passed": n_pass == 5, "n_pass": n_pass, "n_total": 5, "trials": trials}


def run_phase4():
    from lift_transport import run_lift_transport_trial
    offsets = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0), (0.0, 0.001), (0.0, -0.001)]
    trials = []
    for offset in offsets:
        r = run_lift_transport_trial(payload_offset_xy=offset)
        trials.append({
            "offset": offset, "success": r.success, "failure_reason": r.failure_reason,
            "max_relative_slip_mm": round(r.max_relative_slip * 1000, 4),
        })
    n_pass = sum(1 for t in trials if t["success"])
    return {"phase": 4, "name": "lift_transport_5_of_5", "passed": n_pass == 5, "n_pass": n_pass, "n_total": 5, "trials": trials}


def run_phase5():
    from slip_recovery import run_slip_trial
    neg_offsets = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0)]
    inj_offsets = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0)]

    neg_trials = []
    for offset in neg_offsets:
        r = run_slip_trial(inject_disturbance=False, payload_offset_xy=offset)
        neg_trials.append({"offset": offset, "false_positive": r.false_positive, "max_slip_mm": round(r.max_wrist_slip * 1000, 4)})
    n_neg_pass = sum(1 for t in neg_trials if not t["false_positive"])

    inj_trials = []
    for offset in inj_offsets:
        r = run_slip_trial(inject_disturbance=True, payload_offset_xy=offset)
        inj_trials.append({
            "offset": offset, "final_state": r.final_state,
            "detection_latency_steps": r.detection_latency_steps,
            "max_slip_mm": round(r.max_wrist_slip * 1000, 4),
        })
    n_inj_pass = sum(1 for t in inj_trials if t["final_state"] == "RECOVERY_SUCCESS")

    passed = (n_neg_pass == 3) and (n_inj_pass >= 3)
    return {
        "phase": 5, "name": "slip_detection_and_recovery", "passed": passed,
        "negative_controls": {"n_pass": n_neg_pass, "n_total": 3, "trials": neg_trials},
        "injected_recovery": {"n_pass": n_inj_pass, "n_total": 3, "trials": inj_trials},
    }


def run_phase6():
    from placement_controller import run_placement_trial, SAFE_ZONE, CONTAMINATED_ZONE, ZONES
    from triage_safety_gate import evaluate_placement

    malformed_cases = [None, ("a", "b"), (float("nan"), 0.0), (0.0, 0.0), (0.1,), (float("inf"), 0.0)]
    malformed_results = [evaluate_placement(t, ZONES)["verdict"] for t in malformed_cases]
    malformed_all_blocked = all(v == "BLOCK" for v in malformed_results)

    offsets = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0), (0.0, 0.001), (0.0, -0.001)]
    a_trials = []
    for offset in offsets:
        r = run_placement_trial(requested_target_xy=SAFE_ZONE.center_xy, payload_offset_xy=offset)
        a_trials.append({"offset": offset, "final_state": r.final_state, "contaminated_entered": r.contaminated_zone_entered})
    n_a_pass = sum(1 for t in a_trials if t["final_state"] == "PLACED_SAFE")

    b_trials = []
    for offset in offsets:
        r = run_placement_trial(requested_target_xy=CONTAMINATED_ZONE.center_xy, payload_offset_xy=offset)
        b_trials.append({
            "offset": offset, "final_state": r.final_state,
            "contaminated_entered": r.contaminated_zone_entered,
            "contaminated_released": r.contaminated_zone_release,
        })
    n_b_pass = sum(1 for t in b_trials if t["final_state"] == "BLOCKED_FALLBACK_SAFE" and not t["contaminated_released"])

    passed = malformed_all_blocked and (n_a_pass == 5) and (n_b_pass == 5)
    return {
        "phase": 6, "name": "triage_decision_and_placement", "passed": passed,
        "malformed_all_blocked": malformed_all_blocked,
        "scenario_a": {"n_pass": n_a_pass, "n_total": 5, "trials": a_trials},
        "scenario_b": {"n_pass": n_b_pass, "n_total": 5, "trials": b_trials},
    }


def run_phase7():
    from placement_controller import run_placement_trial, SAFE_ZONE, CONTAMINATED_ZONE
    from slip_recovery import run_slip_trial
    from triage_receipt_logger import (
        build_receipt_from_placement_result, build_receipt_from_slip_result,
        log_receipt, verify_event,
    )

    receipts_verified = []

    r_a = run_placement_trial(requested_target_xy=SAFE_ZONE.center_xy)
    ev_a = build_receipt_from_placement_result(r_a, str(uuid.uuid4()))
    signed_a = log_receipt(ev_a)
    receipts_verified.append(verify_event(signed_a))

    r_b = run_placement_trial(requested_target_xy=CONTAMINATED_ZONE.center_xy)
    ev_b = build_receipt_from_placement_result(r_b, str(uuid.uuid4()))
    signed_b = log_receipt(ev_b)
    receipts_verified.append(verify_event(signed_b))

    r_neg = run_slip_trial(inject_disturbance=False)
    ev_neg = build_receipt_from_slip_result(r_neg, str(uuid.uuid4()))
    signed_neg = log_receipt(ev_neg)
    receipts_verified.append(verify_event(signed_neg))

    r_inj = run_slip_trial(inject_disturbance=True)
    ev_inj = build_receipt_from_slip_result(r_inj, str(uuid.uuid4()))
    signed_inj = log_receipt(ev_inj)
    receipts_verified.append(verify_event(signed_inj))

    n_verified = sum(receipts_verified)
    n_total = len(receipts_verified)

    # Tamper-detection tests (in-memory, not affecting the log file).
    tamper_tests = []
    for field, new_value in [
        ("planning_verdict", "BLOCK"), ("displacement_slip_detected", True),
        ("min_force_transport_n", [99.9, 99.9]), ("contaminated_zone_release", True),
        ("recovery_outcome", "RECOVERY_FAIL"), ("episode_id", "fake-replay-id"),
    ]:
        for base in (signed_a, signed_inj):
            if field in base:
                tampered = dict(base)
                tampered[field] = new_value
                still_verifies = verify_event(tampered)
                tamper_tests.append({"field": field, "tamper_detected": not still_verifies})
                break

    n_tamper_ok = sum(1 for t in tamper_tests if t["tamper_detected"])

    passed = (n_verified == n_total) and (n_tamper_ok == len(tamper_tests))
    return {
        "phase": 7, "name": "signed_receipts", "passed": passed,
        "receipts_verified": {"n_ok": n_verified, "n_total": n_total},
        "tamper_detection": {"n_ok": n_tamper_ok, "n_total": len(tamper_tests), "tests": tamper_tests},
    }


def main():
    scene_abs_path = assert_production_scene()
    print(f"Production scene confirmed: {scene_abs_path}\n")

    phase_runners = [run_phase1, run_phase2, run_phase3, run_phase4, run_phase5, run_phase6, run_phase7]
    phase_results = []
    for runner in phase_runners:
        result = runner()
        phase_results.append(result)
        status = "PASS" if result["passed"] else "FAIL"
        print(f"[{status}] Phase {result['phase']} — {result['name']}")

    all_passed = all(r["passed"] for r in phase_results)

    report = {
        "harness": "reliability_harness.py",
        "production_scene_path": scene_abs_path,
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "phases": phase_results,
        "all_mandatory_gates_passed": all_passed,
    }
    with open("reliability_report.json", "w") as f:
        json.dump(report, f, indent=2, default=str)

    print()
    print(f"OVERALL: {'PASS' if all_passed else 'FAIL'}")
    print("Report written to reliability_report.json")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
