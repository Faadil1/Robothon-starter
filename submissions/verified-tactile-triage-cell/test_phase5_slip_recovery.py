"""Phase 5 reliability harness: negative controls + injected slip/recovery
trials. Logs every trial including any failures; no trials excluded."""
import json
from slip_recovery import (
    run_slip_trial, SLIP_DETECTION_THRESHOLD_M, VELOCITY_DETECTION_THRESHOLD_MPS,
    FORCE_ASYMMETRY_THRESHOLD_N, DISTURBANCE_FORCE_N, DISTURBANCE_DURATION_STEPS,
    BASELINE_SLIP_MAX_M, RECOVERY_GRIP_TARGET_N,
)

NEGATIVE_CONTROL_OFFSETS = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0)]
INJECTED_TRIAL_OFFSETS = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0)]


def summarize(label, r, expect_disturbance):
    return {
        "trial": label,
        "expect_disturbance": expect_disturbance,
        "final_state": r.final_state,
        "disturbance_applied": r.disturbance_applied,
        "slip_detected": r.slip_detected,
        "detection_step": r.detection_step,
        "detection_latency_steps": r.detection_latency_steps,
        "slip_at_detection_mm": round(r.slip_at_detection * 1000, 4) if r.slip_at_detection is not None else None,
        "velocity_at_detection_mps": round(r.velocity_at_detection, 6) if r.velocity_at_detection is not None else None,
        "force_asymmetry_at_detection_n": round(r.force_asymmetry_at_detection, 4) if r.force_asymmetry_at_detection is not None else None,
        "max_wrist_slip_mm": round(r.max_wrist_slip * 1000, 4),
        "force_before_n": r.force_before,
        "force_during_n": r.force_during,
        "force_after_n": r.force_after,
        "recovery_grip_target_n": r.recovery_grip_target,
        "recovery_duration_steps": r.recovery_duration_steps,
        "payload_dropped": r.payload_dropped,
        "nan_detected": r.nan_detected,
        "unexpected_contacts": list(r.unexpected_contacts),
        "false_positive": r.false_positive,
        "false_negative": r.false_negative,
    }


def main():
    print("=== Thresholds (derived from measured baseline) ===")
    print(f"  Baseline slip max (measured, Phase 4 5-trial set): {BASELINE_SLIP_MAX_M*1000:.3f}mm")
    print(f"  Slip detection threshold (3x baseline max): {SLIP_DETECTION_THRESHOLD_M*1000:.3f}mm")
    print(f"  Velocity detection threshold: {VELOCITY_DETECTION_THRESHOLD_MPS*1000:.1f}mm/s")
    print(f"  Force asymmetry detection threshold: {FORCE_ASYMMETRY_THRESHOLD_N}N")
    print(f"  Disturbance: {DISTURBANCE_FORCE_N}N for {DISTURBANCE_DURATION_STEPS} steps "
          f"({DISTURBANCE_DURATION_STEPS}ms)")
    print(f"  Recovery grip target: {RECOVERY_GRIP_TARGET_N}N")
    print()

    print("=== A: Negative controls (no disturbance) ===")
    neg_results = []
    for i, offset in enumerate(NEGATIVE_CONTROL_OFFSETS):
        r = run_slip_trial(inject_disturbance=False, payload_offset_xy=offset)
        entry = summarize(f"neg_control_{i}_{offset}", r, expect_disturbance=False)
        neg_results.append(entry)
        status = "PASS" if not r.false_positive else "FAIL (false positive)"
        print(f"[{status}] offset={offset}: max_slip={r.max_wrist_slip*1000:.4f}mm, "
              f"slip_detected={r.slip_detected}, final_state={r.final_state}")

    n_neg_pass = sum(1 for e in neg_results if not e["false_positive"])
    print(f"\nNegative control gate: {n_neg_pass}/{len(neg_results)} (zero false positives required)")
    print()

    print("=== B: Injected slip and recovery ===")
    inj_results = []
    for i, offset in enumerate(INJECTED_TRIAL_OFFSETS):
        r = run_slip_trial(inject_disturbance=True, payload_offset_xy=offset)
        entry = summarize(f"injected_{i}_{offset}", r, expect_disturbance=True)
        inj_results.append(entry)
        status = "PASS" if r.final_state == "RECOVERY_SUCCESS" else f"FAIL ({r.final_state})"
        print(f"[{status}] offset={offset}: detection_latency={r.detection_latency_steps} steps, "
              f"max_slip={r.max_wrist_slip*1000:.4f}mm, recovery_target={r.recovery_grip_target}N, "
              f"recovery_duration={r.recovery_duration_steps} steps, dropped={r.payload_dropped}")

    n_inj_pass = sum(1 for e in inj_results if e["final_state"] == "RECOVERY_SUCCESS")
    print(f"\nInjected recovery gate: {n_inj_pass}/{len(inj_results)} (>=3/3 required)")

    gate_passed = (n_neg_pass == len(neg_results)) and (n_inj_pass >= 3)
    print()
    print(f"PHASE 5 RELIABILITY GATE: {'PASS' if gate_passed else 'FAIL'}")

    report = {
        "phase": "Phase 5 - Slip Detection and Recovery",
        "thresholds": {
            "baseline_slip_max_mm": round(BASELINE_SLIP_MAX_M * 1000, 4),
            "slip_detection_threshold_mm": round(SLIP_DETECTION_THRESHOLD_M * 1000, 4),
            "velocity_detection_threshold_mps": VELOCITY_DETECTION_THRESHOLD_MPS,
            "force_asymmetry_threshold_n": FORCE_ASYMMETRY_THRESHOLD_N,
        },
        "disturbance": {
            "force_n": DISTURBANCE_FORCE_N,
            "duration_steps": DISTURBANCE_DURATION_STEPS,
        },
        "negative_controls": neg_results,
        "injected_trials": inj_results,
        "n_negative_pass": n_neg_pass,
        "n_negative_total": len(neg_results),
        "n_injected_pass": n_inj_pass,
        "n_injected_total": len(inj_results),
        "gate_passed": gate_passed,
    }
    with open("phase5_reliability_report.json", "w") as f:
        json.dump(report, f, indent=2, default=str)
    print("Report written to phase5_reliability_report.json")


if __name__ == "__main__":
    main()
