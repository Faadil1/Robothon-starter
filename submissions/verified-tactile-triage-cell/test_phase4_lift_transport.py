"""Phase 4 reliability harness (permanent file): 5 deterministic lift/
transport trials against production scene.xml. Logs every trial including
failures. Reconstructed from the validated Phase 4 behavior documented in
PROJECT_STATE.md / DECISION_LOG.md (solver Variant B already applied to
production scene.xml)."""
import json
from lift_transport import run_lift_transport_trial

DETERMINISTIC_OFFSETS = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0), (0.0, 0.001), (0.0, -0.001)]


def main():
    results = []
    for offset in DETERMINISTIC_OFFSETS:
        r = run_lift_transport_trial(payload_offset_xy=offset)
        entry = {
            "offset": offset,
            "success": r.success,
            "failure_reason": r.failure_reason,
            "min_force_lift_n": r.min_force_lift,
            "min_force_transport_n": r.min_force_transport,
            "peak_force_n": round(r.peak_force, 4),
            "max_relative_slip_mm": round(r.max_relative_slip * 1000, 4),
            "lift_height_m": round(r.lift_height_achieved, 5) if r.lift_height_achieved is not None else None,
            "transport_error_m": round(r.transport_error, 6) if r.transport_error is not None else None,
            "table_contact_during_airborne": r.table_contact_during_airborne,
            "unexpected_contacts": list(r.unexpected_contacts),
            "actuator_saturated": r.actuator_saturated,
            "nan_detected": r.nan_detected,
        }
        results.append(entry)
        status = "PASS" if r.success else f"FAIL ({r.failure_reason})"
        print(f"[{status}] offset={offset}: slip={r.max_relative_slip*1000:.3f}mm, "
              f"force_lift={r.min_force_lift}, force_transport={r.min_force_transport}, "
              f"lift_h={r.lift_height_achieved*1000:.2f}mm, "
              f"transport_err={r.transport_error*1000:.4f}mm")

    n_pass = sum(1 for e in results if e["success"])
    gate_passed = n_pass == 5
    print()
    print(f"PHASE 4 RELIABILITY GATE: {n_pass}/5 {'PASS' if gate_passed else 'FAIL'}")

    report = {
        "phase": "Phase 4 - Lift and Transport",
        "trials": results,
        "n_pass": n_pass,
        "n_total": 5,
        "gate_passed": gate_passed,
    }
    with open("phase4_reliability_report.json", "w") as f:
        json.dump(report, f, indent=2, default=str)
    print("Report written to phase4_reliability_report.json")


if __name__ == "__main__":
    main()
