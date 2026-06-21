"""Phase 3 reliability harness: 5/5 grasp-and-hold trials with deterministic
payload position perturbations. Logs every trial including failures."""
import json
from tactile_grasp import run_tactile_grasp_trial, PHYSICS_MIN_FORCE, TARGET_FORCE

TRIALS = [
    ("center", (0.0, 0.0)),
    ("x_plus_1mm", (0.001, 0.0)),
    ("x_minus_1mm", (-0.001, 0.0)),
    ("y_plus_1mm", (0.0, 0.001)),
    ("y_minus_1mm", (0.0, -0.001)),
]

results = []
for label, offset in TRIALS:
    r = run_tactile_grasp_trial(payload_offset_xy=offset)
    hold_duration_s = 2200 * 0.001  # HOLD_STEPS * timestep
    entry = {
        "trial": label,
        "payload_offset_xy": offset,
        "success": r.success,
        "failure_reason": r.failure_reason,
        "contact_acquired_step": r.contact_acquired_step,
        "left_peak_force": round(r.left_peak_force, 4),
        "right_peak_force": round(r.right_peak_force, 4),
        "left_steady_force": round(r.left_steady_force, 4),
        "right_steady_force": round(r.right_steady_force, 4),
        "force_asymmetry": round(r.max_force_asymmetry, 6),
        "payload_max_displacement_m": round(r.payload_max_displacement, 6),
        "actuator_saturated": r.actuator_saturated,
        "nan_detected": r.nan_detected,
        "unexpected_contacts": list(r.unexpected_contacts),
        "hold_duration_s": hold_duration_s,
        "physics_min_force_n": round(PHYSICS_MIN_FORCE, 4),
        "target_force_n": TARGET_FORCE,
    }
    results.append(entry)
    print(f"[{'PASS' if r.success else 'FAIL'}] {label}: "
          f"offset={offset}, steady=({r.left_steady_force:.3f},{r.right_steady_force:.3f})N, "
          f"asymmetry={r.max_force_asymmetry:.6f}N, disp={r.payload_max_displacement:.6f}m, "
          f"reason={r.failure_reason}")

n_pass = sum(1 for e in results if e["success"])
print()
print(f"RELIABILITY GATE: {n_pass}/5 PASS")

with open("phase3_reliability_report.json", "w") as f:
    json.dump({
        "phase": "Phase 3 - Bilateral Tactile Grasp",
        "gate": "5/5 grasp and hold",
        "trials": results,
        "n_pass": n_pass,
        "n_total": len(results),
        "gate_passed": n_pass == len(results),
    }, f, indent=2)
print("Report written to phase3_reliability_report.json")
