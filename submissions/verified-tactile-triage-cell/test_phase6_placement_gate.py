"""Phase 6 reliability harness: Scenario A (safe placement) 5/5, Scenario B
(unsafe-request blocked) 5/5, plus malformed/unknown-destination unit
tests against the gate in isolation. Logs every trial including failures."""
import json
from placement_controller import run_placement_trial, SAFE_ZONE, CONTAMINATED_ZONE, ZONES
from triage_safety_gate import evaluate_placement

DETERMINISTIC_OFFSETS = [(0.0, 0.0), (0.001, 0.0), (-0.001, 0.0), (0.0, 0.001), (0.0, -0.001)]


def summarize(label, r, scenario):
    return {
        "trial": label,
        "scenario": scenario,
        "requested_target": r.requested_target,
        "final_state": r.final_state,
        "planning_gate_ran": r.planning_gate_ran,
        "planning_verdict": r.planning_verdict["verdict"] if r.planning_verdict else None,
        "planning_reason": r.planning_verdict["reason"] if r.planning_verdict else None,
        "prerelease_gate_ran": r.prerelease_gate_ran,
        "prerelease_verdict": r.prerelease_verdict["verdict"] if r.prerelease_verdict else None,
        "actual_release_xy": r.actual_release_xy,
        "final_payload_xy": r.final_payload_xy,
        "placement_error_m": round(r.placement_error_m, 6) if r.placement_error_m is not None else None,
        "payload_stability_velocity_mps": round(r.payload_stability_velocity, 8) if r.payload_stability_velocity is not None else None,
        "min_force_transport_n": r.min_force_transport,
        "contaminated_zone_entered": r.contaminated_zone_entered,
        "contaminated_zone_release": r.contaminated_zone_release,
        "nan_detected": r.nan_detected,
        "unexpected_contacts": list(r.unexpected_contacts),
        "gate_bypassed": r.gate_bypassed,
    }


def main():
    print("=== Zone geometry ===")
    print(f"  SAFE_ZONE: center={SAFE_ZONE.center_xy}, half_extent={SAFE_ZONE.half_extent_xy}")
    print(f"  CONTAMINATED_ZONE: center={CONTAMINATED_ZONE.center_xy}, half_extent={CONTAMINATED_ZONE.half_extent_xy}")
    print()

    print("=== Malformed/unknown destination unit tests (gate only, no simulation) ===")
    malformed_cases = [
        ("none", None),
        ("string_tuple", ("a", "b")),
        ("nan", (float("nan"), 0.0)),
        ("outside_all_zones", (0.0, 0.0)),
        ("wrong_length", (0.1,)),
        ("inf", (float("inf"), 0.0)),
    ]
    malformed_results = []
    all_blocked = True
    for label, target in malformed_cases:
        v = evaluate_placement(target, ZONES)
        ok = v["verdict"] == "BLOCK"
        all_blocked = all_blocked and ok
        malformed_results.append({"case": label, "target": target, "verdict": v["verdict"], "reason": v["reason"]})
        print(f"[{'PASS' if ok else 'FAIL'}] {label}: target={target} -> {v['verdict']} ({v['reason']})")
    print(f"\nMalformed-input gate: {'PASS' if all_blocked else 'FAIL'} (all must BLOCK)\n")

    print("=== Scenario A: safe placement (5 trials) ===")
    a_results = []
    for i, offset in enumerate(DETERMINISTIC_OFFSETS):
        r = run_placement_trial(requested_target_xy=SAFE_ZONE.center_xy, payload_offset_xy=offset)
        entry = summarize(f"scenario_a_{i}_{offset}", r, "A")
        a_results.append(entry)
        ok = r.final_state == "PLACED_SAFE"
        print(f"[{'PASS' if ok else 'FAIL'}] offset={offset}: final_state={r.final_state}, "
              f"error={r.placement_error_m}, contaminated_entered={r.contaminated_zone_entered}")

    n_a_pass = sum(1 for e in a_results if e["final_state"] == "PLACED_SAFE")
    n_a_contaminated_entry = sum(1 for e in a_results if e["contaminated_zone_entered"])
    n_a_contaminated_release = sum(1 for e in a_results if e["contaminated_zone_release"])
    print(f"\nScenario A gate: {n_a_pass}/5 PASS, contaminated entries={n_a_contaminated_entry}/5, "
          f"contaminated releases={n_a_contaminated_release}/5\n")

    print("=== Scenario B: unsafe destination blocked (5 trials) ===")
    b_results = []
    for i, offset in enumerate(DETERMINISTIC_OFFSETS):
        r = run_placement_trial(requested_target_xy=CONTAMINATED_ZONE.center_xy, payload_offset_xy=offset)
        entry = summarize(f"scenario_b_{i}_{offset}", r, "B")
        b_results.append(entry)
        ok = (r.final_state == "BLOCKED_FALLBACK_SAFE" and not r.contaminated_zone_release
              and r.planning_verdict["verdict"] == "BLOCK")
        print(f"[{'PASS' if ok else 'FAIL'}] offset={offset}: final_state={r.final_state}, "
              f"planning_verdict={r.planning_verdict['verdict']}, "
              f"contaminated_entered={r.contaminated_zone_entered}, "
              f"contaminated_released={r.contaminated_zone_release}")

    n_b_pass = sum(1 for e in b_results
                   if e["final_state"] == "BLOCKED_FALLBACK_SAFE" and not e["contaminated_zone_release"]
                   and e["planning_verdict"] == "BLOCK")
    n_b_contaminated_entry = sum(1 for e in b_results if e["contaminated_zone_entered"])
    n_b_contaminated_release = sum(1 for e in b_results if e["contaminated_zone_release"])
    n_b_planning_ran = sum(1 for e in b_results if e["planning_gate_ran"])
    n_b_prerelease_ran = sum(1 for e in b_results if e["prerelease_gate_ran"])
    print(f"\nScenario B gate: {n_b_pass}/5 PASS, contaminated entries={n_b_contaminated_entry}/5, "
          f"contaminated releases={n_b_contaminated_release}/5, "
          f"planning_ran={n_b_planning_ran}/5, prerelease_ran={n_b_prerelease_ran}/5\n")

    gate_passed = (
        n_a_pass == 5 and n_a_contaminated_release == 0
        and n_b_pass == 5 and n_b_contaminated_entry == 0 and n_b_contaminated_release == 0
        and n_b_planning_ran == 5 and n_b_prerelease_ran == 5
        and all_blocked
    )
    print(f"PHASE 6 RELIABILITY GATE: {'PASS' if gate_passed else 'FAIL'}")

    report = {
        "phase": "Phase 6 - Triage Decision and Placement",
        "zones": {
            "safe": {"center": SAFE_ZONE.center_xy, "half_extent": SAFE_ZONE.half_extent_xy},
            "contaminated": {"center": CONTAMINATED_ZONE.center_xy, "half_extent": CONTAMINATED_ZONE.half_extent_xy},
        },
        "malformed_input_tests": malformed_results,
        "malformed_input_all_blocked": all_blocked,
        "scenario_a_trials": a_results,
        "scenario_b_trials": b_results,
        "n_scenario_a_pass": n_a_pass,
        "n_scenario_b_pass": n_b_pass,
        "n_contaminated_entries_total": n_a_contaminated_entry + n_b_contaminated_entry,
        "n_contaminated_releases_total": n_a_contaminated_release + n_b_contaminated_release,
        "gate_passed": gate_passed,
    }
    with open("phase6_reliability_report.json", "w") as f:
        json.dump(report, f, indent=2, default=str)
    print("Report written to phase6_reliability_report.json")


if __name__ == "__main__":
    main()
