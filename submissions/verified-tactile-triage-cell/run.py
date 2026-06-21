"""
run.py — one-command production validation.

Usage:
    python run.py

Runs the complete reliability harness against production scene.xml,
prints a compact summary, writes reliability_report.json, and exits 0
only when every mandatory gate passes. Never runs against
solver_experiment variants.
"""
import sys
import json
from reliability_harness import main as run_harness


def print_summary():
    with open("reliability_report.json") as f:
        report = json.load(f)

    print()
    print("=" * 50)
    print("VERIFIED TACTILE TRIAGE CELL — VALIDATION SUMMARY")
    print("=" * 50)
    print(f"Production scene: {report['production_scene_path']}")
    print(f"Timestamp: {report['timestamp']}")
    print()
    for p in report["phases"]:
        status = "PASS" if p["passed"] else "FAIL"
        print(f"  [{status}] Phase {p['phase']}: {p['name']}")
    print()
    overall = "PASS" if report["all_mandatory_gates_passed"] else "FAIL"
    print(f"OVERALL: {overall}")
    print("=" * 50)


if __name__ == "__main__":
    exit_code = run_harness()
    print_summary()
    sys.exit(exit_code)
