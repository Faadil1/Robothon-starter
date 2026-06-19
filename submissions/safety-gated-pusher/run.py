"""
run.py — CLI entrypoint for the Safety-Gated Pusher demo.

Runs Scenario A (ALLOW: safe push reaches goal) and Scenario B (BLOCK:
unsafe push blocked before execution), writing episode logs and signed
receipts for both. Exits 0 on success, non-zero on any failure.

Usage:
    python run.py
"""
import sys

from episode_runner import run_episode, in_goal_zone, GOAL_CENTER_XY, GOAL_HALF_XY
from receipt_logger import log_episode, log_receipt, verify_log_file

SCENARIO_A_TARGET = (0.55, 0.45)   # within goal zone -> ALLOW expected
SCENARIO_B_TARGET = (0.4, -0.25)   # across no-go zone -> BLOCK expected


def main():
    print("=== Safety-Gated Pusher — Robothon 2026 ===\n")

    failures = []

    # --- Scenario A: ALLOW ---
    print("[Scenario A] Planning safe push toward goal zone...")
    event_a, _, _ = run_episode("A", target_xy=SCENARIO_A_TARGET)
    log_episode(event_a)
    signed_a = log_receipt(event_a)
    print(f"  verdict: {event_a['verdict']} ({event_a['reason']})")
    print(f"  object_start: {event_a['object_start_pos']}")
    print(f"  object_end:   {event_a['object_end_pos']}")

    if event_a["verdict"] != "ALLOW":
        failures.append("Scenario A: expected verdict ALLOW, got " + event_a["verdict"])
    if not in_goal_zone(tuple(event_a["object_end_pos"])):
        failures.append("Scenario A: object did not reach goal zone")
    print()

    # --- Scenario B: BLOCK ---
    print("[Scenario B] Planning unsafe push across no-go zone...")
    event_b, _, _ = run_episode("B", target_xy=SCENARIO_B_TARGET)
    log_episode(event_b)
    signed_b = log_receipt(event_b)
    print(f"  verdict: {event_b['verdict']} ({event_b['reason']})")
    print(f"  object_start: {event_b['object_start_pos']}")
    print(f"  object_end:   {event_b['object_end_pos']}")

    if event_b["verdict"] != "BLOCK":
        failures.append("Scenario B: expected verdict BLOCK, got " + event_b["verdict"])
    if event_b["object_start_pos"] != event_b["object_end_pos"]:
        failures.append("Scenario B: object moved despite BLOCK verdict")
    print()

    # --- Verify receipts ---
    print("[Receipts] Verifying signed receipt log...")
    n_ok, n_total = verify_log_file()
    print(f"  verified {n_ok}/{n_total} receipts")
    if n_total == 0 or n_ok != n_total:
        failures.append(f"Receipt verification failed: {n_ok}/{n_total} valid")
    print()

    if failures:
        print("RESULT: FAIL")
        for f in failures:
            print(f"  - {f}")
        return 1

    print("RESULT: PASS — both scenarios executed and verified correctly.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
