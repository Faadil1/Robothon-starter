"""
verify_receipts.py — independent receipt verifier.

Deliberately minimal and self-contained: requires only the PUBLIC key
file and the receipt log. Does NOT import triage_receipt_logger.py's
signing path or require the private key, so it can be run by a third
party (or a CI job) that should never have private-key access, to
independently confirm receipts have not been tampered with.

Usage:
    python verify_receipts.py [path/to/receipts.jsonl]
"""
import sys
import json
import base64
import os
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
from cryptography.hazmat.primitives import serialization

PUBLIC_KEY_PATH = os.path.join("keys", "triage_receipt_public.pem")
DEFAULT_LOG_PATH = os.path.join("logs", "triage_receipts.jsonl")

# Must match triage_receipt_logger.py's SIGNED_FIELDS exactly -- duplicated
# here deliberately so this verifier has no import-time dependency on the
# signing module (independence is the point).
SIGNED_FIELDS = [
    "episode_id",
    "scenario",
    "timestamp",
    "planning_verdict",
    "planning_reason",
    "prerelease_verdict",
    "prerelease_reason",
    "min_force_transport_n",
    "peak_force_n",
    "disturbance_or_slip_risk_detected",
    "displacement_slip_detected",
    "displacement_slip_max_mm",
    "recovery_action_taken",
    "recovery_outcome",
    "final_state",
    "placement_error_m",
    "contaminated_zone_entered",
    "contaminated_zone_release",
]


def load_public_key(path=PUBLIC_KEY_PATH):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Public key not found at {path}. The private key must never be "
            f"used for verification -- only the public key is needed."
        )
    with open(path, "rb") as f:
        key = serialization.load_pem_public_key(f.read())
    if not isinstance(key, Ed25519PublicKey):
        raise ValueError("Key at the given path is not an Ed25519 public key.")
    return key


def canonical_message(event: dict) -> bytes:
    try:
        signed_subset = {k: event[k] for k in SIGNED_FIELDS}
    except KeyError as e:
        raise ValueError(f"Receipt missing required signed field: {e}")
    return json.dumps(signed_subset, sort_keys=True, separators=(",", ":")).encode("utf-8")


def verify_one(public_key, event: dict) -> bool:
    if event.get("signature_algorithm") != "ed25519":
        return False
    try:
        message = canonical_message(event)
        signature = base64.urlsafe_b64decode(event["signature"])
        public_key.verify(signature, message)
        return True
    except Exception:
        return False


def verify_file(path):
    public_key = load_public_key()
    n_ok, n_total = 0, 0
    seen_episode_ids = set()
    duplicate_episode_ids = []
    results = []

    with open(path) as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            n_total += 1
            event = json.loads(line)
            ok = verify_one(public_key, event)
            n_ok += 1 if ok else 0

            ep_id = event.get("episode_id")
            if ep_id in seen_episode_ids:
                duplicate_episode_ids.append(ep_id)
            seen_episode_ids.add(ep_id)

            results.append({"line": line_no, "episode_id": ep_id, "verified": ok})

    return {
        "n_ok": n_ok,
        "n_total": n_total,
        "all_verified": (n_total > 0 and n_ok == n_total),
        "unique_episode_ids": len(seen_episode_ids),
        "duplicate_episode_ids": duplicate_episode_ids,
        "results": results,
    }


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG_PATH
    if not os.path.exists(path):
        print(f"No receipt log found at {path}")
        sys.exit(1)

    report = verify_file(path)
    print(f"Verified {report['n_ok']}/{report['n_total']} receipts in {path}")
    print(f"Unique episode IDs: {report['unique_episode_ids']}")
    if report["duplicate_episode_ids"]:
        print(f"WARNING: duplicate episode IDs found: {report['duplicate_episode_ids']}")
    sys.exit(0 if report["all_verified"] and not report["duplicate_episode_ids"] else 1)
