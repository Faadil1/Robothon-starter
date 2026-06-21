"""
triage_receipt_logger.py — Ed25519 signed receipts for the Verified
Tactile Triage Cell.

Same conventions as the Safety-Gated Pusher project's receipt_logger.py:
canonical sorted-key JSON over an explicit SIGNED_FIELDS list, Ed25519
sign/verify, key_id, append-only .jsonl logs. This is a fresh
implementation for this prototype (separate key material from the pusher
project), not a shared/reused key.

A receipt for one full triage episode covers:
  - both gate checkpoints (planning, pre_release) -- verdict + reason each
  - tactile force evidence (min/peak forces during transport)
  - disturbance/slip-risk detection (force-asymmetry channel) -- separate
    from displacement-based slip detection, per the Phase 5 correction
  - displacement-slip status (whether the 2.43mm threshold was crossed)
  - recovery action taken (if any) and outcome
  - placement outcome (final state, error, contaminated-zone checks)
"""
import json
import base64
import os
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey,
    Ed25519PublicKey,
)
from cryptography.hazmat.primitives import serialization

KEY_DIR = "keys"
PRIVATE_KEY_PATH = os.path.join(KEY_DIR, "triage_receipt_private.pem")
PUBLIC_KEY_PATH = os.path.join(KEY_DIR, "triage_receipt_public.pem")
KEY_ID = "triage-cell-key-1"

LOGS_DIR = "logs"
RECEIPTS_LOG_PATH = os.path.join(LOGS_DIR, "triage_receipts.jsonl")

# Exactly these fields are covered by the signature (canonical, sorted keys).
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


def ensure_keys():
    os.makedirs(KEY_DIR, exist_ok=True)
    if os.path.exists(PRIVATE_KEY_PATH) and os.path.exists(PUBLIC_KEY_PATH):
        with open(PRIVATE_KEY_PATH, "rb") as f:
            return serialization.load_pem_private_key(f.read(), password=None)

    private_key = Ed25519PrivateKey.generate()
    public_key = private_key.public_key()

    priv_bytes = private_key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.PKCS8,
        encryption_algorithm=serialization.NoEncryption(),
    )
    pub_bytes = public_key.public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo,
    )
    with open(PRIVATE_KEY_PATH, "wb") as f:
        f.write(priv_bytes)
    with open(PUBLIC_KEY_PATH, "wb") as f:
        f.write(pub_bytes)
    return private_key


def canonical_message(event: dict) -> bytes:
    signed_subset = {k: event[k] for k in SIGNED_FIELDS}
    return json.dumps(signed_subset, sort_keys=True, separators=(",", ":")).encode("utf-8")


def sign_event(event: dict) -> dict:
    private_key = ensure_keys()
    message = canonical_message(event)
    signature = private_key.sign(message)

    signed_event = dict(event)
    signed_event["key_id"] = KEY_ID
    signed_event["signature_algorithm"] = "ed25519"
    signed_event["signature"] = base64.urlsafe_b64encode(signature).decode("ascii")
    return signed_event


def verify_event(signed_event: dict) -> bool:
    if signed_event.get("signature_algorithm") != "ed25519":
        return False
    if not os.path.exists(PUBLIC_KEY_PATH):
        return False
    with open(PUBLIC_KEY_PATH, "rb") as f:
        public_key = serialization.load_pem_public_key(f.read())
    if not isinstance(public_key, Ed25519PublicKey):
        return False
    try:
        message = canonical_message(signed_event)
        signature = base64.urlsafe_b64decode(signed_event["signature"])
        public_key.verify(signature, message)
        return True
    except Exception:
        return False


def log_receipt(event: dict):
    os.makedirs(LOGS_DIR, exist_ok=True)
    signed = sign_event(event)
    with open(RECEIPTS_LOG_PATH, "a") as f:
        f.write(json.dumps(signed) + "\n")
    return signed


def verify_log_file(path=RECEIPTS_LOG_PATH):
    if not os.path.exists(path):
        return 0, 0
    n_ok, n_total = 0, 0
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            n_total += 1
            event = json.loads(line)
            if verify_event(event):
                n_ok += 1
    return n_ok, n_total


def build_receipt_from_placement_result(r, episode_id):
    """Construct a receipt event dict from a placement_controller.PlacementResult."""
    import datetime
    release_forces = [f for pair in r.release_force_profile for f in pair] if r.release_force_profile else []
    peak_force = max(release_forces) if release_forces else None
    return {
        "episode_id": episode_id,
        "scenario": "A" if (r.planning_verdict and r.planning_verdict["verdict"] == "ALLOW") else "B",
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "planning_verdict": r.planning_verdict["verdict"] if r.planning_verdict else None,
        "planning_reason": r.planning_verdict["reason"] if r.planning_verdict else None,
        "prerelease_verdict": r.prerelease_verdict["verdict"] if r.prerelease_verdict else None,
        "prerelease_reason": r.prerelease_verdict["reason"] if r.prerelease_verdict else None,
        "min_force_transport_n": list(r.min_force_transport) if r.min_force_transport[0] is not None else [None, None],
        "peak_force_n": peak_force,
        "disturbance_or_slip_risk_detected": False,  # Phase 6 trials: no disturbance injected
        "displacement_slip_detected": False,
        "displacement_slip_max_mm": None,
        "recovery_action_taken": None,
        "recovery_outcome": None,
        "final_state": r.final_state,
        "placement_error_m": r.placement_error_m,
        "contaminated_zone_entered": r.contaminated_zone_entered,
        "contaminated_zone_release": r.contaminated_zone_release,
    }


def build_receipt_from_slip_result(r, episode_id):
    """Construct a receipt event dict from a slip_recovery.SlipRecoveryResult."""
    import datetime
    displacement_slip_detected = r.slip_at_detection is not None and r.max_wrist_slip * 1000 > 2.43
    return {
        "episode_id": episode_id,
        "scenario": "slip_recovery",
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "planning_verdict": None,
        "planning_reason": None,
        "prerelease_verdict": None,
        "prerelease_reason": None,
        "min_force_transport_n": list(r.force_before) if r.force_before[0] is not None else [None, None],
        "peak_force_n": None,
        "disturbance_or_slip_risk_detected": r.slip_detected,
        "displacement_slip_detected": bool(displacement_slip_detected),
        "displacement_slip_max_mm": round(r.max_wrist_slip * 1000, 4),
        "recovery_action_taken": r.recovery_grip_target,
        "recovery_outcome": r.final_state,
        "final_state": r.final_state,
        "placement_error_m": None,
        "contaminated_zone_entered": None,
        "contaminated_zone_release": None,
    }


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 3 and sys.argv[1] == "verify":
        path = sys.argv[2]
        n_ok, n_total = verify_log_file(path)
        print(f"verified {n_ok}/{n_total} receipts in {path}")
        sys.exit(0 if (n_total > 0 and n_ok == n_total) else 1)
    else:
        print("usage: python triage_receipt_logger.py verify <path>")
        sys.exit(1)
