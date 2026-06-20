"""
Receipt logger — thin Ed25519 signing layer over episode verdict events.

NOTE: This is a fresh minimal implementation, not a reuse of the original
`receipts.py` (Ed25519 core from the Agent Decision Receipt project) — that
file was not present in this build environment. This module follows the same
conventions referenced in prior work (canonical sorted-key JSON, Ed25519,
key_id) but is newly written and freshly tested here.

Design intent: thin evidence layer only. Signs the verdict produced by the
safety gate for a real, already-executed (or already-blocked) simulation
episode. Never the primary deliverable — appears in logs only, no UI.
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
PRIVATE_KEY_PATH = os.path.join(KEY_DIR, "receipt_private.pem")
PUBLIC_KEY_PATH = os.path.join(KEY_DIR, "receipt_public.pem")
KEY_ID = "robothon-pusher-key-1"

LOGS_DIR = "logs"
RECEIPTS_LOG_PATH = os.path.join(LOGS_DIR, "receipts.jsonl")
EPISODES_LOG_PATH = os.path.join(LOGS_DIR, "episodes.jsonl")

# Exactly these fields are covered by the signature (canonical, sorted keys).
SIGNED_FIELDS = [
    "episode_id",
    "scenario",
    "verdict",
    "reason",
    "planned_target",
    "object_start_pos",
    "object_end_pos",
    "timestamp",
    "contact_count",
    "max_contact_force",
]


def ensure_keys():
    """Generate an Ed25519 keypair on first run; reuse thereafter."""
    os.makedirs(KEY_DIR, exist_ok=True)
    if os.path.exists(PRIVATE_KEY_PATH) and os.path.exists(PUBLIC_KEY_PATH):
        with open(PRIVATE_KEY_PATH, "rb") as f:
            private_key = serialization.load_pem_private_key(f.read(), password=None)
        return private_key

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
    """Build the canonical signed message: sorted-key compact JSON over
    exactly SIGNED_FIELDS, UTF-8 encoded."""
    signed_subset = {k: event[k] for k in SIGNED_FIELDS}
    return json.dumps(signed_subset, sort_keys=True, separators=(",", ":")).encode("utf-8")


def sign_event(event: dict) -> dict:
    """Sign an episode event dict, returning the event plus signature fields."""
    private_key = ensure_keys()
    message = canonical_message(event)
    signature = private_key.sign(message)

    signed_event = dict(event)
    signed_event["key_id"] = KEY_ID
    signed_event["signature_algorithm"] = "ed25519"
    signed_event["signature"] = base64.urlsafe_b64encode(signature).decode("ascii")
    return signed_event


def verify_event(signed_event: dict) -> bool:
    """Independently verify a signed event's Ed25519 signature."""
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


def log_episode(event: dict):
    """Append the raw unsigned event to episodes.jsonl (debugging)."""
    os.makedirs(LOGS_DIR, exist_ok=True)
    with open(EPISODES_LOG_PATH, "a") as f:
        f.write(json.dumps(event) + "\n")


def log_receipt(event: dict):
    """Sign the event and append to receipts.jsonl."""
    os.makedirs(LOGS_DIR, exist_ok=True)
    signed = sign_event(event)
    with open(RECEIPTS_LOG_PATH, "a") as f:
        f.write(json.dumps(signed) + "\n")
    return signed


def verify_log_file(path=RECEIPTS_LOG_PATH):
    """Verify every signed receipt in a .jsonl log file. Returns (n_ok, n_total)."""
    if not os.path.exists(path):
        return 0, 0
    n_ok = 0
    n_total = 0
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


if __name__ == "__main__":
    import sys

    if len(sys.argv) >= 3 and sys.argv[1] == "verify":
        path = sys.argv[2]
        n_ok, n_total = verify_log_file(path)
        print(f"verified {n_ok}/{n_total} receipts in {path}")
        sys.exit(0 if (n_total > 0 and n_ok == n_total) else 1)
    else:
        print("usage: python receipt_logger.py verify <path>")
        sys.exit(1)
