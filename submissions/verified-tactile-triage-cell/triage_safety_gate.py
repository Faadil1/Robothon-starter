"""
triage_safety_gate.py — pure decision function for triage placement.

Deliberately independent of MuJoCo, the controller, and any simulation
state: this module only does geometry and policy. That makes it directly
unit-testable and means the controller cannot bypass it by constructing
its own ad-hoc check -- there is exactly one function that produces a
placement verdict, and the controller is required to call it.

Zone definitions are passed in explicitly (not hardcoded) so tests can
exercise edge cases without touching the real scene.
"""
import datetime


class Zone:
    def __init__(self, name, center_xy, half_extent_xy, kind):
        """kind: 'safe' or 'unsafe'."""
        self.name = name
        self.center_xy = center_xy
        self.half_extent_xy = half_extent_xy
        self.kind = kind

    def contains(self, point_xy):
        px, py = point_xy
        cx, cy = self.center_xy
        hx, hy = self.half_extent_xy
        return (cx - hx <= px <= cx + hx) and (cy - hy <= py <= cy + hy)


def evaluate_placement(target_xy, zones, episode_id=None, check_label="planning"):
    """
    Evaluate whether a requested placement target is safe.

    target_xy: (x, y) requested destination, or None/malformed input.
    zones: list of Zone objects (at least one 'safe', at least one 'unsafe'
        expected, but the function does not assume this -- it checks what
        it's given).
    check_label: 'planning' or 'pre_release' -- recorded in the verdict so
        callers can confirm both checkpoints actually ran.

    Returns a dict: {verdict, reason, matched_zone, target_xy, episode_id,
    check_label, timestamp}. verdict is always exactly "ALLOW" or "BLOCK".

    Default-deny policy: any malformed, missing, out-of-bounds-of-all-
    known-zones, or ambiguous (matches zero zones, or matches more than
    one zone) target results in BLOCK.
    """
    timestamp = datetime.datetime.now(datetime.timezone.utc).isoformat()

    base = {
        "target_xy": target_xy,
        "episode_id": episode_id,
        "check_label": check_label,
        "timestamp": timestamp,
    }

    # Malformed input: not a 2-tuple of finite numbers.
    if target_xy is None:
        return {**base, "verdict": "BLOCK", "reason": "target_missing", "matched_zone": None}
    try:
        x, y = target_xy
        x = float(x)
        y = float(y)
    except (TypeError, ValueError):
        return {**base, "verdict": "BLOCK", "reason": "target_malformed", "matched_zone": None}

    if not (x == x and y == y):  # NaN check (NaN != NaN)
        return {**base, "verdict": "BLOCK", "reason": "target_nan", "matched_zone": None}

    matches = [z for z in zones if z.contains((x, y))]

    if len(matches) == 0:
        return {**base, "verdict": "BLOCK", "reason": "target_outside_all_known_zones", "matched_zone": None}

    if len(matches) > 1:
        # Ambiguous: target falls inside more than one zone (e.g.
        # overlapping zone definitions). Default-deny rather than guess.
        return {
            **base, "verdict": "BLOCK", "reason": "target_ambiguous_multiple_zones",
            "matched_zone": [z.name for z in matches],
        }

    zone = matches[0]
    if zone.kind == "safe":
        return {**base, "verdict": "ALLOW", "reason": "target_inside_safe_zone", "matched_zone": zone.name}
    else:
        return {**base, "verdict": "BLOCK", "reason": "target_inside_unsafe_zone", "matched_zone": zone.name}
