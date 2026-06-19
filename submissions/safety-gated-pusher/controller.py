"""Deterministic push planning — no RL, no learning, pure geometry."""
import numpy as np


class PushPlan:
    def __init__(self, start, target, waypoints):
        self.start = np.array(start, dtype=float)
        self.target = np.array(target, dtype=float)
        self.waypoints = waypoints  # list of (x, y) pusher target positions


def plan_push(object_pos_xy, target_xy, n_steps=40, approach_offset=0.08):
    """
    Plan a straight-line push: pusher approaches behind the object (opposite
    side from target), then drives through the object toward target.

    object_pos_xy, target_xy: (x, y) tuples
    """
    object_pos_xy = np.array(object_pos_xy, dtype=float)
    target_xy = np.array(target_xy, dtype=float)

    direction = target_xy - object_pos_xy
    norm = np.linalg.norm(direction)
    if norm < 1e-6:
        raise ValueError("object already at target")
    unit = direction / norm

    # Pusher starting waypoint: behind the object, opposite the push direction
    approach_pos = object_pos_xy - unit * approach_offset

    # Waypoints: approach -> push through to target (pusher ends slightly
    # past object's target so contact force carries object to target)
    push_end = target_xy + unit * 0.02

    waypoints = []
    for i in range(n_steps + 1):
        t = i / n_steps
        wp = approach_pos + (push_end - approach_pos) * t
        waypoints.append(tuple(wp))

    return PushPlan(start=object_pos_xy, target=target_xy, waypoints=waypoints)
