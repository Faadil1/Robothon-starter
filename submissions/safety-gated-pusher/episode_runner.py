"""Episode runner — orchestrates one scenario: plan -> gate -> execute (or not)."""
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import uuid
import datetime
import mujoco
import numpy as np

from controller import plan_push
from safety_gate import check_safety

NOGO_CENTER_XY = (0.15, -0.25)
NOGO_HALF_XY = (0.5, 0.15)

GOAL_CENTER_XY = (0.55, 0.45)
GOAL_HALF_XY = (0.12, 0.12)

SCENE_PATH = "scene.xml"


# Pusher body anchor position (must match scene.xml body pos for pusher)
PUSHER_ANCHOR_XY = (-0.55, 0.45)


def in_goal_zone(object_pos_xy):
    px, py = object_pos_xy
    cx, cy = GOAL_CENTER_XY
    hx, hy = GOAL_HALF_XY
    return (cx - hx <= px <= cx + hx) and (cy - hy <= py <= cy + hy)


def run_episode(scenario_label, target_xy, model=None, data=None, renderer=None,
                 frame_collector=None):
    """
    Run a single episode: plan a push toward target_xy, check the safety
    gate, and either execute (ALLOW) or halt before any pusher motion
    (BLOCK). Returns an event dict describing the outcome.

    If model/data are not provided, fresh ones are created (resets scene).
    frame_collector, if provided, is a callable(model, data, renderer) used
    to capture frames during execution (for video capture later).
    """
    if model is None:
        model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    if data is None:
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)

    object_body_id = model.body("object").id
    object_start = data.xpos[object_body_id][:2].copy()

    plan = plan_push(tuple(object_start), target_xy)
    verdict, reason = check_safety(plan, NOGO_CENTER_XY, NOGO_HALF_XY)

    event = {
        "episode_id": str(uuid.uuid4()),
        "scenario": scenario_label,
        "verdict": verdict,
        "reason": reason,
        "planned_target": [float(target_xy[0]), float(target_xy[1])],
        "object_start_pos": [float(object_start[0]), float(object_start[1])],
        "object_end_pos": None,
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }

    if verdict == "BLOCK":
        # Do NOT execute the push. Settle physics briefly with no pusher
        # motion so the scene is visibly static/unchanged, then record.
        if frame_collector is not None:
            for _ in range(60):
                mujoco.mj_step(model, data)
                frame_collector(model, data, renderer)
        event["object_end_pos"] = [float(object_start[0]), float(object_start[1])]
        return event, model, data

    # ALLOW: execute the planned push waypoints via position actuators
    act_x_id = model.actuator("act_x").id
    act_y_id = model.actuator("act_y").id

    for wp in plan.waypoints:
        # Convert absolute world waypoint to joint-relative offset from the
        # pusher body's anchor position (slide joints are anchor-relative).
        data.ctrl[act_x_id] = wp[0] - PUSHER_ANCHOR_XY[0]
        data.ctrl[act_y_id] = wp[1] - PUSHER_ANCHOR_XY[1]
        for _ in range(20):  # substeps per waypoint for smoother motion
            mujoco.mj_step(model, data)
            if frame_collector is not None:
                frame_collector(model, data, renderer)

    # Let physics settle briefly after push completes
    for _ in range(80):
        mujoco.mj_step(model, data)
        if frame_collector is not None:
            frame_collector(model, data, renderer)

    object_end = data.xpos[object_body_id][:2].copy()
    event["object_end_pos"] = [float(object_end[0]), float(object_end[1])]
    return event, model, data
