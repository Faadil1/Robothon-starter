"""Safety gate — pure function, unit-testable standalone, no MuJoCo dependency."""


def point_in_rect(point_xy, center_xy, half_extents_xy):
    """Check if point falls within an axis-aligned rectangle."""
    px, py = point_xy
    cx, cy = center_xy
    hx, hy = half_extents_xy
    return (cx - hx <= px <= cx + hx) and (cy - hy <= py <= cy + hy)


def check_safety(plan, nogo_center_xy, nogo_half_extents_xy):
    """
    Check whether any waypoint in the planned push path crosses the no-go zone.

    Returns (verdict, reason) where verdict is "ALLOW" or "BLOCK".
    """
    for wp in plan.waypoints:
        if point_in_rect(wp, nogo_center_xy, nogo_half_extents_xy):
            return "BLOCK", "planned push path crosses no-go zone"
    return "ALLOW", "path clear of no-go zone"
