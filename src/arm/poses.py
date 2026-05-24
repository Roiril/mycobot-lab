"""Named joint poses (degrees). All 6 joints in order J1..J6."""

# Upright neutral, end effector pointing up-forward. Safe for clearance check.
HOME = [0, 0, -90, 0, 0, 0]

# Compact rest pose — folded, low profile. Use before power_off / release.
REST = [0, 90, -130, 40, 0, 0]

# Ready-to-grasp horizontal reach forward.
READY = [0, -30, -60, 0, 0, 0]

# Greet wave — used for demos.
WAVE_A = [0, 0, -90, 30, 30, 0]
WAVE_B = [0, 0, -90, -30, -30, 0]

# --- Observation poses (vision Phase 1) ---
# Workspace surveillance poses for /perceive. Wrist camera looks down/forward at
# the table. All must pass safety.check_angles — covered by tests/test_poses_observe.py.
# These are conservative defaults that lie within the arm's natural folded reach;
# they may be tuned later once camera FoV is measured with the real robot.
# TODO(phase2): 実機確認 — Pose angles tuned by inspection; verify FoV with actual wrist cam.
OBSERVE = [0, -30, -60, -30, 0, 0]         # straight forward look-down
OBSERVE_LEFT = [30, -30, -60, -30, 0, 0]   # look-down, base rotated +30°
OBSERVE_RIGHT = [-30, -30, -60, -30, 0, 0] # look-down, base rotated -30°
OBSERVE_HIGH = [0, -10, -40, -40, 0, 0]    # higher vantage, shallower angle
