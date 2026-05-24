"""Single source of truth for tunable constants.

Imported by safety, client, planner, hub, server. Keeps the values testable and
serves the same numbers to the frontend via the /kinematics endpoint.
"""
from __future__ import annotations

# --- motion limits ---
MAX_SPEED = 40              # firmware accepts 1-100; we cap at 40 for safety
DEFAULT_SPEED = 25

# --- path planner ---
PATH_STEP_DEG = 8.0         # max joint delta per waypoint (degrees) — for SAFETY check granularity
WAYPOINT_WAIT = 0.15        # initial sleep before readback poll (s)
WAYPOINT_TOLERANCE = 3.0    # angle match tolerance for "reached" (deg)
WAYPOINT_TIMEOUT = 8.0      # per-waypoint readback timeout (s) — bumped from 4 to cover large single-shot motions
ANGLE_DRIFT_TOL = 3.0       # client expected_current drift tolerance (deg)

# Smooth-motion mode: if the joint-space path is monotonic and all intermediate
# waypoints pass safety, send only the final target to firmware (single command)
# and let the firmware interpolate smoothly. Avoids the deceleration/pause
# between every PATH_STEP_DEG chunk. Disabled paths still chunk (paths that
# change direction in joint space — rare for typical IK results).
SMOOTH_SINGLE_SHOT = True
SMOOTH_PROGRESS_NEXT = 0.6  # for chunked paths: send next waypoint when this fraction of progress reached

# --- safety geometry (mm) ---
TABLE_MARGIN = 10.0         # genuine clearance above table top
FK_TOOL_SLOP = 5.0          # residual FK error budget (URDF FK is ~1mm accurate; was 30 with old broken DH)
FLOOR_Z = TABLE_MARGIN + FK_TOOL_SLOP  # min Z for any joint/tool position
LINK_RADIUS = 35.0          # effective half-width of arm links
SELF_CLEARANCE = 105.0      # min centerline distance between non-adjacent (variable) links
# ^ bumped from 60 → 105 after observing real-arm rubbing at the J5 latch poses.
# Variable pairs (excluding geometric invariants 0-2 at 74mm and 3-5 at 95mm)
# at HOME are ≥135mm. Stall poses had 92-119mm. 105 blocks A,B,C; D requires
# the runtime stall detector (the model can't reach D from centerline alone).
DEGENERATE_LINK_MM = 5.0    # below this length, skip a link in collision pairs

# --- arm geometry ---
# The 65.5mm flange offset is now built into the URDF FK (kinematics.py).
# TOOL_LENGTH means "extension beyond the J6 tool flange, along flange +z".
# Set this when a gripper / tool is attached; 0 = bare flange.
TOOL_LENGTH = 0.0

# --- motion stall (collision/latch) detection ---
# Replaces the dead current-based monitor: watches whether commanded joints
# actually progress. When the arm hits something (table, self, joint limit, or
# a latched servo), the angles stop changing — that's the signal.
STALL_WINDOW_S = 0.8        # sliding window of angle readback (s)
STALL_MIN_PROGRESS_DEG = 0.6 # max joint movement in window below which we call stall
STALL_GRACE_S = 0.4         # initial grace period after send_angles (servo accel)
STALL_MIN_REMAINING_DEG = 4.0 # only flag stall if any joint is still > N° from target

# --- current / force monitoring ---
# pymycobot.get_servo_currents() returns per-joint current in mA (range 0~3250 per official docs).
# These defaults are conservative starting points; they MUST be calibrated empirically:
#   1. Move arm gently through expected workspace without external load
#   2. Log peak current per joint (`/log_currents` endpoint or scripts/calibrate_currents.py)
#   3. Set threshold = peak * 1.8~2.0
CURRENT_THRESHOLD_MA = 1500   # per-joint absolute (mA); over this for SUSTAINED_OVER_COUNT polls → abort
CURRENT_POLL_HZ = 10          # monitor polling rate (Hz)
SUSTAINED_OVER_COUNT = 3      # consecutive polls above threshold to trigger (~300ms at 10Hz)

# --- HTTP server ---
DEFAULT_PORT = 8000
DEFAULT_CAM_INDEX = 3
HOME_ANGLES = [0.0, 0.0, -90.0, 0.0, 0.0, 0.0]

# --- camera observation ---
# The wrist camera is physically mounted 90° rotated relative to flange "up"
# (verified empirically: at observe pose with J6=0 the image came out sideways;
# at J6=-90° it's upright). J6 rotates around flange +Z = camera optical axis,
# so changing J6 only rolls the image, not the camera position.
CAMERA_UPRIGHT_J6_DEG = 90.0

# --- cartesian path generation ---
CART_STEP_MM = 30.0                  # max cartesian step size between waypoints
CART_LIFT_Z_MM = 320.0               # lift-translate-lower middle height
CART_LIFT_HORIZ_THRESHOLD_MM = 80.0  # min horizontal distance to trigger lift mode
CART_AUTO_LINEAR_THRESHOLD_MM = 20.0 # under this, lift mode falls through to linear

# --- IK continuity / safety buffer ---
IK_MAX_JOINT_JUMP_DEG = 20.0   # if seed-IK solution jumps more than this, subdivide cartesian segment
IK_MAX_SUBDIVIDE = 4           # max bisection depth before giving up
IK_MAX_JOINT_WAYPOINTS = 200   # global cap on joint waypoints produced (refuse plan if exceeded)
IK_SEED_RETRIES = 3            # try N alternate seeds before declaring IK failure
JOINT_LIMIT_BUFFER_DEG = 3.0   # for IK-result validation only (real limit check unchanged)

# --- pre-flight safety ---
SAFE_MODE_CURRENT_MA = 800     # used until empirical calibration exists
CALIBRATION_MARKER = ".calibrated"  # presence of this file in project root means thresholds are trusted

# --- grasp sequence (Phase 1+2: visual target + approach motion, no gripper actuation) ---
GRASP_APPROACH_OFFSET_MM = 80.0   # pre-grasp height above object center (along approach axis)
GRASP_LIFT_OFFSET_MM = 100.0      # post-grasp lift height
GRASP_APPROACH_SPEED_DEFAULT = 15 # default speed for grasp sequence
GRASP_APPROACH_SPEED_MAX = 25     # hard cap (grasp is intentionally slow)
GRASP_OFFSET_MIN_MM = 20.0        # approach/lift offset lower bound
GRASP_OFFSET_MAX_MM = 300.0       # approach/lift offset upper bound
TARGET_RADIUS_MIN_MM = 5.0
TARGET_RADIUS_MAX_MM = 60.0
TARGET_RADIUS_DEFAULT_MM = 20.0
GRIPPER_TIP_CLEARANCE_MM = 5.0    # extra margin above object surface when no gripper present

# --- vision workspace sanity (base frame, mm) ---
# Used by VisionHub.perceive() to reject localized objects that fall outside
# the arm's plausible reach cylinder/box. Acts as a final safety net even if
# the calibration is wrong.
WORKSPACE_REACH_MAX_MM = 380.0    # sqrt(x²+y²) upper bound (≈ arm physical reach)
WORKSPACE_Z_MAX_MM = 500.0        # upper Z bound for any perceived object
# WORKSPACE_Z_MIN_MM is provided implicitly by FLOOR_Z above.
