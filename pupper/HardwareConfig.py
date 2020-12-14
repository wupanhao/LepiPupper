"""
Per-robot configuration file that is particular to each individual robot, not just the type of robot.
"""
import numpy as np


MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
#[[-64.,  35.,  13.,   8.,],
# [ 52.,  60.,  49.,  87.,],
# [ 23., -45., -33., -25.]]
#[[-65.,  36.,  12.,   8.],
# [ 53.,  61.,  48.,  85.],
# [ 38., -38., -23., -17.]]
[[-65.  ,37.  ,12.   ,8.],
 [ 54.  ,61.  ,47.  ,88.],
 [ -9. ,-76. ,-62. ,-55.]]
)

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}
