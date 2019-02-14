
import numpy as np

MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
[[-66.0, 37.0, 11.0, 5.0], [67.0, 104.0, 47.0, 86.0], [-36.0, -107.0, -95.0, -84.0]]
,dtype = 'float')

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}
