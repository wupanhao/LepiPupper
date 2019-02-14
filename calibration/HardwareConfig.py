
import numpy as np

MICROS_PER_RAD = 11.333 * 180.0 / np.pi  # Must be calibrated
NEUTRAL_ANGLE_DEGREES = np.array(
[[56, 39, 82, 115], [11, 57, 89, 128], [28, 70, 105, 140]]
,dtype = 'float')

PS4_COLOR = {"red": 0, "blue": 0, "green": 255}
PS4_DEACTIVATED_COLOR = {"red": 0, "blue": 0, "green": 50}
