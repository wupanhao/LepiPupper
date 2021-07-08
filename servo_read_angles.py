from pupper.HardwareConfig import NEUTRAL_ANGLE_DEGREES
from sservo import SServo

servo = SServo()

ids = servo.scan()
print(ids)
for i in ids:
    print(servo.get_position(i))
