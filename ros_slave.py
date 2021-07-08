import rospy
import numpy as np
from pupper.HardwareInterface import HardwareInterface
from sensor_msgs.msg import JointState

hardware_interface = HardwareInterface()


def toConfig(angles):
    states = np.zeros((3, 4))
    for i in range(4):
        for j in range(3):
            states[j][i] = angles[i*3+j]
    return states


def cbJointState(msg):
    print(msg)
    states = toConfig(msg.position)
    hardware_interface.set_actuator_postions(states)


if __name__ == '__main__':
    rospy.init_node('pupper_node', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, cbJointState)
    rospy.spin()
