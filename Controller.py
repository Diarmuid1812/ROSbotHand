#!/usr/bin/python
import rospy
from hand_control import *
from Rasia_Prototyp.msg import grip

grips = {
    1: {'Thumb': [70, 0, -40, 10],
        'Finger1': [0, 20, 30],
        'Finger2': [0, 32, 30],
        'Finger3': [0, 37, 30], },
    2 : {'Thumb': [90, 0, -35, 10],
        'Finger1': [0, 60, 30],
        'Finger2': [0, 32, 30],
        'Finger3': [0, 37, 30]},
    3 : {'Thumb': [0, 50, 20, 0],
        'Finger1': [0, 60, 20],
        'Finger2': [0, 80, 90],
        'Finger3': [0, 80, 90]},
    4 : {'Thumb': [0, 50, 10, 0],
        'Finger1': [0, 90, 80],
        'Finger2': [0, 90, 80],
        'Finger3': [0, 90, 80]}
}


if __name__ == '__main__':
    print "Start Classifier node"
    rospy.init_node('CONTROLLER_NODE')
    hand_handle = RoboHand(Port('/dev/ttyACM0'))

    def grip_msg_callback(data):
        gr = grips[data.grip_class]
        hand_handle.set_position(gr)

    rospy.Subscriber("CLASSIFIER_NODE", grip, grip_msg_callback, queue_size=1)

    rospy.spin()
