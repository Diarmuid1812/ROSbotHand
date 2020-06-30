#!/usr/bin/python3.7
import pickle

from rosbotic_hand.msg import grip
import rospy
from hand_control import *
from std_msgs.msg import String
import numpy as np


class Classifier:

    def __init__(self):
        with open('/home/pi/catkin_ws/src/ROSbotHand/modelPZEZMS.pkl', 'rb') as f:
            self._clf = pickle.load(f)
        self._publisher = rospy.Publisher("CLASSIFIER_NODE", grip, queue_size=1)

    def predict(self, data):
        array_predict = np.fromstring(data.data, dtype=np.float64, sep=' ').reshape(1, -1)
        decision = self._clf.predict(array_predict)

        self._publisher.publish(decision)


if __name__ == "__main__":
    rospy.init_node('Classifier_node')
    print("Start Controller node")

    classifier = Classifier()
    rospy.Subscriber("STFTtalker", String, classifier.predict)

    rate = rospy.Rate(1000)  # 10hz
    rospy.spin()
