#!/usr/bin/python
import pickle

from Rasia_Prototyp.msg import zeros, grip
import rospy
from hand_control import *


class Classifier:

	def __init__(self):
		with open('/home/pi/catkin_ws/src/Rasia_Prototyp/modelPZEZMS.pkl', 'rb') as f:
			self._clf = pickle.load(f)
		self._publisher = rospy.Publisher("CLASSIFIER_NODE", grip, queue_size=1)

	def predict(self, data):
		zeros_list = list(data.samples)
		decision = self._clf.predict([zeros_list])

		self._publisher.publish(decision)


if __name__ == "__main__":
	rospy.init_node('Classifier_node')
	print "Start Controller node"

	classifier = Classifier()
	rospy.Subscriber("ADC_NODE", zeros, classifier.predict)

	rate = rospy.Rate(1000)  # 10hz
	rospy.spin()
