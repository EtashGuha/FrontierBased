#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import math

def filterCells(data):
	rospy.loginfo("working")
	entropy = 0
	for x in range(0, len(data.data)):
		if data.data[x] != 0 and data.data[x] != 100:
			probability = data.data[x]/100.0
			entropy = entropy - (probability * math.log10(probability) + ((1 - probability) * math.log10(1 - probability)))
	entropy_str = "Entropy: " + str(entropy)
	rospy.loginfo(entropy_str)
	
def Mapper():
	rospy.init_node('Mapper')
	rospy.loginfo("hello world")	
	sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, filterCells)
	rospy.spin()

if __name__ == '__main__':
	Mapper()
		
