#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import math

def MapMaker():
	rospy.init_node("MapMaker")
	sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, filterCells)
	rospy.spin()

def  filterCells(data):
	rospy.loginfo(data.data)

if __name__ == '__main__':
	MapMaker()