#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"


void printCostmap(nav_msgs::OccupancyGrid grid){
	ROS_INFO("YOU GOT ME");
	for(int i = 0; i < grid.data.size(); i++){
		printf("%d", grid.data.at(i));
	}
}

int main(int argc, char **argv){
	ROS_INFO("HELLO");
	ros::init(argc, argv, "ScanGenerator");
	ros::NodeHandle n; 
	ros::Subscriber sub = n.subscribe("/move_base/local_costmap/costmap", 1000, printCostmap);
	ros::spin();

	return 0; 
}


