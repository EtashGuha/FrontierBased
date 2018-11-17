#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "amcl/map/map.h"
#include <math.h>
#include <boost/foreach.hpp>
#include "nav_msgs/OccupancyGrid.h"
#define forEach BOOST_FOREACH

class TempClass
{
 nav_msgs::OccupancyGrid map;
 ros::ServiceClient serviceClient;
	public: 
 		void init(){
 		ROS_INFO("Calling init function");
 		ros::NodeHandle n; 
		ros::Subscriber sub = n.subscribe("/move_base/global_costmap/costmap", 1000, &TempClass::saveCostmap, this);
 		serviceClient = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan", true);
 		ros::spin();

 		}


map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  
  return map;
}

// TRY FIXING BY ADDING CONSTANT POINTER!!!!!!!!!! ::ConstPtr& 
void saveCostmap(nav_msgs::OccupancyGrid grid){
	
	map = grid;
	this->printPlan();
}

map_t* convertMap(const nav_msgs::OccupancyGrid map_msg){
	map_t* map = map_alloc();
  	ROS_ASSERT(map);

  	map->size_x = map_msg.info.width;
  	map->size_y = map_msg.info.height;
  	map->scale = map_msg.info.resolution;
  	map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  	map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  	// Convert to player format
  	map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  	ROS_ASSERT(map->cells);
  	for(int i=0;i<map->size_x * map->size_y;i++) {
  	  	if(map_msg.data[i] == 0)
   	  		map->cells[i].occ_state = -1;
    	else if(map_msg.data[i] == 100)
      		map->cells[i].occ_state = +1;
   	 	else
      		map->cells[i].occ_state = 0;
  	}
	return map;
}

double map_calc_range(map_t *map, double ox, double oy, double oa, double max_range)
{
  // Bresenham raytracing
  int x0,x1,y0,y1;
  int x,y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = MAP_GXWX(map,ox);
  y0 = MAP_GYWY(map,oy);
  
  x1 = MAP_GXWX(map,ox + max_range * cos(oa));
  y1 = MAP_GYWY(map,oy + max_range * sin(oa));

  if(abs(y1-y0) > abs(x1-x0))
    steep = 1;
  else
    steep = 0;

  if(steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1-x0);
  deltay = abs(y1-y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if(x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if(y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if(steep)
  {
    if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
  }
  else
  {
    if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
  }

  while(x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if(2*error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if(steep)
    {
      if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
    }
    else
    {
      if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
    }
  }
  return max_range;
}	


void callPlanningService(ros::ServiceClient &serviceClient, nav_msgs::GetPlan &srv){
	 // Perform the actual path planner call

	if (serviceClient.call(srv)) {
		if (!srv.response.plan.poses.empty()) {
			//double entropy = 0;
			forEach(const geometry_msgs::PoseStamped &p, srv.response.plan.poses) {
				map_t* tMap = new map_t();
				tMap = this->convertMap(map);
				for(int angle = 0; angle < 360; angle++){
					printf("Angle: %d, Range: %d\n",angle, map_calc_range(tMap, (double)p.pose.position.x, (double)p.pose.position.y, (3.14/180) * (double)angle, 50.0));
				}
			}
		}

		else {
			ROS_WARN("Got empty planer");
		}
	}
	else {
		ROS_ERROR("Failed to call service %s - is the robot moving?", serviceClient.getService().c_str());
	}
}


void printPlan(){
	nav_msgs::GetPlan getPlan;


    geometry_msgs::PoseStamped Goal;
    Goal.header.seq = 0;
    Goal.header.frame_id = "map";
    Goal.pose.position.x = 7.0;
    Goal.pose.position.y = -2.0;
    Goal.pose.position.z = 0.0;
    Goal.pose.orientation.x = 0.0;
    Goal.pose.orientation.y = 0.0;
    Goal.pose.orientation.w = 1.0;
    getPlan.request.goal = Goal;
    getPlan.request.tolerance = 1.5;

	ros::NodeHandle nh;
	//serviceClient as member and put initialization in init function 
	
	callPlanningService(serviceClient, getPlan);
}

};

int main(int argc, char **argv){
	ros::init(argc, argv, "PoseGenerator");
	ROS_INFO("Initialization of TempClass");
	TempClass a;
	a.init();
	ros::spin();

	return 0;
}

