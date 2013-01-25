#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "lunabotics/Emergency.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>


bool getPath(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
{
	//Use req.start as a start pose and req.goal as a goal pose
	//And req.tolerance if the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing. 
	//Return path
	//res.plan = 
	ROS_INFO("Got a path request with tolerance %.2f", req.tolerance);
	return true;
}

void emergencyCallback(const lunabotics::Emergency& msg)
{
	//Use msg to stop driving if applicable
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_navigator");
	ros::NodeHandle nodeHandle;
	ros::Subscriber emergencySubscriber = nodeHandle.subscribe("luna_alert", 256, emergencyCallback);
	ros::ServiceClient mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("luna_map");
	ros::ServiceServer pathServer = nodeHandle.advertiseService("luna_path", getPath);
	nav_msgs::GetMap mapService;
	
	
	ROS_INFO("Navigator ready"); 
	
	
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		//Whenever needed to get a map
		if (false) {
			if (mapClient.call(mapService)) {
				//Use map
				//srv.response.map
				ROS_INFO("Got map from service");
			}
			else {
				ROS_ERROR("Failed to call service luna_map");
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	return 0;
}
