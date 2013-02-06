#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "lunabotics/Emergency.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/GetPlan.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include "planning/a_star_graph.h"
using namespace std;

int seq = 0;
nav_msgs::GetMap mapService;
ros::ServiceClient mapClient;

bool getPath(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
{
	ROS_INFO("Got a path request with tolerance %.2f", req.tolerance);
	bool result = true;
	if (mapClient.call(mapService)) {
		ROS_INFO("Got map from service (%ld nodes)", mapService.response.map.data.size());
		ROS_INFO("------------------------------------");
		for (unsigned int i = 0; i < mapService.response.map.info.height; i++) {
		    stringstream sstr;
			for (unsigned int j = 0; j < mapService.response.map.info.width; j++) {
				int8_t probability = mapService.response.map.data.at(i*mapService.response.map.info.width+j);
				sstr << static_cast<int>(probability) << " ";
			}
			ROS_INFO("%s", sstr.str().c_str());
		}
		ROS_INFO("\n");
		
		ROS_INFO("Looking for a path...");
		
		//Generate a path
		a_star_graph pathPlan;
		float resolution = mapService.response.map.info.resolution;
		
		int start_x = round(req.start.pose.position.x/resolution);
		int start_y = round(req.start.pose.position.y/resolution);
		int goal_x = round(req.goal.pose.position.x/resolution);
		int goal_y = round(req.goal.pose.position.y/resolution);
		
		vector<a_star_node> graph = pathPlan.find_path(mapService.response.map.data, 
												mapService.response.map.info.width, 
												mapService.response.map.info.height,
												start_x, start_y, goal_x, goal_y);
		
		if (graph.size() == 0) {
			ROS_INFO("Path is not found");
		}
		else {
			ROS_INFO("Path found");
		}
		
		ros::Time now = ros::Time::now();
		
		res.plan.header.seq = seq;
		res.plan.header.frame_id = 1;
		res.plan.header.stamp = now;
		
		
		//Static path
		for (int i = 0; i < (int)graph.size(); i++) {
			a_star_node graphNode = graph.at(i);
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.header.seq = i;
			poseStamped.header.frame_id = 1;	//Global frame
			poseStamped.header.stamp = now;
			poseStamped.pose.position.x = graphNode.x;
			poseStamped.pose.position.y = graphNode.y;
			poseStamped.pose.position.z = 0;
			poseStamped.pose.orientation.x = 0.5;
			poseStamped.pose.orientation.y = 0.1;
			poseStamped.pose.orientation.z = 0;
			poseStamped.pose.orientation.w = 1;
			
			res.plan.poses.push_back(poseStamped);
		}
			
		seq++;
	}
	else {
		result = false;
		ROS_ERROR("Failed to call service luna_map");
	}
	return result;
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
	ros::ServiceServer pathServer = nodeHandle.advertiseService("luna_path", getPath);
	mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("luna_map");
	
	
	
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
