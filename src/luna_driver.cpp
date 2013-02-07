#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "lunabotics/BoolValue.h"
#include "lunabotics/Emergency.h"
#include "lunabotics/Control.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include "planning/a_star_graph.h"
using namespace std;

int seq = 0;
nav_msgs::GetMap mapService;
ros::ServiceClient mapClient;

vector<a_star_node> getPath(geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, float &res)
{
	vector<a_star_node> graph;
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
		float resolution = mapService.response.map.info.resolution;
		res = resolution;
		
		int start_x = round(startPose.position.x/resolution);
		int start_y = round(startPose.position.y/resolution);
		int goal_x = round(goalPose.position.x/resolution);
		int goal_y = round(goalPose.position.y/resolution);
		
		a_star_graph pathPlan;
		graph = pathPlan.find_path(mapService.response.map.data, 
								mapService.response.map.info.width, 
								mapService.response.map.info.height,
								start_x, start_y, goal_x, goal_y);
		
		if (graph.size() == 0) {
			ROS_INFO("Path is not found");
		}
		else {
			ROS_INFO("Path found");
		}
		
			
		seq++;
	}
	else {
		ROS_ERROR("Failed to call service luna_map");
	}
	return graph;
}

void emergencyCallback(const lunabotics::Emergency& msg)
{
	//Use msg to stop driving if applicable
}

void autonomyCallback(const lunabotics::BoolValue& msg)
{
	//Use msg to toggle autonomy
	if (msg.flag) {
			
		//Specify params
		geometry_msgs::Pose start;
		geometry_msgs::Pose goal;
		start.position.x = 0.1;
		start.position.y = 0.1;
		start.position.z = 0;
		goal.position.x = 0.4;
		goal.position.y = 0.4;
		goal.position.z = 0;
		
		ROS_INFO("Requesting path between (%.1f,%.1f) and (%.1f,%.1f)",
				  start.position.x, start.position.y,
				  goal.position.x, goal.position.y);
		float resolution;
		vector<a_star_node> path = getPath(start, goal, resolution);
		
		stringstream sstr;
		for (vector<a_star_node>::iterator it = path.begin(); it != path.end(); it++) {
			a_star_node node = *it;
			sstr << "->(" << node.x*resolution << "," << node.y*resolution << ")";
		}
		ROS_INFO("Returned path: %s", sstr.str().c_str());
		
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_driver");
	ros::NodeHandle nodeHandle;
	ros::Subscriber emergencySubscriber = nodeHandle.subscribe("luna_alert", 256, emergencyCallback);
	ros::Subscriber autonomySubscriber = nodeHandle.subscribe("luna_auto", 256, autonomyCallback);
	ros::Publisher controlPublisher = nodeHandle.advertise<lunabotics::Control>("luna_ctrl", 256);
	mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("luna_map");
	
	
	
	ROS_INFO("Driver ready"); 
	
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		//Whenever needed send control message
		if (false) {
			lunabotics::Control controlMsg;
			controlMsg.motion.linear.x = 0;
			controlPublisher.publish(controlMsg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
