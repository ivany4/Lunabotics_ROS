#include "ros/ros.h"
#include "lunabotics/Vision.h"
#include "lunabotics/Telemetry.h"
#include "nav_msgs/GetMap.h"

bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
	//Return map
	//res.map = 
	ROS_INFO("Got a map request. Sending map back.");
	return true;
}

void visionCallback(const lunabotics::Vision& msg)
{
	//Update map from new data
}

void telemetryCallback(const lunabotics::Telemetry& msg)
{
	//Update pose from new telemetry
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_slam");
	ros::NodeHandle nodeHandle;
	ros::Subscriber visionSubscriber = nodeHandle.subscribe("luna_vision", 256, visionCallback);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	ros::ServiceServer mapServer = nodeHandle.advertiseService("luna_map", getMap);
	
	ROS_INFO("SLAM ready"); 

	ros::spin();

	return 0;
}
