#include "ros/ros.h"
#include "lunabotics/Vision.h"
#include "lunabotics/Telemetry.h"
#include "nav_msgs/GetMap.h"

int seq = 0;

inline int randomNumber(int min, int max)
{
	return min + (rand() % (int)(max - min + 1));
}

bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{	
	ROS_INFO("Got a map request");
	ros::Time now = ros::Time::now();
	res.map.header.seq = seq;
	res.map.header.frame_id = 1;
	res.map.header.stamp = now;
	res.map.info.width = 10;
	res.map.info.height = 10;
	res.map.info.resolution = 10/10.0;
	res.map.info.map_load_time = now;
	res.map.info.origin.position.x = 0;
	res.map.info.origin.position.y = 0;
	res.map.info.origin.position.z = 0;
	res.map.info.origin.orientation.x = 0;
	res.map.info.origin.orientation.y = 0;
	res.map.info.origin.orientation.z = 0;
	res.map.info.origin.orientation.w = 0;
	
	for (unsigned int i = 0; i < res.map.info.height; i++) {
		for (unsigned int j = 0; j < res.map.info.width; j++) {
			int8_t occupancy = 0;
			if (i > 2 && i < 6 && j > 4 && j < 8) {
				occupancy = 100;
			}
				
			res.map.data.push_back(occupancy);
		}
	}
	
	seq++;
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
