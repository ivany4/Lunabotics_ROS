#include "ros/ros.h"
#include "lunabotics/Vision.h"
#include "lunabotics/State.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <fstream>

#define MAP_FROM_FILE	0

int seq = 0;
ros::ServiceClient mapClient;
nav_msgs::GetMap mapService;

inline int randomNumber(int min, int max)
{
	return min + (rand() % (int)(max - min + 1));
}

nav_msgs::OccupancyGrid getMapFromFile()
{
	std::ifstream infile("/home/ivany4/ROS/pkgs/lunabotics/world.txt");
	std::string line;
	nav_msgs::OccupancyGrid map;
	if (std::getline(infile, line)) {
		std::istringstream iss(line);
		if (iss >> map.info.width >> map.info.height >> map.info.resolution) {
			ROS_INFO("Reading map from file: width %d, height %d, resolution %f", map.info.width, map.info.height, map.info.resolution);
			unsigned int lines = 0;
			while (std::getline(infile, line)) {
				std::istringstream iss(line);
				for (unsigned int i = 0; i < map.info.width; i++) {
					int occupancy;
					if (iss >> occupancy) {
						map.data.push_back(occupancy);
					}
					else {
						ROS_ERROR("Failed to parse file. Unexpected end of the line");
						break;
					}
				}
				lines++;
			}
			if (lines != map.info.height) {
				ROS_ERROR("Failed to parse file. Declared height doesn't match number of lines");
			}
		}
		else {
			ROS_ERROR("Failed to read map metadata");
		}
	}
	else {
		ROS_ERROR("Failed to open file");
	}
	map.info.map_load_time = ros::Time::now();
	return map;
}

bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{	
	ROS_INFO("Got a map request");
	
#if MAP_FROM_FILE
	ROS_INFO("Reading from file");
	res.map = getMapFromFile();
#else
	//Gmapping map
	
	if (mapClient.call(mapService)) {
		res = mapService.response;
	}
	else {
		ROS_WARN("Failed to call /map gmapping");
	}

	//Generated map
	/*
	res.map.info.width = 24;
	res.map.info.height = 21;
	res.map.info.resolution = 0.33;
	for (unsigned int i = 0; i < res.map.info.height; i++) {
		for (unsigned int j = 0; j < res.map.info.width; j++) {
			int8_t occupancy = ((i+j)*4)%75;
			if ((i > 2 && i < 6 && j > 4 && j < 8) || (i > 5 && i < 9 && j > 8 && j < 12)) {
				occupancy = 74+i+j;
			}
				
			res.map.data.push_back(occupancy);
		}
	}
	*/
#endif
		
	res.map.header.seq = seq;
	res.map.header.frame_id = "map";
	res.map.header.stamp = ros::Time::now();
	
	seq++;
	return true;
}

void visionCallback(const lunabotics::Vision& msg)
{
	//Update map from new data
}

void stateCallback(const lunabotics::State& msg)
{
	//Update pose from new telemetry
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_slam");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber visionSubscriber = nodeHandle.subscribe("vision", 256, visionCallback);
	ros::Subscriber stateSubscriber = nodeHandle.subscribe("state", 256, stateCallback);
	ros::ServiceServer mapServer = nodeHandle.advertiseService("map", getMap);
	mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("/dynamic_map");
	
	ROS_INFO("SLAM ready"); 

	ros::spin();

	return 0;
}
