#include "ros/ros.h"
#include "lunabotics/Vision.h"
#include "lunabotics/Telemetry.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include <fstream>

#define MAP_FROM_FILE	1


int seq = 0;

inline int randomNumber(int min, int max)
{
	return min + (rand() % (int)(max - min + 1));
}

nav_msgs::OccupancyGrid getMapFromFile()
{
	std::ifstream infile("world.txt");
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
	map.info.map_load_time = ros::Time::now();
	return map;
}

bool getMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{	
	ROS_INFO("Got a map request");
	
#if MAP_FROM_FILE
	res.map = getMapFromFile();
#else
	//read from Anuraj
	
	//for (unsigned int i = 0; i < res.map.info.height; i++) {
		//for (unsigned int j = 0; j < res.map.info.width; j++) {
			//int8_t occupancy = ((i+j)*4)%75;
			//if ((i > 2 && i < 6 && j > 4 && j < 8) || (i > 5 && i < 9 && j > 8 && j < 12)) {
				//occupancy = 74+i+j;
			//}
				
			//res.map.data.push_back(occupancy);
		//}
	//}
#endif
		
	res.map.header.seq = seq;
	res.map.header.frame_id = 1;
	res.map.header.stamp = ros::Time::now();
	
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
	ros::Subscriber visionSubscriber = nodeHandle.subscribe("lunabotics/vision", 256, visionCallback);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("lunabotics/telemetry", 256, telemetryCallback);
	ros::ServiceServer mapServer = nodeHandle.advertiseService("lunabotics/map", getMap);
	
	ROS_INFO("SLAM ready"); 

	ros::spin();

	return 0;
}
