#include "Cartographer.h"
#include <fstream>

using namespace lunabotics;

inline int randomNumber(int min, int max)
{
	return min + (rand() % (int)(max - min + 1));
}

Cartographer::Cartographer(int argc, char **argv, std::string name, int frequency):
ROSNode(argc, argv, name, frequency),
_sequence(0), _mapFromFile(false), _mapFilename()
{	
	this->parseArgs();
	
	//Subscribers
	this->_subscriberVision = this->_nodeHandle->subscribe("vision", 256, &Cartographer::callbackVision, this);
	this->_subscriberState = this->_nodeHandle->subscribe("state", 1, &Cartographer::callbackState, this);
	
	//Services
	this->_serverMap = this->_nodeHandle->advertiseService("map", &Cartographer::callbackGetMap, this);
	this->_clientMap = this->_nodeHandle->serviceClient<nav_msgs::GetMap>("/dynamic_map");
}

Cartographer::~Cartographer()
{
}

//---------------------- CALLBACK METHODS ------------------------------

void Cartographer::callbackVision(const lunabotics::Vision::ConstPtr &msg)
{
	//Update map from new data
}

void Cartographer::callbackState(const lunabotics::State::ConstPtr &msg)
{
	//Update pose from new telemetry
}

bool Cartographer::callbackGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{	
	ROS_INFO("Got a map request");
	
	if (this->_mapFromFile) {
		ROS_INFO("Reading from file");
		res.map = this->readMapFromFile(this->_mapFilename);
	}
	else {
		//Gmapping map
		
		if (this->_clientMap.call(this->_serviceMap)) {
			res = this->_serviceMap.response;
		}
		else {
			ROS_WARN("Failed to call /dynamic_map gmapping");
		}
	}
		
	res.map.header.seq = this->_sequence++;
	res.map.header.frame_id = "map";
	res.map.header.stamp = ros::Time::now();
	
	return true;
}

//----------------------- MAPPING METHODS ------------------------------


nav_msgs::OccupancyGrid Cartographer::readMapFromFile(std::string filename)
{
	std::ifstream infile(filename.c_str());
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


//------------------ INHERITED METHODS ----------------------------------

void Cartographer::runOnce()
{
}

void Cartographer::run()
{
	//Let the parent to the trick
	ROSNode::run();
}

void Cartographer::parseArgs()
{
	if (this->commandOptionExists(this->_argv, this->_argv + this->_argc, "-f")) {
		this->_mapFromFile = true;
		this->_mapFilename = std::string(this->getCommandOption(this->_argv, this->_argv + this->_argc, "-f"));
		ROS_INFO("World map file: %s", this->_mapFilename.c_str());
	}
}

//----------------------- MAIN METHOD ----------------------------------

int main(int argc, char **argv)
{
	lunabotics::Cartographer *node = new lunabotics::Cartographer(argc, argv, "luna_slam", 0);
	node->run();
	delete node;
	return 0;
}
