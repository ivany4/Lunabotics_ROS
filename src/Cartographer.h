#ifndef _CARTOGRAPHER_H_
#define _CARTOGRAPHER_H_

#include "ROSNode.h"

//Topics and Services
#include "lunabotics/Vision.h"
#include "lunabotics/State.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

namespace lunabotics {

class Cartographer : ROSNode {
private:
	int _sequence;
	bool _mapFromFile;
	std::string _mapFilename;
	
	//Subscriber
	ros::Subscriber _subscriberVision;
	ros::Subscriber _subscriberState;

	//Services
	ros::ServiceClient _clientMap;
	ros::ServiceServer _serverMap;
	nav_msgs::GetMap _serviceMap;

	//Callback methods
	void callbackVision(const lunabotics::Vision::ConstPtr &msg);
	void callbackState(const lunabotics::State::ConstPtr &msg);
	bool callbackGetMap(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
	
	nav_msgs::OccupancyGrid readMapFromFile(std::string filename);
	
	void runOnce();
	void parseArgs();
public:
	Cartographer(int argc, char **argv, std::string name, int frequency);
	~Cartographer();
	
	
	void run();
};




}



#endif
