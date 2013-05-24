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
	int sequence;
	bool mapFromFile;
	std::string mapFilename;
	
	//Subscriber
	ros::Subscriber subscriberVision;
	ros::Subscriber subscriberState;

	//Services
	ros::ServiceClient clientMap;
	ros::ServiceServer serverMap;
	nav_msgs::GetMap serviceMap;

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
