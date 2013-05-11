#include "NodeAssistant.h"

using namespace lunabotics;

NodeAssistant::NodeAssistant(int argc, char **argv, std::string name, int frequency)
{
	ROS_INFO("Instantiatin the base");
	this->_isDiffDriveRobot = this->commandOptionExists(argv, argv + argc, "-D");
	this->_spinFrequency = frequency;
	ros::init(argc, argv, name);
	this->_nodeHandle = new ros::NodeHandle("lunabotics");
}

NodeAssistant::~NodeAssistant()
{
	delete this->_nodeHandle;
}

void NodeAssistant::callbackGoal(const lunabotics::Goal::ConstPtr &msg)
{
	ROS_INFO("GOAL!");
}


char *NodeAssistant::getCommandOption(char **begin, char **end, const std::string &option)
{
    char **it = std::find(begin, end, option);
    if (it != end && ++it != end){
        return *it;
    }
    return 0;
}

bool NodeAssistant::commandOptionExists(char **begin, char **end, const std::string &option)
{
    return std::find(begin, end, option) != end;
}

void NodeAssistant::run()
{
	ROS_INFO("Running");
	ros::Rate loop_rate(this->_spinFrequency);
	while (ros::ok()) {
		
		this->exec();
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
