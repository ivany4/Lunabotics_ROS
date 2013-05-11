#include "ROSNode.h"
#include <sstream>

using namespace lunabotics;

ROSNode::ROSNode(int argc, char **argv, std::string name, int frequency)
{
	ROS_INFO("Initializing %s", name.c_str());
	this->_argc = 0;
	this->_argv = NULL;
	this->_spinFrequency = frequency;
	if (argc > 1) {
		this->parseArgs(argc, argv);
	}
	ros::init(argc, argv, name);
	this->_nodeHandle = new ros::NodeHandle("lunabotics");
}

ROSNode::~ROSNode()
{
	for (int i = 0; i < this->_argc; i++) {
		delete this->_argv[i];
	}
	if (this->_argv) {
		delete this->_argv;
	}
	delete this->_nodeHandle;
}

char *ROSNode::getCommandOption(char **begin, char **end, const std::string &option)
{
    char **it = std::find(begin, end, option);
    if (it != end && ++it != end){
        return *it;
    }
    return 0;
}

bool ROSNode::commandOptionExists(char **begin, char **end, const std::string &option)
{
    return std::find(begin, end, option) != end;
}

void ROSNode::run()
{
	ROS_INFO("Spinning with frequency %d Hz", this->_spinFrequency);
	if (this->_spinFrequency > 0) {
		
		ros::Rate loop_rate(this->_spinFrequency);
		while (ros::ok()) {
			
			this->runOnce();
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else {
		ros::spin();
	}
}

void ROSNode::parseArgs(int argc, char **argv)
{
	ROS_INFO("Saving %d command line argument(s)", argc-1); //ROS init erases them, so need to copy
	
	//Not saving first argument (command name)
    std::stringstream sstr;
	this->_argc = argc-1;
	this->_argv = new char *[argc-1];
	for (int i = 1; i < argc; i++) {
		int len = strlen(argv[i]);
		this->_argv[i-1] = new char[len];
		strcpy(this->_argv[i-1], argv[i]);
		sstr << argv[i];
	}
	
	ROS_INFO("Parsing command line argument(s): %s", sstr.str().c_str());
	
	this->_isDiffDriveRobot = this->commandOptionExists(this->_argv, this->_argv + this->_argc, "-D");
	if (this->_isDiffDriveRobot) {
		ROS_INFO("Configuration: Diffrential Drive");
	}
	else {
		ROS_INFO("Configuration: All-wheeler");
	}
}
