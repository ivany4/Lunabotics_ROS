#ifndef _NODE_ASSISTANT_H_
#define _NODE_ASSISTANT_H_

#include "ros/ros.h"
namespace lunabotics {

class ROSNode {
	protected:
		bool _isDiffDriveRobot;
		ros::NodeHandle *_nodeHandle;
		ros::Subscriber sub;
		int _spinFrequency;
		int _argc;
		char **_argv;
		
		char *getCommandOption(char **begin, char **end, const std::string &option);
		bool commandOptionExists(char **begin, char **end, const std::string &option);
	
		virtual void runOnce() = 0;
		void parseArgs(int argc, char **argv);
	public:
		ROSNode(int argc, char **argv, std::string name, int frequency);
		virtual ~ROSNode();
		
		virtual void run();
};



}


#endif
