#ifndef _NODE_ASSISTANT_H_
#define _NODE_ASSISTANT_H_

#include "ros/ros.h"
#include "topics.h"

namespace lunabotics {

class ROSNode {
	protected:
		bool isDiffDriveRobot;
		ros::NodeHandle *nodeHandle;
		ros::Subscriber sub;
		int spinFrequency;
		int argc;
		char **argv;
		
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
