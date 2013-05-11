#ifndef _NODE_ASSISTANT_H_
#define _NODE_ASSISTANT_H_

#include "ros/ros.h"
#include "lunabotics/Goal.h"

namespace lunabotics {

class NodeAssistant {
	protected:
		bool _isDiffDriveRobot;
		ros::NodeHandle *_nodeHandle;
		ros::Subscriber sub;
		int _spinFrequency;
		
		char *getCommandOption(char **begin, char **end, const std::string &option);
		bool commandOptionExists(char **begin, char **end, const std::string &option);
		
		void callbackGoal(const lunabotics::Goal::ConstPtr &msg);
	
	public:
		NodeAssistant(int argc, char **argv, std::string name, int frequency);
		virtual ~NodeAssistant();
		
		virtual void run();
		
		virtual void exec() = 0;
};



}


#endif
