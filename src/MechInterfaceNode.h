#ifndef _MECH_INTERFACE_NODE_H_
#define _MECH_INTERFACE_NODE_H_

#include "ROSNode.h"

//Topics and Services
#include "lunabotics/Teleoperation.h"
#include "nav_msgs/Odometry.h"

namespace lunabotics {

class MechInterfaceNode : ROSNode {
private:
	
	//Publishers
	ros::Publisher publisherTwist;
	ros::Publisher publisherState;
	ros::Publisher publisherICRControl;
	ros::Publisher publisherAllWheelCommon;
	ros::Publisher publisherAllWheel;
	
	//Subscribers
	ros::Subscriber subscriberOdometry;
	ros::Subscriber subscriberTeleoperation;

	//Callback methods
	void callbackOdometry(const nav_msgs::Odometry::ConstPtr &msg);
	void callbackTeleoperation(const lunabotics::Teleoperation::ConstPtr &msg);
	
	void runOnce();
	void parseArgs();
public:
	MechInterfaceNode(int argc, char **argv, std::string name, int frequency);
	~MechInterfaceNode();
	
	
	void run();
};




}



#endif
