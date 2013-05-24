#include "MechInterfaceNode.h"

using namespace lunabotics;

MechInterfaceNode::MechInterfaceNode(int argc, char **argv, std::string name, int frequency):
ROSNode(argc, argv, name, frequency)
{
	//Create publishers
	this->publisherTwist = this->nodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel", 256);
	this->publisherState = this->nodeHandle->advertise<lunabotics::State>("state", 256);
	
	//Create subscribers
	this->subscriberOdometry = this->nodeHandle->subscribe("/odom", 256, &MechInterfaceNode::callbackOdometry, this);
	this->subscriberTeleoperation = this->nodeHandle->subscribe("control", 256, &MechInterfaceNode::callbackTeleoperation, this);
	
	
}

MechInterfaceNode::~MechInterfaceNode()
{
}

//---------------------- CALLBACK METHODS ------------------------------

void MechInterfaceNode::callbackOdometry(const nav_msgs::Odometry::ConstPtr &msg)
{
	lunabotics::State stateMsg;
	stateMsg.odometry = *msg;
	this->publisherState.publish(stateMsg);
}

void MechInterfaceNode::callbackTeleoperation(const lunabotics::Teleoperation::ConstPtr &msg)
{
	ROS_INFO("%s%s%s%s", msg->forward ? "^" : "", msg->backward ? "v" : "", msg->left ? "<" : "", msg->right ? ">" : "");
	
	float v = 0;
	if (msg->forward && !msg->backward) {
		v = 5.0;
	}
	else if (!msg->forward && msg->backward) {
		v = -3.0;
	}
	
	float w = 0;
	if (msg->left && !msg->right) {
		w = 1.0;
	}
	else if (!msg->left && msg->right) {
		w = -1.0;
	}
	
	geometry_msgs::Twist controlMsg;
	controlMsg.linear.x = v;
	controlMsg.linear.y = 0;
	controlMsg.linear.z = 0;
	controlMsg.angular.x = 0;
	controlMsg.angular.y = 0;
	controlMsg.angular.z = w;
	this->publisherTwist.publish(controlMsg);
}
				


//------------------ INHERITED METHODS ----------------------------------

void MechInterfaceNode::runOnce()
{
}

void MechInterfaceNode::run()
{
	//Let the parent to the trick
	ROSNode::run();
}

//----------------------- MAIN METHOD ----------------------------------

int main(int argc, char **argv)
{
	lunabotics::MechInterfaceNode *node = new lunabotics::MechInterfaceNode(argc, argv, "luna_mech_gw", 0);
	node->run();
	delete node;
	return 0;
}
