#include "MechInterfaceNode.h"

#include "lunabotics/State.h"
#include "lunabotics/ICRControl.h"
#include "lunabotics/AllWheelCommon.h"
#include "lunabotics/AllWheelState.h"
#include "geometry_msgs/Twist.h"

#include "../protos_gen/AllWheelControl.pb.h"

using namespace lunabotics;

MechInterfaceNode::MechInterfaceNode(int argc, char **argv, std::string name, int frequency):
ROSNode(argc, argv, name, frequency)
{
	//Create publishers
	this->publisherTwist = this->nodeHandle->advertise<geometry_msgs::Twist>(TOPIC_CMD_TWIST, 256);
	this->publisherState = this->nodeHandle->advertise<lunabotics::State>(TOPIC_TM_ROBOT_STATE, 256);
	this->publisherICRControl = this->nodeHandle->advertise<lunabotics::ICRControl>(TOPIC_CMD_ICR, sizeof(float)*3);
	this->publisherAllWheelCommon = this->nodeHandle->advertise<lunabotics::AllWheelCommon>(TOPIC_CMD_ALL_WHEEL, 256);
	this->publisherAllWheel = this->nodeHandle->advertise<lunabotics::AllWheelState>(TOPIC_CMD_EXPLICIT_ALL_WHEEL, 256);
	
	//Create subscribers
	this->subscriberOdometry = this->nodeHandle->subscribe(TOPIC_TM_ODOMETRY, 256, &MechInterfaceNode::callbackOdometry, this);
	this->subscriberTeleoperation = this->nodeHandle->subscribe(TOPIC_CMD_TELEOP, 256, &MechInterfaceNode::callbackTeleoperation, this);
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
	
	if (this->isDiffDriveRobot) {
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
	else {
		if ((!msg->left && !msg->right) || (msg->left && msg->right)) {
			lunabotics::AllWheelState controlMsg;
			controlMsg.steering.left_front = 0;
			controlMsg.steering.right_front = 0;
			controlMsg.steering.left_rear = 0;
			controlMsg.steering.right_rear = 0;
			if ((msg->forward || msg->backward) && !(msg->forward && msg->backward)) {
				//Predefined longitudal
				ROS_INFO("TELEOP: Longitudal");
				controlMsg.driving.left_front = 3;
				controlMsg.driving.right_front = 3;
				controlMsg.driving.left_rear = 3;
				controlMsg.driving.right_rear = 3;
			}
			else {
				ROS_INFO("TELEOP: Stop");
				controlMsg.driving.left_front = 0;
				controlMsg.driving.right_front = 0;
				controlMsg.driving.left_rear = 0;
				controlMsg.driving.right_rear = 0;
			}
			this->publisherAllWheel.publish(controlMsg);
			
		}
		else if (!msg->forward && !msg->backward) {
			//Predefined point-turn
			ROS_INFO("TELEOP: Point-turn");
			lunabotics::AllWheelCommon controlMsg;
			controlMsg.predefined_cmd = msg->left ? lunabotics::proto::AllWheelControl::TURN_CCW : lunabotics::proto::AllWheelControl::TURN_CW;
			this->publisherAllWheelCommon.publish(controlMsg);
		}
		else if (msg->forward && msg->backward) {
			//Predefined crab
			ROS_INFO("TELEOP: Crab");
			lunabotics::AllWheelCommon controlMsg;
			controlMsg.predefined_cmd = msg->left ? lunabotics::proto::AllWheelControl::CRAB_LEFT : lunabotics::proto::AllWheelControl::CRAB_RIGHT;
			this->publisherAllWheelCommon.publish(controlMsg);
		}
		else {
			//ICR
			ROS_INFO("TELEOP: ICR");
			lunabotics::ICRControl controlMsg;
			controlMsg.ICR.x = 0;
			controlMsg.ICR.y = msg->left ? 1 : -1;
			controlMsg.velocity = 0.3;
			this->publisherICRControl.publish(controlMsg);
		}
	}
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
