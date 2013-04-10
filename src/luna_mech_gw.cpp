#include "ros/ros.h"
#include "lunabotics/State.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/AllWheelSteering.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"

geometry_msgs::Twist twistMsg;
lunabotics::State stateMsg;
bool publishState;

void controlCallback(const lunabotics::Control& msg)
{	
	twistMsg = msg.motion;
}

void odoCallback(const nav_msgs::Odometry& msg)
{
	stateMsg.odometry = msg;
	publishState = true;
}

void allWheelCallback(const lunabotics::AllWheelSteering& msg)
{
	//Do nothing yet
}

void steeringCompleteCallback(const std_msgs::Empty& msg)
{
	ROS_INFO("Sequence complete!");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_mech_gw");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber controlSubscriber = nodeHandle.subscribe("control", 256, controlCallback);
	ros::Subscriber allWheelSubscriber = nodeHandle.subscribe("all_wheel", sizeof(float)*8, allWheelCallback);
	ros::Subscriber steeringCallbackSubscriber = nodeHandle.subscribe("steering_complete", 1, steeringCompleteCallback);
	ros::Publisher statePublisher = nodeHandle.advertise<lunabotics::State>("state", 256);
	
	int twistSize = 256;
	int odomSize = 256;
	
	//This is generic. Used by stageros and gazebo lunabotics nodes
	//To remap for Pioneer use RosAria/cmd_vel and RosAria/pose
	ros::Publisher ctrlPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);	
	ros::Subscriber odoSubscriber = nodeHandle.subscribe("/odom", odomSize, odoCallback);
	
	ros::Rate loop_rate(50);
	stateMsg.odometry.header.frame_id = "base_link";
	publishState = false;
	
	ROS_INFO("MECH Gateway ready"); 
	
	while (ros::ok()) {     
		//ROS_INFO("Publishing twist %f,%f", twistMsg.linear.x, twistMsg.angular.z);
		ctrlPublisher.publish(twistMsg);
		//ROS_INFO("Twist published");
		
		//Whenever needed send telemetry message
		if (publishState) {
			//ROS_INFO("Publishing state");
			publishState = false;
			statePublisher.publish(stateMsg);
			//ROS_INFO("State published");
		}
		
		//ROS_INFO("Spin");
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
