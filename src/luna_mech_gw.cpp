#include "ros/ros.h"
#include "lunabotics/State.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "nav_msgs/Odometry.h"

geometry_msgs::Twist twistMsg;
lunabotics::State stateMsg;
bool publishState;

void controlCallback(const lunabotics::Control& msg)
{	
	twistMsg.linear.x = msg.motion.linear.x;
	twistMsg.angular.z = msg.motion.angular.z;
}

void odoCallback(const nav_msgs::Odometry& msg)
{
	stateMsg.odometry = msg;
	publishState = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_mech_gw");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber controlSubscriber = nodeHandle.subscribe("control", 256, controlCallback);
	ros::Publisher statePublisher = nodeHandle.advertise<lunabotics::State>("state", 256);
	
	int twistSize = 256;
	int odomSize = 256;
	
	//This is generic. Used by stageros and gazebo lunabotics nodes
	//To remap for Pioneer use RosAria/cmd_vel and RosAria/pose
	ros::Publisher ctrlPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", twistSize);	
	ros::Subscriber odoSubscriber = nodeHandle.subscribe("/odom", odomSize, odoCallback);
	
	ros::Rate loop_rate(50);
	stateMsg.odometry.header.frame_id = "base_link";
	publishState = false;
	
	ROS_INFO("MECH Gateway ready"); 
	
	while (ros::ok()) {     
		ctrlPublisher.publish(twistMsg);
		
		//Whenever needed send telemetry message
		if (publishState) {
			publishState = false;
			statePublisher.publish(stateMsg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
