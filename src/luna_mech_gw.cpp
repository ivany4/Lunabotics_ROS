#include "ros/ros.h"
#include "lunabotics/Telemetry.h"
#include "lunabotics/Control.h"

geometry_msgs::Twist stageMsg;

void controlCallback(const lunabotics::Control& msg)
{
	stageMsg.linear.x = msg.motion.linear.x;
	stageMsg.angular.z = msg.motion.angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_mech_gw");
	ros::NodeHandle nodeHandle;
	ros::Subscriber controlSubscriber = nodeHandle.subscribe("luna_ctrl", 256, controlCallback);
	ros::Publisher telemetryPublisher = nodeHandle.advertise<lunabotics::Telemetry>("luna_tm", 256);
	ros::Publisher stagePublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 256);			//Stageros
	ros::Rate loop_rate(50);
	
	ROS_INFO("MECH Gateway ready"); 
	
	while (ros::ok()) {     
		stagePublisher.publish(stageMsg);
		  		
		//Whenever needed send telemetry message
		if (false) {
			lunabotics::Telemetry telemetryMsg;
			telemetryPublisher.publish(telemetryMsg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
