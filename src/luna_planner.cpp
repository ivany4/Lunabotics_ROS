#include "ros/ros.h"
#include "lunabotics/BoolValue.h"
#include "lunabotics/Telemetry.h"
#include "lunabotics/Control.h"

void autonomyCallback(const lunabotics::BoolValue& msg)
{
	//Use msg to toggle autonomy
}

void telemetryCallback(const lunabotics::Telemetry& msg)
{
	//Use msg to recognize tilt angle. Raise alert if something is dangerous
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_planner");
	ros::NodeHandle nodeHandle;
	ros::Subscriber autonomySubscriber = nodeHandle.subscribe("luna_auto", 256, autonomyCallback);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	ros::Publisher controlPublisher = nodeHandle.advertise<lunabotics::Control>("luna_ctrl", 256);
	ros::Rate loop_rate(50);
	
	ROS_INFO("State Machine ready"); 
	
	
	while (ros::ok()) {     		
		//Whenever needed send control message
		if (false) {
			lunabotics::Control controlMsg;
			controlMsg.motion.linear.x = 0;
			controlPublisher.publish(controlMsg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
  
	return 0;
}
