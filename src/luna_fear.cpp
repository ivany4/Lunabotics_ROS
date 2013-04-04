#include "ros/ros.h"
#include "lunabotics/Emergency.h"
#include "lunabotics/Telemetry.h"
#include "lunabotics/Vision.h"

lunabotics::Emergency emergencyMsg;

void visionCallback(const lunabotics::Vision& msg)
{
	//Use msg to recognize obstacles and craters. Raise alert if something is dangerous
}
void telemetryCallback(const lunabotics::Telemetry& msg)
{
	//Use msg to recognize tilt angle. Raise alert if something is dangerous
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_fear");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber visionSubscriber = nodeHandle.subscribe("vision", 256, visionCallback);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("telemetry", 256, telemetryCallback);
	ros::Publisher emergencyPublisher = nodeHandle.advertise<lunabotics::Emergency>("emergency", 256);
	ros::Rate loop_rate(50);
	
	ROS_INFO("Emergency behavior ready"); 
	
	while (ros::ok()) {     
		emergencyPublisher.publish(emergencyMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}
  
	return 0;
}
