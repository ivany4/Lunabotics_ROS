#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "lunabotics/BoolValue.h"
#include "lunabotics/Emergency.h"
#include "lunabotics/Control.h"
#include "nav_msgs/GetPlan.h"

void autonomyCallback(const lunabotics::BoolValue& msg)
{
	//Use msg to toggle autonomy
}

void emergencyCallback(const lunabotics::Emergency& msg)
{
	//Use msg to stop driving if applicable
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_pilot");
	ros::NodeHandle nodeHandle;
	ros::Subscriber autonomySubscriber = nodeHandle.subscribe("luna_auto", 256, autonomyCallback);
	ros::Subscriber emergencySubscriber = nodeHandle.subscribe("luna_alert", 256, emergencyCallback);
	ros::ServiceClient pathClient = nodeHandle.serviceClient<nav_msgs::GetPlan>("luna_path");
	ros::Publisher controlPublisher = nodeHandle.advertise<lunabotics::Control>("luna_ctrl", 256);
	ros::Rate loop_rate(50);
	nav_msgs::GetPlan pathService;
	
	ROS_INFO("Pilot ready"); 
	
	
	while (ros::ok()) {    
		
		//Whenever needed to get a path
		if (false) {
			//Specify params
			pathService.request.tolerance = 1.0;
			//pathService.request.start = 
			//pathService.request.goal = 
			if (pathClient.call(pathService)) {
				//Use path
				//srv.response.plan
				ROS_INFO("Got path from service");
			}
			else {
				ROS_ERROR("Failed to call service luna_path");
			}
		}
		
		
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
