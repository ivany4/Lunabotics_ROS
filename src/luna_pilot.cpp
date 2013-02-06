#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "lunabotics/BoolValue.h"
#include "lunabotics/Emergency.h"
#include "lunabotics/Control.h"
#include "nav_msgs/GetPlan.h"

bool request_path = false;

void autonomyCallback(const lunabotics::BoolValue& msg)
{
	//Use msg to toggle autonomy
	if (msg.flag) {
		request_path = true;
	}
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
		if (request_path) {
			request_path = false;
			
			//Specify params
			geometry_msgs::PoseStamped start;
			geometry_msgs::PoseStamped goal;
			start.header.seq = 0;
			goal.header.seq = 1;
			start.header.frame_id = goal.header.frame_id = 1;
			start.pose.position.x = 0.1;
			start.pose.position.y = 0.1;
			start.pose.position.z = 0;
			goal.pose.position.x = 0.4;
			goal.pose.position.y = 0.4;
			goal.pose.position.z = 0;
			
			pathService.request.tolerance = 1.0;
			pathService.request.start = start;
			pathService.request.goal = goal;
			if (pathClient.call(pathService)) {
				
				for (int i = 0; i < pathService.response.plan.poses.size(); i++) {
					geometry_msgs::PoseStamped poseStamped = pathService.response.plan.poses.at(i);
					ROS_INFO("-> %f %f", poseStamped.pose.position.x, poseStamped.pose.position.y);
				}
				ROS_INFO("-------------------------");
				//Use path
				ROS_INFO("Returned %d waypoints", pathService.response.plan.poses.size());
				
				
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
