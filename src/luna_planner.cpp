#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "lunabotics/State.h"
#include "topics.h"

void autonomyCallback(const std_msgs::Bool& msg)
{
	//Use msg to toggle autonomy
}

void stateCallback(const lunabotics::State& msg)
{
	//Use msg to recognize tilt angle. Raise alert if something is dangerous
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_planner");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber autonomySubscriber = nodeHandle.subscribe(TOPIC_CMD_AUTONOMY, 256, autonomyCallback);
	ros::Subscriber stateSubscriber = nodeHandle.subscribe(TOPIC_TM_ROBOT_STATE, 256, stateCallback);
	ros::Rate loop_rate(50);
	
	ROS_INFO("State Machine ready"); 
	
	
	while (ros::ok()) {     		
		//Whenever needed send control message		
		ros::spinOnce();
		loop_rate.sleep();
	}
  
	return 0;
}
