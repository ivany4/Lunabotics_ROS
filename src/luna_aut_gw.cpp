#include "ros/ros.h"
#include "lunabotics/Vision.h"

lunabotics::Vision visionMsg;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "luna_aut_gw");
  ros::NodeHandle nodeHandle;
  
  ros::Publisher visionPublisher = nodeHandle.advertise<lunabotics::Vision>("luna_vision", 256);
    
  ROS_INFO("AutSys Gateway ready"); 
   
   
  ros::Rate loop_rate(50);
  while (ros::ok())
  {     
		visionPublisher.publish(visionMsg);
		ros::spinOnce();
		loop_rate.sleep();
  }
  

  return 0;
}
