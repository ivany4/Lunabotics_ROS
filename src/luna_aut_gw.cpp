#include "ros/ros.h"
#include "lunabotics/Vision.h"
#include "sensor_msgs/LaserScan.h"

lunabotics::Vision visionMsg;
bool publishVision = false;


void stageLaserCallback(const sensor_msgs::LaserScan& msg)
{
	std::vector<float> reducedRanges;
	
	for (unsigned int i = 0; i < msg.ranges.size(); i+=10) {
		reducedRanges.push_back(msg.ranges.at(i));
	}
	visionMsg.lidar_data = msg;
	visionMsg.lidar_data.ranges = reducedRanges;
	visionMsg.lidar_data.angle_increment *= 10.0;
	publishVision = true;
	ROS_INFO("Getting %d ranges. Reducing to %d", (int)msg.ranges.size(), (int)reducedRanges.size());
}

void pioneerLaserCallback(const sensor_msgs::LaserScan& msg)
{
	visionMsg.lidar_data = msg;
	publishVision = true;
	ROS_INFO("Getting %d ranges", (int)msg.ranges.size());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_aut_gw");
	ros::NodeHandle nodeHandle;
	
	ros::Publisher visionPublisher = nodeHandle.advertise<lunabotics::Vision>("lunabotics/vision", 256);
	
	//Stageros subscriptions
	ros::Subscriber stageLaserSubscriber = nodeHandle.subscribe("/base_scan", 256, stageLaserCallback);
	
	//Pioneer subscriptions
	ros::Subscriber pioneerLaserSubscriber = nodeHandle.subscribe("/scan", 256, pioneerLaserCallback);
	
	ROS_INFO("AutSys Gateway ready"); 
	
	
	ros::Rate loop_rate(50);
	while (ros::ok()) {     
		if (publishVision) {
			visionPublisher.publish(visionMsg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
  

	return 0;
}
