#include "ros/ros.h"
#include "lunabotics/Telemetry.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "nav_msgs/Odometry.h"

geometry_msgs::Twist twistMsg;
lunabotics::Telemetry telemetryMsg;
bool publishTelemetry;

void controlCallback(const lunabotics::Control& msg)
{	
	twistMsg.linear.x = msg.motion.linear.x;
	twistMsg.angular.z = msg.motion.angular.z;
}

void odoCallback(const nav_msgs::Odometry& msg)
{
	telemetryMsg.odometry = msg;
	publishTelemetry = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_mech_gw");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber controlSubscriber = nodeHandle.subscribe("control", 256, controlCallback);
	ros::Publisher telemetryPublisher = nodeHandle.advertise<lunabotics::Telemetry>("telemetry", 256);
	
	int twistSize = 256;
	int odomSize = 256;
	
	//This is generic. Used by stageros and gazebo lunabotics nodes
	//To remap for Pioneer use RosAria/cmd_vel and RosAria/pose
	ros::Publisher ctrlPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", twistSize);	
	ros::Subscriber odoSubscriber = nodeHandle.subscribe("/odom", odomSize, odoCallback);
	
	ros::Rate loop_rate(50);
	telemetryMsg.odometry.header.frame_id = "/map";
	publishTelemetry = false;
	
	ROS_INFO("MECH Gateway ready"); 
	
	while (ros::ok()) {     
		ctrlPublisher.publish(twistMsg);
		
		//Whenever needed send telemetry message
		if (publishTelemetry) {
			publishTelemetry = false;
			telemetryPublisher.publish(telemetryMsg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
