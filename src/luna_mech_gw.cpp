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
	//For stage and pioneer
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
	ros::NodeHandle nodeHandle;
	ros::Subscriber controlSubscriber = nodeHandle.subscribe("lunabotics/control", 256, controlCallback);
	ros::Publisher telemetryPublisher = nodeHandle.advertise<lunabotics::Telemetry>("lunabotics/telemetry", 256);
	
	int twistSize = 256;
	int odomSize = 256;
	
	//Stageros communication protocols
	ros::Publisher stagePublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", twistSize);	
	ros::Subscriber stageOdoSubscriber = nodeHandle.subscribe("odom", odomSize, odoCallback);
	
	//Pioneer communication protocols
	ros::Publisher pioneerPublisher = nodeHandle.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", twistSize);	
	ros::Subscriber pioneerOdoSubscriber = nodeHandle.subscribe("RosAria/pose", odomSize, odoCallback);
	
	//Gazebo communication protocols
	ros::Publisher gazeboPublisher = nodeHandle.advertise<geometry_msgs::Twist>("gz/cmd_vel", twistSize);	
	ros::Subscriber gazeboOdoSubscriber = nodeHandle.subscribe("gz/odom", odomSize, odoCallback);
			
	ros::Rate loop_rate(50);
	telemetryMsg.odometry.header.frame_id = "/map";
	publishTelemetry = false;
	
	ROS_INFO("MECH Gateway ready"); 
	
	while (ros::ok()) {     
		stagePublisher.publish(twistMsg);
        pioneerPublisher.publish(twistMsg);
        gazeboPublisher.publish(twistMsg);
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
