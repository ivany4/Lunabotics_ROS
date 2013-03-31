#include "ros/ros.h"
#include "lunabotics/Telemetry.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "nav_msgs/Odometry.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

geometry_msgs::Twist twistMsg;
lunabotics::Telemetry telemetryMsg;
gazebo_msgs::SetModelState setGazeboStateService;
bool publishTelemetry;

void controlCallback(const lunabotics::Control& msg)
{	
	//For stage and pioneer
	twistMsg.linear.x = msg.motion.linear.x;
	twistMsg.angular.z = msg.motion.angular.z;
	
	//For gazebo
	setGazeboStateService.request.model_state.twist = twistMsg;
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
	
	//Stageros communication protocols
	ros::Publisher stagePublisher = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 256);	
	ros::Subscriber stageOdoSubscriber = nodeHandle.subscribe("odom", 256, odoCallback);
	
	//Pioneer communication protocols
	ros::Publisher pioneerPublisher = nodeHandle.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 256);	
	ros::Subscriber pioneerOdoSubscriber = nodeHandle.subscribe("RosAria/pose", 256, odoCallback);
	
	//Gazebo communication protocols
	bool gazeboAvailable = false;
	gazebo_msgs::GetModelState getGazeboStateService;
	setGazeboStateService.request.model_state.model_name = getGazeboStateService.request.model_name = "elias";
	setGazeboStateService.request.model_state.reference_frame = ""; //Empty for world frame. Otherwise
	getGazeboStateService.request.relative_entity_name = ""; //Empty for world frame. Or reference object name
	setGazeboStateService.request.model_state.pose.position.x = 0;
	setGazeboStateService.request.model_state.pose.position.y = 0;
	setGazeboStateService.request.model_state.pose.position.z = 0;
	setGazeboStateService.request.model_state.pose.orientation.x = 0;
	setGazeboStateService.request.model_state.pose.orientation.y = 0;
	setGazeboStateService.request.model_state.pose.orientation.z = 0;
	setGazeboStateService.request.model_state.pose.orientation.w = 0;
	
	
	ros::ServiceClient setGazeboStateClient = nodeHandle.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
	ros::ServiceClient getGazeboStateClient = nodeHandle.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
			
	ros::Rate loop_rate(50);
	telemetryMsg.odometry.header.frame_id = "/map";
	publishTelemetry = false;
	
	ROS_INFO("MECH Gateway ready"); 
	
	while (ros::ok()) {     
		stagePublisher.publish(twistMsg);
        pioneerPublisher.publish(twistMsg);
        if (setGazeboStateClient.call(setGazeboStateService)) {
			gazeboAvailable = true;
			if (!setGazeboStateService.response.success) {
				ROS_ERROR("Gazebo set state error: %s", setGazeboStateService.response.status_message.c_str());
			}
			else {
				ROS_INFO("Setting gazebo state. Pose %.2f,%.2f,%.2f. Twist %.2f,%.5f", setGazeboStateService.request.model_state.pose.position.x, setGazeboStateService.request.model_state.pose.position.y, setGazeboStateService.request.model_state.pose.position.z, setGazeboStateService.request.model_state.twist.linear.x, setGazeboStateService.request.model_state.twist.angular.z);
			}
		}
		else {
			gazeboAvailable = false;
		}
		  		
		//Whenever needed send telemetry message
		if (publishTelemetry) {
			if (gazeboAvailable) {
				if (getGazeboStateClient.call(getGazeboStateService)) {
					if (getGazeboStateService.response.success) {
						telemetryMsg.odometry.twist.twist = getGazeboStateService.response.twist;
						telemetryMsg.odometry.pose.pose = getGazeboStateService.response.pose;
					}
					else {
						ROS_ERROR("Gazebo get state error: %s", getGazeboStateService.response.status_message.c_str());
					}
				}
				else {
					ROS_ERROR("Failed to call get gazebo state service");
				}		
			}
			
			publishTelemetry = false;
			telemetryPublisher.publish(telemetryMsg);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
