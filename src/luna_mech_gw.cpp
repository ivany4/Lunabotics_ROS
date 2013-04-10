#include "ros/ros.h"
#include "lunabotics/State.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/AllWheelSteering.h"
#include "lunabotics/AllWheelCommon.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "../protos_gen/AllWheelControl.pb.h"

geometry_msgs::Twist twistMsg;
lunabotics::State stateMsg;
bool publishState;
int state_cnt = -1;
bool increment_state = false;
lunabotics::AllWheelControl::PredefinedControlType currentPredefinedCommand;

void incrementState() {
	state_cnt++;
	increment_state = true;
}

void controlCallback(const lunabotics::Control& msg)
{	
	twistMsg = msg.motion;
}

void odoCallback(const nav_msgs::Odometry& msg)
{
	stateMsg.odometry = msg;
	publishState = true;
}

void allWheelCallback(const lunabotics::AllWheelSteering& msg)
{
	//Do nothing yet
}

void allWheelCommonCallback(const lunabotics::AllWheelCommon& msg)
{
	currentPredefinedCommand = (lunabotics::AllWheelControl::PredefinedControlType)msg.predefined_cmd;
	state_cnt = -1;
	incrementState();
}

void steeringCompleteCallback(const std_msgs::Empty& msg)
{
	ROS_INFO("Action complete!");
	incrementState();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_mech_gw");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber controlSubscriber = nodeHandle.subscribe("control", 256, controlCallback);
	ros::Subscriber allWheelSubscriber = nodeHandle.subscribe("all_wheel", sizeof(float)*8, allWheelCallback);
	ros::Subscriber allWheelCommonSubscriber = nodeHandle.subscribe("all_wheel_common", 1, allWheelCommonCallback);
	ros::Subscriber steeringCallbackSubscriber = nodeHandle.subscribe("steering_complete", 1, steeringCompleteCallback);
	ros::Publisher statePublisher = nodeHandle.advertise<lunabotics::State>("state", 256);
	ros::Publisher allWheelStatePublisher = nodeHandle.advertise<lunabotics::AllWheelSteering>("all_wheel", 256);
	
	//This is generic. Used by stageros and gazebo lunabotics nodes
	//To remap for Pioneer use RosAria/cmd_vel and RosAria/pose
	ros::Publisher ctrlPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 256);	
	ros::Subscriber odoSubscriber = nodeHandle.subscribe("/odom", 256, odoCallback);
	
	ros::Rate loop_rate(50);
	stateMsg.odometry.header.frame_id = "base_link";
	publishState = false;
	
	ROS_INFO("MECH Gateway ready"); 
	
	while (ros::ok()) {
		ctrlPublisher.publish(twistMsg);
		
		//Whenever needed send telemetry message
		if (publishState) {
			publishState = false;
			statePublisher.publish(stateMsg);
		}
		
		if (increment_state) {
			increment_state = false;
			if (state_cnt > 1) {
				state_cnt = -1;
				ROS_INFO("Sequence completed");
			}
			else {
				lunabotics::AllWheelSteering msg;
				msg.report = true;
				switch (currentPredefinedCommand) {
					case lunabotics::AllWheelControl::CRAB_LEFT: {
						switch (state_cnt) {
							case 0: {
								msg.left_front_driving_vel = 0;
								msg.right_front_driving_vel = 0;
								msg.left_rear_driving_vel = 0;
								msg.right_rear_driving_vel = 0;
								msg.left_front_steering_ang = -M_PI_2;
								msg.right_front_steering_ang = M_PI_2;
								msg.left_rear_steering_ang = M_PI_2;
								msg.right_rear_steering_ang = -M_PI_2;
							}
							break;
							
							case 1: {
								msg.left_front_driving_vel = -1;
								msg.right_front_driving_vel = 1;
								msg.left_rear_driving_vel = 1;
								msg.right_rear_driving_vel = -1;
								msg.left_front_steering_ang = -M_PI_2;
								msg.right_front_steering_ang = M_PI_2;
								msg.left_rear_steering_ang = M_PI_2;
								msg.right_rear_steering_ang = -M_PI_2;
							}
							break;
						}
					}
					break;
					
					case lunabotics::AllWheelControl::CRAB_RIGHT: {
						switch (state_cnt) {
							case 0: {
								msg.left_front_driving_vel = 0;
								msg.right_front_driving_vel = 0;
								msg.left_rear_driving_vel = 0;
								msg.right_rear_driving_vel = 0;
								msg.left_front_steering_ang = -M_PI_2;
								msg.right_front_steering_ang = M_PI_2;
								msg.left_rear_steering_ang = M_PI_2;
								msg.right_rear_steering_ang = -M_PI_2;
							}
							break;
							
							case 1: {
								msg.left_front_driving_vel = 1;
								msg.right_front_driving_vel = -1;
								msg.left_rear_driving_vel = -1;
								msg.right_rear_driving_vel = 1;
								msg.left_front_steering_ang = -M_PI_2;
								msg.right_front_steering_ang = M_PI_2;
								msg.left_rear_steering_ang = M_PI_2;
								msg.right_rear_steering_ang = -M_PI_2;
							}
							break;
						}
					}
					break;
					
					default:
					break;
				}
				allWheelStatePublisher.publish(msg);
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
