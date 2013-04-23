#include "ros/ros.h"
#include "lunabotics/State.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/AllWheelStateROS.h"
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
lunabotics::AllWheelStateROS expectedResult;
bool expectingResult = false;

#define STEERING_ACCURACY	0.01

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

void AllWheelStateCallback(const lunabotics::AllWheelStateROS& msg)
{
	if (expectingResult) {
		ROS_INFO("Expecting %.2f, %.2f, %.2f, %.2f", msg.steering.left_front, msg.steering.right_front, msg.steering.left_rear, msg.steering.right_rear);
		ROS_INFO("Getting %.2f, %.2f, %.2f, %.2f", expectedResult.steering.left_front, expectedResult.steering.right_front, expectedResult.steering.left_rear, expectedResult.steering.right_rear);
		if (fabs(msg.steering.left_front-expectedResult.steering.left_front) <= STEERING_ACCURACY &&
		fabs(msg.steering.right_front-expectedResult.steering.right_front) <= STEERING_ACCURACY &&
		fabs(msg.steering.left_rear-expectedResult.steering.left_rear) <= STEERING_ACCURACY &&
		fabs(msg.steering.right_rear-expectedResult.steering.right_rear) <= STEERING_ACCURACY) {
			ROS_INFO("Reached correct steering angles");
			expectingResult = false;
			incrementState();
		}
	}
}

void allWheelCommonCallback(const lunabotics::AllWheelCommon& msg)
{
	currentPredefinedCommand = (lunabotics::AllWheelControl::PredefinedControlType)msg.predefined_cmd;
	state_cnt = -1;
	expectingResult = true;
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
	ros::Subscriber allWheelCommonSubscriber = nodeHandle.subscribe("all_wheel_common", 1, allWheelCommonCallback);
	ros::Subscriber AllWheelStateSubscriber = nodeHandle.subscribe("all_wheel_feeback", sizeof(float)*8, AllWheelStateCallback);
	ros::Subscriber steeringCallbackSubscriber = nodeHandle.subscribe("steering_complete", 1, steeringCompleteCallback);
	ros::Publisher statePublisher = nodeHandle.advertise<lunabotics::State>("state", 256);
	ros::Publisher AllWheelStateROSPublisher = nodeHandle.advertise<lunabotics::AllWheelStateROS>("all_wheel", 256);
	
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
				lunabotics::AllWheelStateROS msg;
				switch (currentPredefinedCommand) {
					case lunabotics::AllWheelControl::CRAB_LEFT: {
						switch (state_cnt) {
							case 0: {
								msg.driving.left_front = 0;
								msg.driving.right_front = 0;
								msg.driving.left_rear = 0;
								msg.driving.right_rear = 0;
								msg.steering.left_front = -M_PI_2;
								msg.steering.right_front = M_PI_2;
								msg.steering.left_rear = M_PI_2;
								msg.steering.right_rear = -M_PI_2;
								expectedResult = msg;
							}
							break;
							
							case 1: {
								msg.driving.left_front = -1;
								msg.driving.right_front = 1;
								msg.driving.left_rear = 1;
								msg.driving.right_rear = -1;
								msg.steering.left_front = -M_PI_2;
								msg.steering.right_front = M_PI_2;
								msg.steering.left_rear = M_PI_2;
								msg.steering.right_rear = -M_PI_2;
								expectingResult = false;
							}
							break;
						}
					}
					break;
					
					case lunabotics::AllWheelControl::CRAB_RIGHT: {
						switch (state_cnt) {
							case 0: {
								msg.driving.left_front = 0;
								msg.driving.right_front = 0;
								msg.driving.left_rear = 0;
								msg.driving.right_rear = 0;
								msg.steering.left_front = -M_PI_2;
								msg.steering.right_front = M_PI_2;
								msg.steering.left_rear = M_PI_2;
								msg.steering.right_rear = -M_PI_2;
								expectedResult = msg;
							}
							break;
							
							case 1: {
								msg.driving.left_front = 1;
								msg.driving.right_front = -1;
								msg.driving.left_rear = -1;
								msg.driving.right_rear = 1;
								msg.steering.left_front = -M_PI_2;
								msg.steering.right_front = M_PI_2;
								msg.steering.left_rear = M_PI_2;
								msg.steering.right_rear = -M_PI_2;
								expectingResult = false;
							}
							break;
						}
					}
					break;
					
					case lunabotics::AllWheelControl::TURN_CCW: {
						switch (state_cnt) {
							case 0: {
								msg.driving.left_front = 0;
								msg.driving.right_front = 0;
								msg.driving.left_rear = 0;
								msg.driving.right_rear = 0;
								msg.steering.left_front = -M_PI_4;
								msg.steering.right_front = M_PI_4;
								msg.steering.left_rear = M_PI_4;
								msg.steering.right_rear = -M_PI_4;
								expectedResult = msg;
							}
							break;
							
							case 1: {
								msg.driving.left_front = -1;
								msg.driving.right_front = 1;
								msg.driving.left_rear = -1;
								msg.driving.right_rear = 1;
								msg.steering.left_front = -M_PI_4;
								msg.steering.right_front = M_PI_4;
								msg.steering.left_rear = M_PI_4;
								msg.steering.right_rear = -M_PI_4;
								expectingResult = false;
							}
							break;
						}
					}
					break;
					
					case lunabotics::AllWheelControl::TURN_CW: {
						switch (state_cnt) {
							case 0: {
								msg.driving.left_front = 0;
								msg.driving.right_front = 0;
								msg.driving.left_rear = 0;
								msg.driving.right_rear = 0;
								msg.steering.left_front = -M_PI_4;
								msg.steering.right_front = M_PI_4;
								msg.steering.left_rear = M_PI_4;
								msg.steering.right_rear = -M_PI_4;
								expectedResult = msg;
							}
							break;
							
							case 1: {
								msg.driving.left_front = 1;
								msg.driving.right_front = -1;
								msg.driving.left_rear = 1;
								msg.driving.right_rear = -1;
								msg.steering.left_front = -M_PI_4;
								msg.steering.right_front = M_PI_4;
								msg.steering.left_rear = M_PI_4;
								msg.steering.right_rear = -M_PI_4;
								expectingResult = false;
							}
							break;
						}
					}
					break;
					
					case lunabotics::AllWheelControl::DRIVE_FORWARD: {
						switch (state_cnt) {
							case 0: {
								msg.driving.left_front = 0;
								msg.driving.right_front = 0;
								msg.driving.left_rear = 0;
								msg.driving.right_rear = 0;
								msg.steering.left_front = 0;
								msg.steering.right_front = 0;
								msg.steering.left_rear = 0;
								msg.steering.right_rear = 0;
								expectedResult = msg;
							}
							break;
							
							case 1: {
								msg.driving.left_front = 1;
								msg.driving.right_front = 1;
								msg.driving.left_rear = 1;
								msg.driving.right_rear = 1;
								msg.steering.left_front = 0;
								msg.steering.right_front = 0;
								msg.steering.left_rear = 0;
								msg.steering.right_rear = 0;
								expectingResult = false;
							}
							break;
						}
					}
					break;
					
					case lunabotics::AllWheelControl::DRIVE_BACKWARD: {
						switch (state_cnt) {
							case 0: {
								msg.driving.left_front = 0;
								msg.driving.right_front = 0;
								msg.driving.left_rear = 0;
								msg.driving.right_rear = 0;
								msg.steering.left_front = 0;
								msg.steering.right_front = 0;
								msg.steering.left_rear = 0;
								msg.steering.right_rear = 0;
								expectedResult = msg;
							}
							break;
							
							case 1: {
								msg.driving.left_front = -1;
								msg.driving.right_front = -1;
								msg.driving.left_rear = -1;
								msg.driving.right_rear = -1;
								msg.steering.left_front = 0;
								msg.steering.right_front = 0;
								msg.steering.left_rear = 0;
								msg.steering.right_rear = 0;
								expectingResult = false;
							}
							break;
						}
					}
					break;
					
					default:
					break;
				}
				AllWheelStateROSPublisher.publish(msg);
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
