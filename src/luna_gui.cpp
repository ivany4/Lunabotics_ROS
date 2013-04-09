#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlMode.h"
#include "lunabotics/Goal.h"
#include "lunabotics/PID.h"
#include "lunabotics/AllWheelSteering.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include <boost/asio.hpp>
#include <string>
#include <signal.h>
#include "types.h"
#include "../protos_gen/Telecommand.pb.h"
#include "../protos_gen/AllWheelControl.pb.h"

boost::asio::io_service io_service; 
boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), 44324); 
boost::asio::ip::tcp::acceptor acceptor(io_service, endpoint); 
boost::asio::ip::tcp::socket sock(io_service);
boost::array<char, 4096> buffer;

ros::Publisher allWheelPublisher;
ros::Publisher controlPublisher;
ros::Publisher controlModePublisher;
ros::Publisher pidPublisher;
ros::Publisher autonomyPublisher;
ros::Publisher goalPublisher;
ros::Publisher mapRequestPublisher;


void quit(int sig) {
	ROS_INFO("Terminating node"); 
	sock.close();
    ros::shutdown(); 
    exit(0); 
}	

void emergency_stop()
{
	ROS_WARN("Emergency STOP");
	lunabotics::Control controlMsg;
	controlMsg.control_type = 0;	//Motion only
	controlMsg.motion.linear.x = 0;
	controlMsg.motion.linear.y = 0;
	controlMsg.motion.linear.z = 0;
	controlMsg.motion.angular.x = 0;
	controlMsg.motion.angular.y = 0;
	controlMsg.motion.angular.z = 0;
	controlPublisher.publish(controlMsg);
	
	lunabotics::AllWheelSteering msg;
	msg.left_front_driving_vel = 0;
	msg.right_front_driving_vel = 0;
	msg.left_rear_driving_vel = 0;
	msg.right_rear_driving_vel = 0;
	allWheelPublisher.publish(msg);
}

void accept_handler(const boost::system::error_code &ec);

void read_handler(boost::system::error_code ec, std::size_t bytes_transferred)
{
	if (!ec) {
		ROS_INFO("Received %d bytes", (int)bytes_transferred);
		
		lunabotics::Telecommand tc;
		
		if (!tc.ParseFromArray(buffer.data(), bytes_transferred)) {
			ROS_ERROR("Failed to parse Telecommand object");
			emergency_stop();
			return;
		}
		
		switch(tc.type()) {
			case lunabotics::Telecommand::SET_AUTONOMY: {
				
				ROS_INFO("%s autonomy", tc.autonomy_data().enabled() ? "Enabling" : "Disabling");
				
				std_msgs::Bool autonomyMsg;
				autonomyMsg.data = tc.autonomy_data().enabled();
				autonomyPublisher.publish(autonomyMsg);
			}
			break;
			
			case lunabotics::Telecommand::STEERING_MODE: {
				
				lunabotics::SteeringModeType type = tc.steering_mode_data().type();
				
				ROS_INFO("Switching control mode to %s", controlModeToString(type).c_str());
				
				lunabotics::ControlMode controlModeMsg;
				controlModeMsg.mode = type;
				if (type == lunabotics::ACKERMANN) {
					controlModeMsg.linear_speed_limit = tc.steering_mode_data().ackermann_steering_data().max_linear_velocity();
					controlModeMsg.smth_else = tc.steering_mode_data().ackermann_steering_data().bezier_curve_segments();
				}
				controlModePublisher.publish(controlModeMsg);
			}
			break;
			
			case lunabotics::Telecommand::TELEOPERATION: {
				lunabotics::Control controlMsg;
				
			    bool driveForward 	= tc.teleoperation_data().forward();
			    bool driveBackward 	= tc.teleoperation_data().backward();
			    bool driveLeft 		= tc.teleoperation_data().left();
			    bool driveRight 	= tc.teleoperation_data().right();
			
				ROS_INFO("%s%s%s%s", driveForward ? "^" : "", driveBackward ? "v" : "", driveLeft ? "<" : "", driveRight ? ">" : "");
				
				float stageLinearSpeed = 0;
				if (driveForward && !driveBackward) {
					stageLinearSpeed = 5.0;
				}
				else if (!driveForward && driveBackward) {
					stageLinearSpeed = -3.0;
				}
				
				float stageAngularSpeed = 0;
				if (driveLeft && !driveRight) {
					stageAngularSpeed = 1.0;
				}
				else if (!driveLeft && driveRight) {
					stageAngularSpeed = -1.0;
				}
				
				controlMsg.control_type = 0;	//Motion only
				controlMsg.motion.linear.x = stageLinearSpeed;
				controlMsg.motion.linear.y = 0;
				controlMsg.motion.linear.z = 0;
				controlMsg.motion.angular.x = 0;
				controlMsg.motion.angular.y = 0;
				controlMsg.motion.angular.z = stageAngularSpeed;
				controlPublisher.publish(controlMsg);
			}
			break;
			
			case lunabotics::Telecommand::DEFINE_ROUTE: {
				float goalX = tc.define_route_data().goal().x();
				float goalY = tc.define_route_data().goal().y();
				float toleranceAngle = tc.define_route_data().heading_accuracy();
				float toleranceDistance = tc.define_route_data().position_accuracy();
				
				ROS_INFO("Navigation to (%.1f,%.1f)", goalX, goalY);
				
				lunabotics::Goal goalMsg;
				goalMsg.point.x = goalX;
				goalMsg.point.y = goalY;
				goalMsg.angleAccuracy = toleranceAngle;
				goalMsg.distanceAccuracy = toleranceDistance;
				goalPublisher.publish(goalMsg);
			}
			break;
			
			case lunabotics::Telecommand::REQUEST_MAP: {
				ROS_INFO("Receiving map request");
				std_msgs::Empty emptyMsg;
				mapRequestPublisher.publish(emptyMsg);
			}
			break;
			
			case lunabotics::Telecommand::ADJUST_PID: {
				lunabotics::PID pidMsg;
				pidMsg.p = tc.adjust_pid_data().p();
				pidMsg.i = tc.adjust_pid_data().i();
				pidMsg.d = tc.adjust_pid_data().d();
				pidMsg.velocity_offset = tc.adjust_pid_data().velocity_offset();
				pidMsg.velocity_multiplier = tc.adjust_pid_data().velocity_multiplier();
				pidPublisher.publish(pidMsg);
			}
			break;
			
			case lunabotics::Telecommand::ADJUST_WHEELS: {
				lunabotics::AllWheelSteering msg;
				msg.left_front_driving_vel = tc.all_wheel_control_data().driving().left_front();
				msg.right_front_driving_vel = tc.all_wheel_control_data().driving().right_front();
				msg.left_rear_driving_vel = tc.all_wheel_control_data().driving().left_rear();
				msg.right_rear_driving_vel = tc.all_wheel_control_data().driving().right_rear();
				msg.left_front_steering_ang = tc.all_wheel_control_data().steering().left_front();
				msg.right_front_steering_ang = tc.all_wheel_control_data().steering().right_front();
				msg.left_rear_steering_ang = tc.all_wheel_control_data().steering().left_rear();
				msg.right_rear_steering_ang = tc.all_wheel_control_data().steering().right_rear();
				allWheelPublisher.publish(msg);
			}
			break;
			
			default:
			break;
		}
	}
	else if (ec.value() == boost::asio::error::eof) {
		ROS_INFO("Connection closed by client");
	}
	else {
		ROS_WARN("Failed to read data. Error code %s", ec.message().c_str());
	}
	sock.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
	sock.close();
	acceptor.async_accept(sock, accept_handler); 
}

void accept_handler(const boost::system::error_code &ec) 
{
	if (!ec) { 	
		ROS_INFO("Connection accepted on 44324");
		sock.async_read_some(boost::asio::buffer(buffer), read_handler); 
	}
	else {
		ROS_WARN("Failed to accept connection. Error code %s", ec.message().c_str());
		acceptor.async_accept(sock, accept_handler);
	}
}

int main(int argc, char **argv)
{
	GOOGLE_PROTOBUF_VERIFY_VERSION;	//Verify version of ProtoBuf package

	ros::init(argc, argv, "luna_gui");
		
	ros::NodeHandle nodeHandle("lunabotics");
	controlPublisher = nodeHandle.advertise<lunabotics::Control>("control", 256);
	pidPublisher = nodeHandle.advertise<lunabotics::PID>("pid", sizeof(float)*3);
	autonomyPublisher = nodeHandle.advertise<std_msgs::Bool>("autonomy", 1);
	controlModePublisher = nodeHandle.advertise<lunabotics::ControlMode>("control_mode", 1);
	goalPublisher = nodeHandle.advertise<lunabotics::Goal>("goal", 1);
	allWheelPublisher = nodeHandle.advertise<lunabotics::AllWheelSteering>("all_wheel", sizeof(float)*8);
	mapRequestPublisher = nodeHandle.advertise<std_msgs::Empty>("map_update", 1);
	
  	
    signal(SIGINT,quit);   // Quits program if ctrl + c is pressed 
    
	acceptor.listen(); 
	ROS_INFO("Listening on 44324");
	acceptor.async_accept(sock, accept_handler); 
	io_service.run(); 
	
    ros::shutdown(); 
	return 0;
}
