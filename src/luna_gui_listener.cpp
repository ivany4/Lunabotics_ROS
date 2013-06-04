#include "ros/ros.h"
#include "lunabotics/Teleoperation.h"
#include "lunabotics/SteeringMode.h"
#include "lunabotics/Goal.h"
#include "lunabotics/PID.h"
#include "lunabotics/AllWheelState.h"
#include "lunabotics/AllWheelCommon.h"
#include "lunabotics/ICRControl.h"
#include "lunabotics/CrabControl.h"
#include "geometry/allwheel.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include <boost/asio.hpp>
#include <string>
#include <signal.h>
#include "types.h"
#include "topics.h"
#include "../protos_gen/Telecommand.pb.h"
#include "../protos_gen/AllWheelControl.pb.h"

#define PORT	49203

boost::asio::io_service io_service; 
boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), PORT); 
boost::asio::ip::tcp::acceptor acceptor(io_service, endpoint); 
boost::asio::ip::tcp::socket sock(io_service);
boost::array<char, 4096> buffer;

ros::Publisher allWheelPublisher;
ros::Publisher allWheelCommonPublisher;
ros::Publisher teleoperationPublisher;
ros::Publisher controlModePublisher;
ros::Publisher pidPublisher;
ros::Publisher autonomyPublisher;
ros::Publisher goalPublisher;
ros::Publisher mapRequestPublisher;
ros::Publisher ICRPublisher;
ros::Publisher CrabPublisher;


void quit(int sig) {
	ROS_INFO("Terminating node"); 
	sock.close();
    ros::shutdown(); 
    exit(0); 
}	

void emergency_stop()
{
	ROS_WARN("Emergency STOP");
	lunabotics::Teleoperation controlMsg;
	controlMsg.forward = false;
	controlMsg.backward = false;
	controlMsg.left = false;
	controlMsg.right = false;
	teleoperationPublisher.publish(controlMsg);
	
	lunabotics::AllWheelState msg;
	msg.driving.left_front = 0;
	msg.driving.right_front = 0;
	msg.driving.left_rear = 0;
	msg.driving.right_rear = 0;
	allWheelPublisher.publish(msg);
}

void accept_handler(const boost::system::error_code &ec);

void read_handler(boost::system::error_code ec, std::size_t bytes_transferred)
{
	if (!ec) {
		ROS_INFO("Received %d bytes", (int)bytes_transferred);
		
		lunabotics::proto::Telecommand tc;
		
		if (!tc.ParseFromArray(buffer.data(), bytes_transferred)) {
			ROS_ERROR("Failed to parse Telecommand object");
			emergency_stop();
			return;
		}
		
		switch(tc.type()) {
			case lunabotics::proto::Telecommand::SET_AUTONOMY: {
				
				ROS_INFO("%s autonomy", tc.autonomy_data().enabled() ? "Enabling" : "Disabling");
				
				std_msgs::Bool autonomyMsg;
				autonomyMsg.data = tc.autonomy_data().enabled();
				autonomyPublisher.publish(autonomyMsg);
			}
			break;
			
			case lunabotics::proto::Telecommand::STEERING_MODE: {
				
				lunabotics::proto::SteeringModeType type = tc.steering_mode_data().type();
				
				ROS_INFO("Switching control mode to %s", steeringModeToString(type).c_str());
				
				lunabotics::SteeringMode controlModeMsg;
				controlModeMsg.mode = type;
				controlModeMsg.distance_accuracy = tc.steering_mode_data().position_accuracy();
				controlModeMsg.angle_accuracy = tc.steering_mode_data().heading_accuracy();
				controlModeMsg.linear_speed_limit = tc.steering_mode_data().max_linear_velocity();
				controlModeMsg.bezier_segments = tc.steering_mode_data().bezier_curve_segments();
				controlModePublisher.publish(controlModeMsg);
			}
			break;
			
			case lunabotics::proto::Telecommand::TELEOPERATION: {
				lunabotics::Teleoperation teleopMsg;
				teleopMsg.forward = tc.teleoperation_data().forward();
				teleopMsg.backward = tc.teleoperation_data().backward();
				teleopMsg.left = tc.teleoperation_data().left();
				teleopMsg.right = tc.teleoperation_data().right();
				teleoperationPublisher.publish(teleopMsg);
			}
			break;
			
			case lunabotics::proto::Telecommand::DEFINE_ROUTE: {
				lunabotics::Goal goalMsg;
				const lunabotics::proto::Telecommand::DefineRoute route = tc.define_route_data();
	            for (int i = 0; i < route.waypoints_size(); i++) {
	                const lunabotics::proto::Point waypoint = route.waypoints(i);
	                geometry_msgs::Point pt = lunabotics::geometry_msgs_Point_from_Point(lunabotics::CreatePoint(waypoint.x(), waypoint.y()));
					goalMsg.waypoints.push_back(pt);
					ROS_INFO("Navigation to (%.1f,%.1f)", pt.x, pt.y);
				}
				goalPublisher.publish(goalMsg);
			}
			break;
			
			case lunabotics::proto::Telecommand::REQUEST_MAP: {
				ROS_INFO("Receiving map request");
				std_msgs::Empty emptyMsg;
				mapRequestPublisher.publish(emptyMsg);
			}
			break;
			
			case lunabotics::proto::Telecommand::ADJUST_PID: {
				lunabotics::PID pidMsg;
				pidMsg.p = tc.adjust_pid_data().p();
				pidMsg.i = tc.adjust_pid_data().i();
				pidMsg.d = tc.adjust_pid_data().d();
				pidMsg.feedback_min_offset = tc.adjust_pid_data().feedback_min_offset();
				pidMsg.feedback_multiplier = tc.adjust_pid_data().feedback_multiplier();
				pidMsg.feedforward_min_offset = tc.adjust_pid_data().feedforward_min_offset();
				pidMsg.feedforward_fraction = tc.adjust_pid_data().feedforward_fraction();
				pidPublisher.publish(pidMsg);
			}
			break;
			
			case lunabotics::proto::Telecommand::ADJUST_WHEELS: {
				switch (tc.all_wheel_control_data().all_wheel_type()) {
					case lunabotics::proto::AllWheelControl::EXPLICIT: {
						lunabotics::proto::AllWheelState::Wheels driving = tc.all_wheel_control_data().explicit_data().driving();
						lunabotics::proto::AllWheelState::Wheels steering = tc.all_wheel_control_data().explicit_data().steering();
						
						float left_front = steering.left_front();
						float right_front = steering.right_front();
						float left_rear = steering.left_rear();
						float right_rear = steering.right_rear();
						lunabotics::validateAngles(left_front, right_front, left_rear, right_rear);
						
						
						lunabotics::AllWheelState msg;
						msg.driving.left_front = driving.left_front();
						msg.driving.right_front = driving.right_front();
						msg.driving.left_rear = driving.left_rear();
						msg.driving.right_rear = driving.right_rear();
						msg.steering.left_front = left_front;
						msg.steering.right_front = right_front;
						msg.steering.left_rear = left_rear;
						msg.steering.right_rear = right_rear;
						allWheelPublisher.publish(msg);
					}
					break;
					
					case lunabotics::proto::AllWheelControl::PREDEFINED: {
						lunabotics::AllWheelCommon msg;
						msg.predefined_cmd = tc.all_wheel_control_data().predefined_data().command();
						allWheelCommonPublisher.publish(msg);
					}
					break;
					
					case lunabotics::proto::AllWheelControl::ICR: {
						lunabotics::ICRControl msg;
						msg.ICR.x = tc.all_wheel_control_data().icr_data().icr().x();
						msg.ICR.y = tc.all_wheel_control_data().icr_data().icr().y();
						msg.velocity = tc.all_wheel_control_data().icr_data().velocity();
						ICRPublisher.publish(msg);
					}
					break;
					
					case lunabotics::proto::AllWheelControl::CRAB: {
						lunabotics::CrabControl msg;
						msg.heading = tc.all_wheel_control_data().crab_data().heading();
						msg.velocity = tc.all_wheel_control_data().crab_data().velocity();
						CrabPublisher.publish(msg);
					}
					break;
					
					default:
					break;
				}
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
		ROS_INFO("Connection accepted on %d", PORT);
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

	ros::init(argc, argv, "luna_gui_listener");
		
	ros::NodeHandle nodeHandle("lunabotics");
	teleoperationPublisher = nodeHandle.advertise<lunabotics::Teleoperation>(TOPIC_CMD_TELEOP, 256);
	pidPublisher = nodeHandle.advertise<lunabotics::PID>(TOPIC_CMD_PATH_FOLLOWING, sizeof(float)*3);
	autonomyPublisher = nodeHandle.advertise<std_msgs::Bool>(TOPIC_CMD_AUTONOMY, 1);
	controlModePublisher = nodeHandle.advertise<lunabotics::SteeringMode>(TOPIC_STEERING_MODE, 1);
	goalPublisher = nodeHandle.advertise<lunabotics::Goal>(TOPIC_CMD_GOAL, 1);
	allWheelPublisher = nodeHandle.advertise<lunabotics::AllWheelState>(TOPIC_CMD_EXPLICIT_ALL_WHEEL, sizeof(float)*8);
	allWheelCommonPublisher = nodeHandle.advertise<lunabotics::AllWheelCommon>(TOPIC_CMD_ALL_WHEEL, sizeof(int32_t));
	mapRequestPublisher = nodeHandle.advertise<std_msgs::Empty>(TOPIC_CMD_UPDATE_MAP, 1);
	ICRPublisher = nodeHandle.advertise<lunabotics::ICRControl>(TOPIC_CMD_ICR, sizeof(float)*3);
	CrabPublisher = nodeHandle.advertise<lunabotics::CrabControl>(TOPIC_CMD_CRAB, sizeof(float)*3);
	
  	
    signal(SIGINT,quit);   // Quits program if ctrl + c is pressed 
    
	acceptor.listen(); 
	ROS_INFO("Listening on %d", PORT);
	acceptor.async_accept(sock, accept_handler); 
	io_service.run(); 
	
    ros::shutdown(); 
	return 0;
}
