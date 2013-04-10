#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/State.h"
#include "lunabotics/Vision.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/ControlMode.h"
#include "lunabotics/PID.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Empty.h"
#include "tf/tf.h"
#include "types.h"
#include <boost/asio.hpp>
#include "../protos_gen/Telemetry.pb.h"

boost::asio::io_service io_service;
boost::asio::ip::tcp::resolver resolver(io_service);
boost::asio::ip::tcp::resolver::iterator end; 
boost::asio::ip::tcp::socket sock(io_service);
boost::system::error_code error = boost::asio::error::host_not_found;

bool sendMap = false;
bool sendState = false;
bool sendVision = false;
bool sendPath = false;
lunabotics::State stateMsg;
lunabotics::ControlParams controlParams;
lunabotics::Vision vision;
nav_msgs::Path path;
lunabotics::SteeringModeType controlMode = lunabotics::ACKERMANN;

bool tryConnect(boost::asio::ip::tcp::resolver::query tcp_query)
{
	ROS_INFO("Trying to connect");
	boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(tcp_query);
	int i = 0;
	while (error && endpoint_iterator != end) {
		ROS_INFO("Checking endpoint %d", i++);
	    sock.close();
	    sock.connect(*endpoint_iterator++, error);
	}
	if (error) {
		//throw boost::system::system_error(error);
		ROS_WARN("Failed to connect. %s", error.message().c_str());
	    return false;
	}
	return true;
}	

void stateCallback(const lunabotics::State& msg)
{    
	stateMsg = msg;
	sendState = true;
}

void mapUpdateCallback(const std_msgs::Empty& msg)
{    
	sendMap = true;
}

void controlModeCallback(const lunabotics::ControlMode& msg)
{
	controlMode = (lunabotics::SteeringModeType)msg.mode;					
}

void controlParamsCallback(const lunabotics::ControlParams& msg)
{
	controlParams = msg;		
}

void pathCallback(const nav_msgs::Path& msg)
{    
	path = msg;
	sendPath = true;
}

void visionCallback(const lunabotics::Vision& msg)
{
	sendState = false;
	vision = msg;
	sendVision = true;
}

int main(int argc, char **argv)
{
	if (argc < 1) {
		ROS_ERROR("USAGE: rosrun luna_gui_gw <GUI IP Address>");
		return 1;
	}
	
	ros::init(argc, argv, "luna_gui_gw");
	
	boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), argv[1], "44325"); 
	
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber stateSubscriber = nodeHandle.subscribe("state", 256, stateCallback);
	ros::Subscriber mapUpdateSubscriber = nodeHandle.subscribe("map_update", 0, mapUpdateCallback);
	ros::Subscriber pathSubscriber = nodeHandle.subscribe("path", 256, pathCallback);
	ros::Subscriber controlModeSubscriber = nodeHandle.subscribe("control_mode", 1, controlModeCallback);
	ros::Subscriber controlParamsSubscriber = nodeHandle.subscribe("control_params", 1, controlParamsCallback);
	ros::Subscriber visionSubscriber = nodeHandle.subscribe("vision", 1, visionCallback);
	ros::ServiceClient mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("map");
	nav_msgs::GetMap mapService;
    
   	ROS_INFO("GUI Gateway ready");
   	   	
	ros::Rate loop_rate(20);
	while (ros::ok()) {
		try {
			if (error) {
				if (!tryConnect(query)) {
			        ROS_ERROR("Failed to connect to server");
				}		
				else {
				    ROS_INFO("Connected to server");
				    sendMap = true;
				}
			}
			else if (sendMap || sendPath || sendState || sendVision) {
				
				lunabotics::Telemetry tm;		
				if (sendState) {
					lunabotics::Telemetry::State *state = tm.mutable_state_data();
					state->mutable_position()->set_x(stateMsg.odometry.pose.pose.position.x);
					state->mutable_position()->set_y(stateMsg.odometry.pose.pose.position.y);
					state->set_heading(tf::getYaw(stateMsg.odometry.pose.pose.orientation));
					state->mutable_velocities()->set_linear(stateMsg.odometry.twist.twist.linear.x);
					state->mutable_velocities()->set_angular(stateMsg.odometry.twist.twist.angular.z);
					state->set_steering_mode(controlMode);
					state->set_autonomy_enabled(controlParams.driving);
					
					if (controlParams.driving) {
						state->set_next_waypoint_idx(controlParams.next_waypoint_idx);
						if (controlMode == lunabotics::ACKERMANN) {
							lunabotics::Telemetry::State::AckermannTelemetry *ackermannData = state->mutable_ackermann_telemetry();
							ackermannData->set_pid_error(controlParams.y_err);
							ackermannData->mutable_closest_trajectory_point()->set_x(controlParams.trajectory_point.x);
							ackermannData->mutable_closest_trajectory_point()->set_y(controlParams.trajectory_point.y);
							ackermannData->mutable_velocity_vector_point()->set_x(controlParams.velocity_point.x);
							ackermannData->mutable_velocity_vector_point()->set_y(controlParams.velocity_point.y);
							ackermannData->mutable_closest_trajectory_local_point()->set_x(controlParams.t_trajectory_point.x);
							ackermannData->mutable_closest_trajectory_local_point()->set_y(controlParams.t_trajectory_point.y);
							ackermannData->mutable_velocity_vector_local_point()->set_x(controlParams.t_velocity_point.x);
							ackermannData->mutable_velocity_vector_local_point()->set_y(controlParams.t_velocity_point.y);
						}
					}
					
				    sendState = false;
				}
				if (sendMap) {
					nav_msgs::OccupancyGrid map;
					bool include_map = false;
					if (sendMap) {
						if (mapClient.call(mapService)) {
							map = mapService.response.map;
							include_map = true;
						}
						else {
							ROS_WARN("Failed to call service /lunabotics/map");
						}
					}
					
					if (include_map) {
						lunabotics::Telemetry::World *world = tm.mutable_world_data();
						world->set_width(map.info.width);
						world->set_height(map.info.height);
						world->set_resolution(map.info.resolution);
						unsigned int mapSize = map.info.width*map.info.height;
						for (unsigned int i = 0; i < mapSize; i++) {
							world->add_cell(mapService.response.map.data.at(i));
						}
						ROS_INFO("Sending a map (%dx%d)", map.info.width, map.info.height);
					}
					
					sendMap = false;
				}
				if (sendPath) {
				    for (unsigned int i = 0; i < path.poses.size(); i++) {
						geometry_msgs::PoseStamped pose = path.poses.at(i);
						lunabotics::Point *point = tm.mutable_path_data()->add_position();
						point->set_x(pose.pose.position.x);
						point->set_y(pose.pose.position.y);
					}
				    
				    sendPath = false;
				}
				if (sendVision) {
					//Encode vision
				    
				    sendVision = false;
				}
				
				if (!tm.IsInitialized()) {
					ROS_WARN("Error serializing Telemetry");
					ROS_WARN_STREAM(tm.InitializationErrorString());
				}
				else {
				    boost::asio::write(sock, boost::asio::buffer(tm.SerializeAsString().c_str(), tm.ByteSize()));
				}
			}
		}
		catch (std::exception& e) {
			ROS_WARN(e.what());
			error = boost::asio::error::host_not_found;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	sock.close();
	
	return 0;
}
