#include "ros/ros.h"
#include "lunabotics/Teleoperation.h"
#include "lunabotics/State.h"
#include "lunabotics/Vision.h"
#include "lunabotics/PathFollowingTelemetry.h"
#include "lunabotics/SteeringMode.h"
#include "lunabotics/AllWheelState.h"
#include "lunabotics/PID.h"
#include "lunabotics/RobotGeometry.h"
#include "lunabotics/PathTopic.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Empty.h"
#include "tf/tf.h"
#include "types.h"
#include "topics.h"
#include <boost/asio.hpp>
#include "../protos_gen/Telemetry.pb.h"

#define MAP_CHUNK_SIZE	400

boost::asio::io_service io_service;
boost::asio::ip::tcp::resolver resolver(io_service);
boost::asio::ip::tcp::resolver::iterator end; 
boost::asio::ip::tcp::socket sock(io_service);
boost::system::error_code error = boost::asio::error::host_not_found;

bool sendMap = false;
bool sendState = false;
bool sendVision = false;
bool sendPath = false;
bool sendAllWheel = false;
bool sendGeometry = false;

int map_total_chunks = 0;
int map_current_chunk = 0;

lunabotics::State stateMsg;
lunabotics::PathFollowingTelemetry pathFollowingMsg;
lunabotics::Vision vision;
lunabotics::RobotGeometry geometry;
lunabotics::AllWheelState allWheelStateMsg;
lunabotics::PathTopic path;
geometry_msgs::Point ICR;
nav_msgs::OccupancyGrid map;
lunabotics::proto::SteeringModeType controlMode = lunabotics::proto::ACKERMANN;

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

void ICRCallback(const geometry_msgs::Point& msg) {
	ICR = msg;
}

void mapUpdateCallback(const std_msgs::Empty& msg)
{    
	ROS_INFO("Will update map");
	sendMap = true;
}

void controlModeCallback(const lunabotics::SteeringMode& msg)
{
	controlMode = (lunabotics::proto::SteeringModeType)msg.mode;					
}

void pathFollowingMsgCallback(const lunabotics::PathFollowingTelemetry& msg)
{
	pathFollowingMsg = msg;		
}

void pathCallback(const lunabotics::PathTopic& msg)
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

void geometryCallback(const lunabotics::RobotGeometry& msg)
{
	sendGeometry = true;
	geometry = msg;
}

void AllWheelStateCallback(const lunabotics::AllWheelState& msg)
{
	allWheelStateMsg = msg;
	sendAllWheel = true;
}

int main(int argc, char **argv)
{
	if (argc < 1) {
		ROS_ERROR("USAGE: rosrun luna_gui_gw <GUI IP Address>");
		return 1;
	}
	
	ros::init(argc, argv, "luna_gui_gw");
	
	boost::asio::ip::tcp::resolver::query query(boost::asio::ip::tcp::v4(), argv[1], "44326"); 
	
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber stateSubscriber = nodeHandle.subscribe(TOPIC_TM_ROBOT_STATE, 256, stateCallback);
	ros::Subscriber mapUpdateSubscriber = nodeHandle.subscribe(TOPIC_CMD_UPDATE_MAP, 0, mapUpdateCallback);
	ros::Subscriber pathSubscriber = nodeHandle.subscribe(TOPIC_TM_PATH, 256, pathCallback);
	ros::Subscriber controlModeSubscriber = nodeHandle.subscribe(TOPIC_STEERING_MODE, 1, controlModeCallback);
	ros::Subscriber pathFollowingMsgSubscriber = nodeHandle.subscribe(TOPIC_TM_PATH_FOLLOWING, 1, pathFollowingMsgCallback);
	ros::Subscriber visionSubscriber = nodeHandle.subscribe(TOPIC_TM_VISION, 1, visionCallback);
	ros::Subscriber ICRSubscriber = nodeHandle.subscribe(TOPIC_TM_ICR, sizeof(float)*2, ICRCallback);
	ros::Subscriber AllWheelStateSubscriber = nodeHandle.subscribe(TOPIC_TM_ALL_WHEEL, sizeof(float)*8, AllWheelStateCallback);
	ros::Subscriber geometrySubscriber = nodeHandle.subscribe(TOPIC_TM_ROBOT_GEOMETRY, sizeof(float)*4*2, geometryCallback);
	ros::ServiceClient mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>(SERVICE_MAP);
	nav_msgs::GetMap mapService;
	ICR.x = 0;
	ICR.y = 0;
    
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
			else if (sendMap || sendPath || sendState || sendVision || sendAllWheel || sendGeometry) {
				
				lunabotics::proto::Telemetry tm;		
				if (sendState) {
					lunabotics::proto::Telemetry::State *state = tm.mutable_state_data();
					state->mutable_position()->set_x(stateMsg.odometry.pose.pose.position.x);
					state->mutable_position()->set_y(stateMsg.odometry.pose.pose.position.y);
					state->set_heading(tf::getYaw(stateMsg.odometry.pose.pose.orientation));
					state->mutable_velocities()->set_linear(stateMsg.odometry.twist.twist.linear.x);
					state->mutable_velocities()->set_angular(stateMsg.odometry.twist.twist.angular.z);
					state->set_steering_mode(controlMode);
					state->set_autonomy_enabled(pathFollowingMsg.is_driving);
					state->mutable_icr()->set_x(ICR.x);
					state->mutable_icr()->set_y(ICR.y);
					
					if (pathFollowingMsg.is_driving) {
						if (pathFollowingMsg.has_min_icr_radius) {
							ROS_INFO("Setting %f", pathFollowingMsg.min_icr_radius);
							state->set_min_icr_offset(pathFollowingMsg.min_icr_radius);
						}
						if (pathFollowingMsg.has_point_turn_state) {
							lunabotics::proto::Telemetry::PointTurnState st = (lunabotics::proto::Telemetry::PointTurnState)pathFollowingMsg.point_turn_state;
							ROS_ERROR("STATE %d", st);
							state->mutable_point_turn_telemetry()->set_state(st);
						}
						if (pathFollowingMsg.has_path_parts_enumeration) {
							state->set_next_waypoint_idx(pathFollowingMsg.next_waypoint_idx);
							state->set_segment_idx(pathFollowingMsg.segment_idx);
						}
						if (pathFollowingMsg.has_path_following_geometry) {
							lunabotics::proto::Telemetry::State::AckermannTelemetry *ackermannData = state->mutable_ackermann_telemetry();
							ackermannData->set_feedback_error(pathFollowingMsg.feedback_error);
							ackermannData->set_feedforward_prediction(pathFollowingMsg.feedforward_prediction);
							ackermannData->set_feedforward_curve_radius(pathFollowingMsg.feedforward_curve_radius);
							ackermannData->mutable_feedback_path_point()->set_x(pathFollowingMsg.feedback_path_point.x);
							ackermannData->mutable_feedback_path_point()->set_y(pathFollowingMsg.feedback_path_point.y);
							ackermannData->mutable_feedback_point()->set_x(pathFollowingMsg.feedback_point.x);
							ackermannData->mutable_feedback_point()->set_y(pathFollowingMsg.feedback_point.y);
							ackermannData->mutable_feedback_path_point_local()->set_x(pathFollowingMsg.feedback_path_point_local.x);
							ackermannData->mutable_feedback_path_point_local()->set_y(pathFollowingMsg.feedback_path_point_local.y);
							ackermannData->mutable_feedback_point_local()->set_x(pathFollowingMsg.feedback_point_local.x);
							ackermannData->mutable_feedback_point_local()->set_y(pathFollowingMsg.feedback_point_local.y);
							ackermannData->mutable_feedforward_center()->set_x(pathFollowingMsg.feedforward_center.x);
							ackermannData->mutable_feedforward_center()->set_y(pathFollowingMsg.feedforward_center.y);
							ackermannData->set_heading_error(pathFollowingMsg.heading_error);
							for (unsigned int i = 0; i < pathFollowingMsg.feedforward_points_local.size(); i++) {
								geometry_msgs::Point pt = pathFollowingMsg.feedforward_points_local.at(i);
								lunabotics::proto::Point *point = ackermannData->add_feedforward_points_local();
								point->set_x(pt.x);
								point->set_y(pt.y);
							}
						}
					}
					
				    sendState = false;
				}
				
				bool include_map = false;
				if (map_current_chunk < map_total_chunks) {
					include_map = true;
				}
				else if (sendMap) {
					if (mapClient.call(mapService)) {
						map = mapService.response.map;
						map_current_chunk = 0;
						unsigned int mapSize = map.info.width*map.info.height;
						map_total_chunks = ceil(mapSize/((float)MAP_CHUNK_SIZE));
						include_map = true;
					}
					else {
						ROS_WARN("Failed to call service /lunabotics/map");
					}
				}
					
				if (include_map) {
					lunabotics::proto::Telemetry::World *world = tm.mutable_world_data();
					world->set_width(map.info.width);
					world->set_height(map.info.height);
					world->set_resolution(map.info.resolution);
					world->set_total_chunks(map_total_chunks);
					world->set_chunk_number(map_current_chunk);
					unsigned int mapSize = map.info.width*map.info.height;
					for (unsigned int i = map_current_chunk*MAP_CHUNK_SIZE; i < MAP_CHUNK_SIZE*(map_current_chunk+1) && i < mapSize && i < MAP_CHUNK_SIZE*map_total_chunks; i++) {
						world->add_cell(mapService.response.map.data.at(i));
					}
					ROS_INFO("Sending a map chunk %d of %d (%d cells)", map_current_chunk+1, map_total_chunks, world->cell().size());
					sendMap = false;
					map_current_chunk++;
				}
				
				
				
				
				if (sendPath) {
				    for (unsigned int i = 0; i < path.path.poses.size(); i++) {
						geometry_msgs::PoseStamped pose = path.path.poses.at(i);
						lunabotics::proto::Point *point = tm.mutable_path_data()->add_position();
						point->set_x(pose.pose.position.x);
						point->set_y(pose.pose.position.y);
					}
					for (unsigned int i = 0; i < path.curves.size(); i++) {
						lunabotics::CurveTopic curve = path.curves.at(i);
						lunabotics::proto::Telemetry::Path::Curve *protoCurve = tm.mutable_path_data()->add_curves();
						protoCurve->set_start_idx(curve.start_idx);
						protoCurve->set_end_idx(curve.end_idx);
						protoCurve->set_curvature(curve.curvature);
					}
				    
				    sendPath = false;
				}
				if (sendVision) {
					//Encode vision
				    
				    sendVision = false;
				}
				if (sendAllWheel) {
					lunabotics::proto::AllWheelState *state = tm.mutable_all_wheel_state();
					lunabotics::proto::AllWheelState::Wheels *steering = state->mutable_steering();
					lunabotics::proto::AllWheelState::Wheels *driving = state->mutable_driving();
					steering->set_left_front(allWheelStateMsg.steering.left_front);
					steering->set_right_front(allWheelStateMsg.steering.right_front);
					steering->set_left_rear(allWheelStateMsg.steering.left_rear);
					steering->set_right_rear(allWheelStateMsg.steering.right_rear);
					driving->set_left_front(allWheelStateMsg.driving.left_front);
					driving->set_right_front(allWheelStateMsg.driving.right_front);
					driving->set_left_rear(allWheelStateMsg.driving.left_rear);
					driving->set_right_rear(allWheelStateMsg.driving.right_rear);
					sendAllWheel = false;
				}
				
				if (sendGeometry) {
					lunabotics::proto::Telemetry::Geometry *positions = tm.mutable_geometry_data();
					positions->mutable_left_front_joint()->set_x(geometry.left_front_joint.x);
					positions->mutable_left_front_joint()->set_y(geometry.left_front_joint.y);
					positions->mutable_left_rear_joint()->set_x(geometry.left_rear_joint.x);
					positions->mutable_left_rear_joint()->set_y(geometry.left_rear_joint.y);
					positions->mutable_right_front_joint()->set_x(geometry.right_front_joint.x);
					positions->mutable_right_front_joint()->set_y(geometry.right_front_joint.y);
					positions->mutable_right_rear_joint()->set_x(geometry.right_rear_joint.x);
					positions->mutable_right_rear_joint()->set_y(geometry.right_rear_joint.y);
					positions->set_wheel_radius(geometry.wheel_radius);
					positions->set_wheel_offset(geometry.wheel_offset);
					positions->set_wheel_width(geometry.wheel_width);
					sendGeometry = false;
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
			ROS_WARN("%s", e.what());
			error = boost::asio::error::host_not_found;
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	sock.close();
	
	return 0;
}
