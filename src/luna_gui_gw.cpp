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
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <pthread.h>
#include <bitset>
#include "../protos_gen/Telemetry.pb.h"

#define SERVER_ADDR	"192.168.218.1"
#define SERVER_PORT	"5556"

using namespace std;

int sock;
bool sock_conn = false;
bool sendMap = false;
bool sendState = false;
bool sendVision = false;
bool sendPath = false;
struct sockaddr_in server;
lunabotics::State stateMsg;
lunabotics::ControlParams controlParams;
lunabotics::Vision vision;
nav_msgs::Path path;
lunabotics::SteeringModeType controlMode = lunabotics::ACKERMANN;
pthread_mutex_t sock_mutex = PTHREAD_MUTEX_INITIALIZER;

bool tryConnect() {
	sock_conn = false;
	if (sock >= 0) {
		close(sock);
	}
	
	if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        ROS_FATAL("Failed to create socket");
	}
	else {
	    sock_conn = !(connect(sock, (struct sockaddr *) &server, sizeof(server)) < 0);
	}
    return sock_conn;
}

void transmit(const char *bytes, int size)
{
	if (sock_conn) {
	    int sent_bytes = 0;
		pthread_mutex_lock(&sock_mutex);
	    /* Send the word to the server */
	    if ((sent_bytes = write(sock, bytes, size)) != size) {
	        ROS_ERROR("Sending data: Mismatch (%d instead of %d)", sent_bytes, size);
			pthread_mutex_unlock(&sock_mutex);
			tryConnect();
	        return;
	    }
	    else {
		//	ROS_INFO("Sending data: OK");
		}
		pthread_mutex_unlock(&sock_mutex);
	    
	    //Currently do not wait for reply from server
	    /*
	    int received = 0;
	    char recv_buffer[BUFFSIZE];
	    bzero(recv_buffer, BUFFSIZE);
	    if ((received = read(sock, recv_buffer, BUFFSIZE)) < 0) {
	        ROS_ERROR("Failed to receive additional bytes from client");
	        return;
	    }
	    */
	    
	    /* Print server message */
	    //ROS_INFO("Server answeded: %s", recv_buffer);
	}
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
	ros::init(argc, argv, "luna_gui_gw");
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Subscriber stateSubscriber = nodeHandle.subscribe("state", 256, stateCallback);
	ros::Subscriber mapUpdateSubscriber = nodeHandle.subscribe("map_update", 0, mapUpdateCallback);
	ros::Subscriber pathSubscriber = nodeHandle.subscribe("path", 256, pathCallback);
	ros::Subscriber controlModeSubscriber = nodeHandle.subscribe("control_mode", 1, controlModeCallback);
	ros::Subscriber controlParamsSubscriber = nodeHandle.subscribe("control_params", 1, controlParamsCallback);
	ros::Subscriber visionSubscriber = nodeHandle.subscribe("vision", 1, visionCallback);
	ros::ServiceClient mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("map");
	nav_msgs::GetMap mapService;
    
    
    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));         	/* Clear struct */
    server.sin_family = AF_INET;                    /* Internet/IP */ 
    server.sin_addr.s_addr = argc > 1 ? inet_addr(argv[1]) : inet_addr(SERVER_ADDR); 
    server.sin_port = argc > 2 ? htons(atoi(argv[2])) : htons(atoi(SERVER_PORT));
    
    
    
    /* Print connection details */
    char *addr;
    addr = inet_ntoa(server.sin_addr); /* cast s_addr as a struct in_addr */
    
   	ROS_INFO("GUI Gateway ready");
   	   	
	ros::Rate loop_rate(20);
	while (ros::ok()) {
		
		if (!sock_conn) {
			if (!tryConnect()) {
		        ROS_ERROR("Failed to connect to server %s:%hu", addr, ntohs(server.sin_port));
			}		
			else {
			    ROS_INFO("Connected to server on %s:%hu (socket %d)", addr, ntohs(server.sin_port), sock);
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
			    transmit(tm.SerializeAsString().c_str(), tm.ByteSize());
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	close(sock);
	
	return 0;
}
