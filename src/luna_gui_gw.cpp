#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/Telemetry.h"
#include "lunabotics/Vision.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/ControlMode.h"
#include "lunabotics/PID.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Empty.h"
#include "tf/tf.h"
#include "types.h"
#include "coding.h"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <pthread.h>
#include <bitset>
#include "motion/pid.h"

#define SERVER_ADDR	"192.168.218.1"
#define SERVER_PORT	"5556"

using namespace std;

int sock;
bool sock_conn = false;
bool sendMap = false;
bool sendTelemetry = false;
bool sendVision = false;
bool sendPath = false;
struct sockaddr_in server;
lunabotics::Telemetry telemetry;
lunabotics::ControlParams controlParams;
lunabotics::Vision vision;
motion::PIDGeometry pidGeometry;
nav_msgs::Path path;
CTRL_MODE_TYPE controlMode = ACKERMANN;
pthread_mutex_t sock_mutex = PTHREAD_MUTEX_INITIALIZER;
    

enum TX_CONTENT_TYPE {
	TELEMETRY 	= 0,
	MAP 		= 1,
	PATH 		= 2,
	LASER 		= 3
};

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

void transmit(char bytes[], int size)
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
	

void telemetryCallback(const lunabotics::Telemetry& msg)
{    
	telemetry = msg;
	sendTelemetry = true;
	
	//if (!controlParams.driving) {
		point_t traj_p, vel_p;
		pose_t currentPose = msg.odometry.pose.pose;
		pidGeometry.setCurrentPose(currentPose);
		pidGeometry.setLinearVelocity(msg.odometry.twist.twist.linear.x);
		controlParams.driving = true;
		controlParams.trajectory_point = pidGeometry.getClosestTrajectoryPoint();
		controlParams.velocity_point = pidGeometry.getReferencePoint();
		controlParams.y_err = pidGeometry.getReferenceDistance(traj_p, vel_p);
		controlParams.t_trajectory_point = traj_p;
		controlParams.t_velocity_point = vel_p;
		controlParams.next_waypoint_idx = 0;
	//}
}

void mapUpdateCallback(const std_msgs::Empty& msg)
{    
	sendMap = true;
}

void controlModeCallback(const lunabotics::ControlMode& msg)
{
	controlMode = (CTRL_MODE_TYPE)msg.mode;					
}

void controlParamsCallback(const lunabotics::ControlParams& msg)
{
	controlParams = msg;		
}

void pathCallback(const nav_msgs::Path& msg)
{    
	path = msg;
	
#pragma message("Test code")
//	if (!controlParams.driving) {
		point_arr poses;
		for (vector<geometry_msgs::PoseStamped>::iterator it = path.poses.begin(); it < path.poses.end(); it++) {
			poses.push_back((*it).pose.position);
		}
		pidGeometry.setTrajectory(poses);
//	}
	
	sendPath = true;
}

void visionCallback(const lunabotics::Vision& msg)
{
	sendTelemetry = false;
	vision = msg;
	sendVision = true;
}

void pidCallback(const lunabotics::PID& msg) 
{
	pidGeometry.setVelocityMultiplier(msg.velocity_multiplier);
	pidGeometry.setVelocityOffset(msg.velocity_offset);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_gui_gw");
	ros::NodeHandle nodeHandle;
	#pragma message("pid listener only for debug");
	ros::Subscriber pidSubscriber = nodeHandle.subscribe("luna_pid", sizeof(float)*3, pidCallback);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	ros::Subscriber mapUpdateSubscriber = nodeHandle.subscribe("luna_map_update", 0, mapUpdateCallback);
	ros::Subscriber pathSubscriber = nodeHandle.subscribe("luna_path", 256, pathCallback);
	ros::Subscriber controlModeSubscriber = nodeHandle.subscribe("luna_ctrl_mode", 1, controlModeCallback);
	ros::Subscriber controlParamsSubscriber = nodeHandle.subscribe("luna_ctrl_params", 1, controlParamsCallback);
	ros::Subscriber visionSubscriber = nodeHandle.subscribe("luna_vision", 1, visionCallback);
	ros::ServiceClient mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("luna_map");
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
   	
   	/*
   	ROS_INFO("TEST START");
   	int a = 0;
   	char *gg = new char[1];
   	gg[0] = '2';
   	ROS_INFO("PUTTING %c", gg[0]);
   	encodeByte(&gg, a, 2);
   	ROS_INFO("EXTRACTING %c", gg[0]);
   	
   	ROS_INFO("TEST SHIFTED POINTER TO %d", a);
   	*/
   	
   	
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
		else {
			bitset<8> type;
			int size = 1;
			nav_msgs::OccupancyGrid map;
			if (sendTelemetry) {
				type.set(TELEMETRY);
				size += sizeof(double)*9+1+1;
			    if (controlParams.driving) {
					size += sizeof(int32_t);
					if (controlMode == ACKERMANN) {
						size += sizeof(double)*9;
					}
				}
			}
			if (sendMap) {
				if (mapClient.call(mapService)) {
					type.set(MAP);
					map = mapService.response.map;
					uint8_t width = map.info.width;
					uint8_t height = map.info.height;
					unsigned int mapSize = width*height;
				    size += 2+sizeof(double)+mapSize;
				}
				else {
					ROS_WARN("Failed to call service /luna_map");
				}
			}
			if (sendPath) {
				type.set(PATH);
				uint8_t numOfPoses = path.poses.size();
			    size += 1+numOfPoses*2*sizeof(double);
			}
			if (sendVision) {
				type.set(LASER);
				unsigned int numOfRanges = vision.lidar_data.ranges.size();
			    size += sizeof(float)*3+sizeof(int)+numOfRanges*sizeof(float);
			}
			
			if (type.any()) {
			
			    uint8_t typeValue = (uint8_t)type.to_ulong();
				
			    char buffer[size];
			    int pointer = 0;
			    			    
				encodeByte(buffer, pointer, typeValue);
				
				
				if (sendTelemetry) {
					encodeDouble(buffer, pointer, telemetry.odometry.pose.pose.position.x);
					encodeDouble(buffer, pointer, telemetry.odometry.pose.pose.position.y);
					encodeDouble(buffer, pointer, tf::getYaw(telemetry.odometry.pose.pose.orientation));
					encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.linear.x);
					encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.linear.y);
					encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.linear.z);
					encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.angular.x);
					encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.angular.y);
					encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.angular.z);
					encodeByte(buffer, pointer, controlMode);
					encodeByte(buffer, pointer, controlParams.driving);
					if (controlParams.driving) {
						encodeInt(buffer, pointer, controlParams.next_waypoint_idx);
						if (controlMode == ACKERMANN) {
							encodeDouble(buffer, pointer, controlParams.y_err);
							encodeDouble(buffer, pointer, controlParams.trajectory_point.x);
							encodeDouble(buffer, pointer, controlParams.trajectory_point.y);
							encodeDouble(buffer, pointer, controlParams.velocity_point.x);
							encodeDouble(buffer, pointer, controlParams.velocity_point.y);
							encodeDouble(buffer, pointer, controlParams.t_trajectory_point.x);
							encodeDouble(buffer, pointer, controlParams.t_trajectory_point.y);
							encodeDouble(buffer, pointer, controlParams.t_velocity_point.x);
							encodeDouble(buffer, pointer, controlParams.t_velocity_point.y);
						}
					}
					
				    sendTelemetry = false;
				}
				if (sendMap) {
					uint8_t width = map.info.width;
					uint8_t height = map.info.height;
					double res = map.info.resolution;
					unsigned int mapSize = width*height;
				    encodeByte(buffer, pointer, width);
				    encodeByte(buffer, pointer, height);
				    encodeDouble(buffer, pointer, res);
					for (unsigned int i = 0; i < mapSize; i++) {
						encodeByte(buffer, pointer, mapService.response.map.data.at(i));
					}
					
					sendMap = false;
					ROS_INFO("Sending a map (%dx%d)", width, height);
				}
				if (sendPath) {
					uint8_t numOfPoses = path.poses.size();
				    encodeByte(buffer, pointer, numOfPoses);
				    for (unsigned int i = 0; i < path.poses.size(); i++) {
						geometry_msgs::PoseStamped pose = path.poses.at(i);
						double x_m = pose.pose.position.x;
						double y_m = pose.pose.position.y;
						encodeDouble(buffer, pointer, x_m);
						encodeDouble(buffer, pointer, y_m);
					}
				    
				    sendPath = false;
				}
				if (sendVision) {
					unsigned int numOfRanges = vision.lidar_data.ranges.size();
				    encodeFloat(buffer, pointer, vision.lidar_data.angle_min);
				    encodeFloat(buffer, pointer, vision.lidar_data.angle_max);
				    encodeFloat(buffer, pointer, vision.lidar_data.angle_increment);
				    encodeInt(buffer, pointer, numOfRanges);
				    for (unsigned int i = 0; i < vision.lidar_data.ranges.size(); i++) {
						encodeFloat(buffer, pointer, vision.lidar_data.ranges.at(i));
					}
				    
				    sendVision = false;
				}
			    transmit(buffer, size);
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	close(sock);
	
	return 0;
}
