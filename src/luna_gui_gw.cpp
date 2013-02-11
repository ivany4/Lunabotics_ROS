#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/Telemetry.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Empty.h"
#include "std_msgs/UInt8.h"
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

#define BUFFSIZE (sizeof(double)*6)
#define SERVER_ADDR	"192.168.218.1"
#define SERVER_PORT	"5556"

using namespace std;

int sock;
bool sock_conn = false;
bool sendMap = false;
bool sendTelemetry = false;
struct sockaddr_in server;
lunabotics::Telemetry telemetry;
CTRL_MODE_TYPE controlMode = ACKERMANN;
pthread_mutex_t sock_mutex = PTHREAD_MUTEX_INITIALIZER;
    
union doubleToBytes
{
    char    bytes[8];
    double	doubleValue;
};

union uint8ToBytes
{
	uint8_t uint8Value;
	char bytes[1];
};

union intToBytes
{
	int intValue;
	char bytes[4];
};

enum TX_CONTENT_TYPE {
	TELEMETRY = 0,
	MAP = 1,
	PATH = 2
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

void encodeDouble(char buffer[], int &pointer, double value)
{
	union doubleToBytes doubleConverter;
	doubleConverter.doubleValue = value;
	for (unsigned int i = 0; i < sizeof(double); i++) {
		buffer[pointer++] = doubleConverter.bytes[i];
	}
}

void encodeInt(char buffer[], int &pointer, int value)
{
	union intToBytes intConverter;
	intConverter.intValue = value;
	for (unsigned int i = 0; i < sizeof(int32_t); i++) {
		buffer[pointer++] = intConverter.bytes[i];
	}
}

void encodeByte(char buffer[], int &pointer, uint8_t value)
{
	union uint8ToBytes uint8Converter;
	uint8Converter.uint8Value = value;
	buffer[pointer++] = uint8Converter.bytes[0];
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
			ROS_INFO("Sending data: OK");
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
}

void mapUpdateCallback(const std_msgs::Empty& msg)
{    
	sendMap = true;
}

void controlModeCallback(const std_msgs::UInt8& msg)
{
	controlMode = (CTRL_MODE_TYPE)msg.data;					
}


void pathCallback(const nav_msgs::Path& msg)
{    
	if (sock_conn) {
		uint8_t numOfPoses = msg.poses.size();
	    int size = 1+1+numOfPoses*2*sizeof(double);
	    char buffer[size];
	    int pointer = 0;
	    
	    sendMap = true;
	  
	    encodeByte(buffer, pointer, PATH);
	    encodeByte(buffer, pointer, numOfPoses);
	    for (unsigned int i = 0; i < msg.poses.size(); i++) {
			geometry_msgs::PoseStamped pose = msg.poses.at(i);
			double x_m = pose.pose.position.x;
			double y_m = pose.pose.position.y;
			encodeDouble(buffer, pointer, x_m);
			encodeDouble(buffer, pointer, y_m);
		}
	    
	    transmit(buffer, size);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_gui_gw");
	ros::NodeHandle nodeHandle;
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	ros::Subscriber mapUpdateSubscriber = nodeHandle.subscribe("luna_map_update", 0, mapUpdateCallback);
	ros::Subscriber pathSubscriber = nodeHandle.subscribe("luna_path", 256, pathCallback);
	ros::Subscriber controlModeSubscriber = nodeHandle.subscribe("luna_ctrl_mode", 1, controlModeCallback);
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
   	
   	
	ros::Rate loop_rate(1000);
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
			if (sendTelemetry) {
			    int size = sizeof(double)*9+1;
			    char buffer[size];
			    int pointer = 0;
			  
			    encodeByte(buffer, pointer, TELEMETRY);
				encodeDouble(buffer, pointer, telemetry.odometry.pose.pose.position.x);
				encodeDouble(buffer, pointer, telemetry.odometry.pose.pose.position.y);
				encodeDouble(buffer, pointer, tf::getYaw(telemetry.odometry.pose.pose.orientation));
				encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.linear.x);
				encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.linear.y);
				encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.linear.z);
				encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.angular.x);
				encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.angular.y);
				encodeDouble(buffer, pointer, telemetry.odometry.twist.twist.angular.z);
				encodeByte(buffer, pointer, (uint8_t)controlMode);
			    
			    sendTelemetry = false;
			    
			    transmit(buffer, size);
			}
			if (sendMap) {
				if (mapClient.call(mapService)) {
					uint8_t width = mapService.response.map.info.width;
					uint8_t height = mapService.response.map.info.height;
					double res = mapService.response.map.info.resolution;
					int mapSize = width*height;
				    int size = 1+sizeof(int)*2+sizeof(double)+mapSize;
				    char buffer[size];
				    int pointer = 0;
				  
				    encodeByte(buffer, pointer, MAP);
				    encodeByte(buffer, pointer, width);
				    encodeByte(buffer, pointer, height);
				    encodeDouble(buffer, pointer, res);
					for (unsigned int i = 0; i < mapSize; i++) {
						encodeByte(buffer, pointer, mapService.response.map.data.at(i));
					}
					
					sendMap = false;
					ROS_INFO("Sending a map (%dx%d)", width, height);
					transmit(buffer, size);
				}
				else {
					ROS_WARN("Failed to call service /luna_map");
				}
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	close(sock);
	
	return 0;
}
