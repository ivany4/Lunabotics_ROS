#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/BoolValue.h"
#include "lunabotics/Telemetry.h"
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <netinet/in.h>
#include <pthread.h>

#define BUFFSIZE 256
#define SERVER_ADDR	"192.168.218.1"
#define SERVER_PORT	"5556"

using namespace std;

int sock;
bool sock_conn = false;
struct sockaddr_in server;
pthread_mutex_t sock_mutex = PTHREAD_MUTEX_INITIALIZER;
    
union doubleToBytes
{
    char    bytes[8];
    double	doubleValue;
};

void sendMsg(const char *msg, int sock) {
    int replylen = sizeof(msg);
    if (write(sock, msg, replylen) != replylen) {
        ROS_WARN("Failed to send bytes to client");
    }
}

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


void telemetryCallback(const lunabotics::Telemetry& msg)
{    
	if (sock_conn) {
	    int size = sizeof(double)*3;
	    char send_buffer[size];
	    
	    ////////////DATA ENCODING////////////////////
	    
	    int pointer = 0;
	    
		union doubleToBytes doubleConverter;
		doubleConverter.doubleValue = msg.odometry.pose.pose.position.x;
		send_buffer[pointer++] = doubleConverter.bytes[0];
		send_buffer[pointer++] = doubleConverter.bytes[1];
		send_buffer[pointer++] = doubleConverter.bytes[2];
		send_buffer[pointer++] = doubleConverter.bytes[3];
		send_buffer[pointer++] = doubleConverter.bytes[4];
		send_buffer[pointer++] = doubleConverter.bytes[5];
		send_buffer[pointer++] = doubleConverter.bytes[6];
		send_buffer[pointer++] = doubleConverter.bytes[7];
		doubleConverter.doubleValue = msg.odometry.pose.pose.position.y;
		send_buffer[pointer++] = doubleConverter.bytes[0];
		send_buffer[pointer++] = doubleConverter.bytes[1];
		send_buffer[pointer++] = doubleConverter.bytes[2];
		send_buffer[pointer++] = doubleConverter.bytes[3];
		send_buffer[pointer++] = doubleConverter.bytes[4];
		send_buffer[pointer++] = doubleConverter.bytes[5];
		send_buffer[pointer++] = doubleConverter.bytes[6];
		send_buffer[pointer++] = doubleConverter.bytes[7];
		doubleConverter.doubleValue = msg.odometry.pose.pose.position.z;
		send_buffer[pointer++] = doubleConverter.bytes[0];
		send_buffer[pointer++] = doubleConverter.bytes[1];
		send_buffer[pointer++] = doubleConverter.bytes[2];
		send_buffer[pointer++] = doubleConverter.bytes[3];
		send_buffer[pointer++] = doubleConverter.bytes[4];
		send_buffer[pointer++] = doubleConverter.bytes[5];
		send_buffer[pointer++] = doubleConverter.bytes[6];
		send_buffer[pointer++] = doubleConverter.bytes[7];
	    
	    /////////////////////////////////////////////
	    
	    
	    int sent_bytes = 0;
	    
	    
		pthread_mutex_lock(&sock_mutex);
	    /* Send the word to the server */
	    if ((sent_bytes = write(sock, send_buffer, size)) != size) {
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_gui_gw");
	ros::NodeHandle nodeHandle;
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	
    
    
    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));         	/* Clear struct */
    server.sin_family = AF_INET;                    /* Internet/IP */ 
    server.sin_addr.s_addr = argc > 1 ? inet_addr(argv[1]) : inet_addr(SERVER_ADDR); 
    server.sin_port = argc > 2 ? htons(atoi(argv[2])) : htons(atoi(SERVER_PORT));
    
    
    
    /* Print connection details */
    char *addr;
    addr = inet_ntoa(server.sin_addr); /* cast s_addr as a struct in_addr */
    
   	ROS_INFO("GUI Gateway ready");
   	
	ros::Rate loop_rate(50);
	while (ros::ok()) {
		if (!sock_conn) {
			if (!tryConnect()) {
		        ROS_ERROR("Failed to connect to server %s:%hu", addr, ntohs(server.sin_port));
			}		
			else {
			    ROS_INFO("Connected to server on %s:%hu (socket %d)", addr, ntohs(server.sin_port), sock);
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	close(sock);
	
	return 0;
}
