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
#define MAXPENDING 5        /* Max connection requests */
#define BUFFSIZE 256

using namespace std;

union bytesToFloat
{
    char    bytes[4];
    float   floatValue;
};

void sendMsg(const char *msg, int sock) {
    int replylen = sizeof(msg);
    if (write(sock, msg, replylen) != replylen) {
        ROS_WARN("Failed to send bytes to client");
    }
}

void telemetryCallback(const lunabotics::Telemetry& msg)
{
	//Use msg to send telemetry to GUI and display it
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_gui_gw");
	
	if (argc < 2 || argc > 3) {
		ROS_FATAL("USAGE: rosrun lunabotics lunabotics <port> [<ip address>]");
		ros::shutdown();
	}
	
	ros::NodeHandle nodeHandle;
	ros::Publisher controlPublisher = nodeHandle.advertise<lunabotics::Control>("luna_ctrl", 256);
	ros::Publisher autonomyPublisher = nodeHandle.advertise<lunabotics::BoolValue>("luna_auto", 256);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	
	int serverSocket, clientSocket;
	struct sockaddr_in server, client;
	
	/* Create the TCP socket */
    if ((serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        ROS_FATAL("Failed to create socket");
        ros::shutdown();
    }
    
    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));         /* Clear struct */
    server.sin_family = AF_INET;                    /* Internet/IP */
    server.sin_addr.s_addr = argc == 3 ? inet_addr(argv[2]) :
    //inet_addr("127.0.0.1"); 
    htonl(INADDR_ANY);     /* Incoming addr */
    server.sin_port = htons(atoi(argv[1]));         /* server port */
    
    /* Print connection details */
    char *z;
    z = inet_ntoa(server.sin_addr); /* cast s_addr as a struct in_addr */
    
    ROS_INFO("Server ready on %s:%s", z, argv[1]);
    
    /* Bind the server socket */
    if (bind(serverSocket, (struct sockaddr *) &server, sizeof(server)) < 0) {
        ROS_ERROR("Failed to bind the server socket");
        ros::shutdown();
    }
    
    /* Listen on the server socket */
    if (listen(serverSocket, MAXPENDING) < 0) {
        ROS_ERROR("Failed to listen on server socket");
        ros::shutdown();
    }
    
	ros::Rate loop_rate(50);
  
	union bytesToFloat converter;  
	float v = 0.0, w = 0.0;
	
	while (ros::ok()) {
        unsigned int clientLenth = sizeof(client);
        
        
        /* Wait for client connection */
        if ((clientSocket = accept(serverSocket, (struct sockaddr *) &client, &clientLenth)) < 0) {
            ROS_WARN("Failed to accept client connection");
        }
        
		char buffer[BUFFSIZE];
		int received = 0;
    
		bzero(buffer,BUFFSIZE);
	    if ((received = read(clientSocket, buffer, BUFFSIZE)) < 0) {
	        sendMsg("FAIL", clientSocket);
	        ROS_WARN("Failed to receive additional bytes from client");
	    }
    
    
		/* Print client message */
        ROS_INFO("Message from client on %s: %s", inet_ntoa(client.sin_addr), buffer);
    
    
		converter.bytes[0] = buffer[0];
		converter.bytes[1] = buffer[1];
		converter.bytes[2] = buffer[2];
		converter.bytes[3] = buffer[3];
		v = converter.floatValue;
		converter.bytes[0] = buffer[4];
		converter.bytes[1] = buffer[5];
		converter.bytes[2] = buffer[6];
		converter.bytes[3] = buffer[7];
		w = converter.floatValue;
    
		bool controlOrAutonomy = true;
		
		if (controlOrAutonomy) {
			lunabotics::Control controlMsg;
			controlMsg.motion.linear.x = v;
			controlMsg.motion.angular.z = w;
			controlPublisher.publish(controlMsg);
		}
		else {
			lunabotics::BoolValue autonomyMsg;
			autonomyMsg.flag = false;
			autonomyPublisher.publish(autonomyMsg);
		}
		
		
	    /* Send back reply */
	    sendMsg("OK", clientSocket);
    
	    close(clientSocket);
        
        
        
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
