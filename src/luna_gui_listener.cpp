#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/BoolValue.h"
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


enum CTRL_MODE_TYPE {
    ACKERMANN 		 = 0,
    TURN_IN_SPOT     = 1,
    LATERAL   		 = 2
};

enum RX_CONTENT_TYPE {
	STEERING		 = 0,
	AUTONOMY		 = 1,
	CTRL_MODE		 = 2,
	ROUTE			 = 3
};

union BytesToFloat {
    char    bytes[4];
    float   floatValue;
};

union BytesToUint8 {
    char bytes[1];
    uint8_t uint8Value;
};
	

void replyToGUI(const char *msg, int sock) {
    int replylen = sizeof(msg);
    if (write(sock, msg, replylen) != replylen) {
        ROS_WARN("Failed to send bytes to client");
    }
}

RX_CONTENT_TYPE contentTypeFromChar(char c)
{
	union BytesToUint8 enumConverter;
	enumConverter.bytes[0] = c;
	uint8_t val = enumConverter.uint8Value;
	if (val == 1) return AUTONOMY;
	else if (val == 2) return CTRL_MODE;
	else if (val == 3) return ROUTE;
	return STEERING;
}

CTRL_MODE_TYPE controlModeFromChar(char c)
{
	union BytesToUint8 enumConverter;
	enumConverter.bytes[0] = c;
	uint8_t val = enumConverter.uint8Value;
	if (val == 1) return TURN_IN_SPOT;
	else if (val == 2) return LATERAL;
	return ACKERMANN;
}		

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_gui_listener");
	
	if (argc < 2 || argc > 3) {
		ROS_FATAL("USAGE: rosrun lunabotics lunabotics <ip address> [port]");
		ros::shutdown();
	}
	
	ros::NodeHandle nodeHandle;
	ros::Publisher controlPublisher = nodeHandle.advertise<lunabotics::Control>("luna_ctrl", 256);
	ros::Publisher autonomyPublisher = nodeHandle.advertise<lunabotics::BoolValue>("luna_auto", 256);
	
	int serverSocket, clientSocket;
	struct sockaddr_in server, client;
	
	/* Create the TCP socket */
    if ((serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        ROS_FATAL("Failed to create socket");
        ros::shutdown();
    }
    
    const char *port = argc > 2 ? argv[2] : "5555";
    
    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));         /* Clear struct */
    server.sin_family = AF_INET;                    /* Internet/IP */
    server.sin_addr.s_addr = inet_addr(argv[1]);
    server.sin_port = htons(atoi(port));         /* server port */
    
    /* Print connection details */
    char *z;
    z = inet_ntoa(server.sin_addr); /* cast s_addr as a struct in_addr */
    
    
    /* Bind the server socket */
    if (bind(serverSocket, (struct sockaddr *) &server, sizeof(server)) < 0) {
        ROS_ERROR("Failed to bind the server socket on %s:%s", z, port);
        ros::shutdown();
    }
    
    /* Listen on the server socket */
    if (listen(serverSocket, MAXPENDING) < 0) {
        ROS_ERROR("Unable to listen to socket on %s:%s", z, port);
        ros::shutdown();
    }
    
    ROS_INFO("GUI Listener ready on %s:%s", z, port);
    
	ros::Rate loop_rate(50);
  	
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
	        replyToGUI("FAILED TO READ", clientSocket);
	        ROS_WARN("Failed to receive additional bytes from client");
	    }
    
    
		/* Print client message */
        ROS_INFO("Received %d bytes from %s", received, inet_ntoa(client.sin_addr));
			
		//Parse autonomy flag
		RX_CONTENT_TYPE contentType = contentTypeFromChar(buffer[0]);
		
		switch (contentType) {
			case AUTONOMY: {
				union BytesToUint8 boolConverter;
				boolConverter.bytes[0] = buffer[1];
				bool enabled = boolConverter.uint8Value;
				
				ROS_INFO("%s autonomy", enabled ? "Enabling" : "Disabling");
				
				lunabotics::BoolValue autonomyMsg;
				autonomyMsg.flag = enabled;
				autonomyPublisher.publish(autonomyMsg);
				
				replyToGUI("OK", clientSocket);
			}
			break;
			
			case CTRL_MODE: {
				
				CTRL_MODE_TYPE type = controlModeFromChar(buffer[1]);
				
				ROS_INFO("Switching control mode to %s", (type == ACKERMANN ? "Ackermann" : type == LATERAL ? "Lateral" : "Spot"));
				
			    replyToGUI("OK", clientSocket);
			}
			break;
			
			case STEERING: {
				lunabotics::Control controlMsg;
				union BytesToFloat floatConverter;
			
				//Parse linear velocity value
				floatConverter.bytes[0] = buffer[1];
				floatConverter.bytes[1] = buffer[2];
				floatConverter.bytes[2] = buffer[3];
				floatConverter.bytes[3] = buffer[4];
			    float linearSpeed = floatConverter.floatValue;
			
				//Parse control type dependent value
			    floatConverter.bytes[0] = buffer[5];
			    floatConverter.bytes[1] = buffer[6];
			    floatConverter.bytes[2] = buffer[7];
			    floatConverter.bytes[3] = buffer[8];
			    float dependentValue = floatConverter.floatValue;
			
	
			    bool driveForward = buffer[9];
			    bool driveBackward = buffer[10];
			    bool driveLeft = buffer[11];
			    bool driveRight = buffer[12];
	
				ROS_INFO("Limits: %f %f %s%s%s%s", linearSpeed, dependentValue, driveForward ? "^" : "", driveBackward ? "v" : "", driveLeft ? "<" : "", driveRight ? ">" : "");
				
				

				//Just to work with Stageros
				////////////////////////////////////////////////////////////////
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
				controlMsg.motion.angular.z = stageAngularSpeed;
				//////////////////////////////////////////////////////////////////
				
				
				
				controlPublisher.publish(controlMsg);
				
			    replyToGUI("OK", clientSocket);
			}
			break;
			
			case ROUTE:
				replyToGUI("NOT IMPLEMENTED", clientSocket);
			break;
			
			default:
				replyToGUI("UNKNOWN CONTENT TYPE", clientSocket);
			break;
		}
		       
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
