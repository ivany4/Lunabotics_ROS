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


union bytesToFloat
{
    char    bytes[4];
    float   floatValue;
};

enum ControlType
{
    ControlAckermann = 0,
    ControlSpot      = 1,
    ControlLateral   = 2
};

union BytesToUint8
{
    char bytes[1];
    uint8_t uint8Value;
};

void sendMsg(const char *msg, int sock) {
    int replylen = sizeof(msg);
    if (write(sock, msg, replylen) != replylen) {
        ROS_WARN("Failed to send bytes to client");
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_gui_listener");
	
	if (argc < 2 || argc > 3) {
		ROS_FATAL("USAGE: rosrun lunabotics lunabotics <port> [<ip address>]");
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
    
    ROS_INFO("GUI Listener ready on %s:%s", z, argv[1]);
    
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
    
	bool autonomyEnabled = false;
    
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
	        sendMsg("FAIL", clientSocket);
	        ROS_WARN("Failed to receive additional bytes from client");
	    }
    
    
		/* Print client message */
        ROS_INFO("Message from client on %s: %s", inet_ntoa(client.sin_addr), buffer);

		//Parse autonomy flag
		bool autoDrive = buffer[0];
		
		if (autoDrive != autonomyEnabled) {
			autonomyEnabled = autoDrive;
			
			lunabotics::BoolValue autonomyMsg;
			autonomyMsg.flag = autoDrive;
			autonomyPublisher.publish(autonomyMsg);
		}
		else {
			lunabotics::Control controlMsg;
			
			union BytesToUint8 intConverter;
			union bytesToFloat floatConverter;
			 
			//Parse motion control type flag
			intConverter.bytes[0] = buffer[1];
			uint8_t intType = intConverter.uint8Value;
			ControlType motionControlType = ControlAckermann;
			if (intType == 1) {
				motionControlType = ControlSpot;
			}
			else if (intType == 2) {
				motionControlType = ControlLateral;
			}
				

			//Parse linear velocity value
			floatConverter.bytes[0] = buffer[2];
			floatConverter.bytes[1] = buffer[3];
			floatConverter.bytes[2] = buffer[4];
			floatConverter.bytes[3] = buffer[5];
		    float linearSpeed = floatConverter.floatValue;
		
			//Parse control type dependent value
		    floatConverter.bytes[0] = buffer[6];
		    floatConverter.bytes[1] = buffer[7];
		    floatConverter.bytes[2] = buffer[8];
		    floatConverter.bytes[3] = buffer[9];
		    float dependentValue = floatConverter.floatValue;
		
		    bool driveForward = buffer[10];
		    bool driveBackward = buffer[11];
		    bool driveLeft = buffer[12];
		    bool driveRight = buffer[13];

			ROS_INFO("Autonomy enabled: %d", autoDrive);
			ROS_INFO("Control Type: %s", (motionControlType == ControlAckermann ? "Ackermann" : motionControlType == ControlLateral ? "Lateral" : "Spot"));
			ROS_INFO("Linear speed: %f", linearSpeed);
			ROS_INFO("Dependent value: %f", dependentValue);
			ROS_INFO("Driving mask: ");
			ROS_INFO("   Forward: %d", driveForward);
			ROS_INFO("   Backward: %d", driveBackward);
			ROS_INFO("   Left: %d", driveLeft);
			ROS_INFO("   Right: %d", driveRight);


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
		}
		
		
		ROS_INFO("Replying with OK");
		
	    /* Send back reply */
	    sendMsg("OK", clientSocket);
    
        
        
        
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
