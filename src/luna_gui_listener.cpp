#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlMode.h"
#include "lunabotics/Goal.h"
#include "lunabotics/PID.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "types.h"
#include "coding.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string>
#include <signal.h>
#include <unistd.h>

#define MAXPENDING 5        /* Max connection requests */
#define BUFFSIZE 256
#define SERVER_PORT "5555"

using namespace std;

int serverSocket = -1;
int clientSocket = -1;
float angleTolerance = 0.4;
float distanceTolerance = 0.1;


enum RX_CONTENT_TYPE {
	STEERING		 = 0,
	AUTONOMY		 = 1,
	CTRL_MODE		 = 2,
	ROUTE			 = 3,
	MAP_REQUEST		 = 4,
	PID				 = 5
};


void quit(int sig) { 
    close(serverSocket); 
    close(clientSocket); 
    ros::shutdown(); 
    exit(0); 
}	

void replyToGUI(const char *msg, int sock) {
    int replylen = sizeof(msg);
    if (write(sock, msg, replylen) != replylen) {
        ROS_WARN("Failed to send bytes to client");
    }
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
	ros::Publisher pidPublisher = nodeHandle.advertise<lunabotics::PID>("luna_pid", sizeof(float)*3);
	ros::Publisher autonomyPublisher = nodeHandle.advertise<std_msgs::Bool>("luna_auto", 1);
	ros::Publisher controlModePublisher = nodeHandle.advertise<lunabotics::ControlMode>("luna_ctrl_mode", 1);
	ros::Publisher goalPublisher = nodeHandle.advertise<lunabotics::Goal>("luna_goal", 1);
	ros::Publisher mapRequestPublisher = nodeHandle.advertise<std_msgs::Empty>("luna_map_update", 1);
	
	
    signal(SIGINT,quit);   // Quits program if ctrl + c is pressed 
	
	struct sockaddr_in server, client;
	
	bool connected = false;
	
    /* Construct the server sockaddr_in structure */
    memset(&server, 0, sizeof(server));         /* Clear struct */
    server.sin_family = AF_INET;                    /* Internet/IP */
    server.sin_addr.s_addr = inet_addr(argv[1]);
    server.sin_port = argc > 2 ? htons(atoi(argv[2])) : htons(atoi(SERVER_PORT));
    
    /* Print connection details */
    char *addr;
    addr = inet_ntoa(server.sin_addr); /* cast s_addr as a struct in_addr */
        
    
	ros::Rate loop_rate(50);
	ROS_INFO("GUI Listener ready");
  	
	while (ros::ok()) {
		if (!connected) {
			if (serverSocket >= 0) {
				close(serverSocket);
			}
			/* Create the TCP socket */
			if ((serverSocket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
				ROS_ERROR("Failed to create socket");
			}
			else if (bind(serverSocket, (struct sockaddr *) &server, sizeof(server)) < 0) {
				ROS_ERROR("Failed to bind the server socket on %s:%hu", addr, ntohs(server.sin_port));
			}
			else if (listen(serverSocket, MAXPENDING) < 0) {
		        ROS_ERROR("Unable to listen to socket on %s:%hu", addr, ntohs(server.sin_port));
			}
			else {
				ROS_INFO("Listening on %s:%hu", addr, ntohs(server.sin_port));
				connected = true;
			}
		}
		else {			
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
				
			int pointer = 0;	
			RX_CONTENT_TYPE contentType = (RX_CONTENT_TYPE)decodeByte(buffer, pointer);
			
			switch (contentType) {
				case AUTONOMY: {
					bool enabled = buffer[pointer++];
					
					ROS_INFO("%s autonomy", enabled ? "Enabling" : "Disabling");
					
					std_msgs::Bool autonomyMsg;
					autonomyMsg.data = enabled;
					autonomyPublisher.publish(autonomyMsg);
					
					//replyToGUI("OK", clientSocket);
				}
				break;
				
				case CTRL_MODE: {
					
					CTRL_MODE_TYPE type = (CTRL_MODE_TYPE)decodeByte(buffer, pointer);
					
					ROS_INFO("Switching control mode to %s", controlModeToString(type).c_str());
					
					lunabotics::ControlMode controlModeMsg;
					controlModeMsg.mode = type;
					if (type == ACKERMANN) {
						controlModeMsg.linear_speed_limit = decodeFloat(buffer, pointer);
						controlModeMsg.smth_else = decodeFloat(buffer, pointer);
					}
					controlModePublisher.publish(controlModeMsg);
					
					
					
				    //replyToGUI("OK", clientSocket);
				    
				    
				}
				break;
				
				case STEERING: {
					lunabotics::Control controlMsg;
					
				    bool driveForward 	= buffer[pointer++];
				    bool driveBackward 	= buffer[pointer++];
				    bool driveLeft 		= buffer[pointer++];
				    bool driveRight 	= buffer[pointer++];
		
					ROS_INFO("%s%s%s%s", driveForward ? "^" : "", driveBackward ? "v" : "", driveLeft ? "<" : "", driveRight ? ">" : "");
					
					
	
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
					
				    //replyToGUI("OK", clientSocket);
				}
				break;
				
				case ROUTE: {
					float goalX = decodeFloat(buffer, pointer);
					float goalY = decodeFloat(buffer, pointer);
					float toleranceAngle = decodeFloat(buffer, pointer);
					float toleranceDistance = decodeFloat(buffer, pointer);
					
					ROS_INFO("Navigation to (%.1f,%.1f)", goalX, goalY);
					
					lunabotics::Goal goalMsg;
					goalMsg.point.x = goalX;
					goalMsg.point.y = goalY;
					goalMsg.angleAccuracy = toleranceAngle;
					goalMsg.distanceAccuracy = toleranceDistance;
					goalPublisher.publish(goalMsg);
					
					//replyToGUI("OK", clientSocket);
				}
				break;
				
				case MAP_REQUEST: {
					ROS_INFO("Receiving map request");
					std_msgs::Empty emptyMsg;
					mapRequestPublisher.publish(emptyMsg);
				}
				break;
				
				case PID: {
					lunabotics::PID pidMsg;
					pidMsg.p = decodeFloat(buffer, pointer);
					pidMsg.i = decodeFloat(buffer, pointer);
					pidMsg.d = decodeFloat(buffer, pointer);
					pidMsg.velocity_offset = decodeFloat(buffer, pointer);
					pidMsg.velocity_multiplier = decodeFloat(buffer, pointer);
					pidPublisher.publish(pidMsg);
				}
				break;
				
				default:
					replyToGUI("UNKNOWN CONTENT TYPE", clientSocket);
				break;
			}
		}
		       
		ros::spinOnce();
		loop_rate.sleep();
	}
	
    close(serverSocket); 
    close(clientSocket); 
	
	return 0;
}
