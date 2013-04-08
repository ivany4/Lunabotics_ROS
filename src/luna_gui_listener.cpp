#include "ros/ros.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlMode.h"
#include "lunabotics/Goal.h"
#include "lunabotics/PID.h"
#include "lunabotics/AllWheelSteering.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Empty.h"
#include "types.h"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string>
#include <signal.h>
#include <unistd.h>
#include "../protos_gen/Telecommand.pb.h"
#include "../protos_gen/AllWheelControl.pb.h"

#define MAXPENDING 5        /* Max connection requests */
#define BUFFSIZE 256
#define SERVER_PORT "5555"

using namespace std;

int serverSocket = -1;
int clientSocket = -1;
float angleTolerance = 0.4;
float distanceTolerance = 0.1;


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
	GOOGLE_PROTOBUF_VERIFY_VERSION;	//Verify version of ProtoBuf package

	ros::init(argc, argv, "luna_gui_listener");
	
	if (argc < 2 || argc > 3) {
		ROS_FATAL("USAGE: rosrun lunabotics lunabotics <ip address> [port]");
		ros::shutdown();
	}
	
	ros::NodeHandle nodeHandle("lunabotics");
	ros::Publisher controlPublisher = nodeHandle.advertise<lunabotics::Control>("control", 256);
	ros::Publisher pidPublisher = nodeHandle.advertise<lunabotics::PID>("pid", sizeof(float)*3);
	ros::Publisher autonomyPublisher = nodeHandle.advertise<std_msgs::Bool>("autonomy", 1);
	ros::Publisher controlModePublisher = nodeHandle.advertise<lunabotics::ControlMode>("control_mode", 1);
	ros::Publisher goalPublisher = nodeHandle.advertise<lunabotics::Goal>("goal", 1);
	ros::Publisher allWheelPublisher = nodeHandle.advertise<lunabotics::AllWheelSteering>("all_wheel", sizeof(float)*8);
	ros::Publisher mapRequestPublisher = nodeHandle.advertise<std_msgs::Empty>("map_update", 1);
	
	
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
				
			lunabotics::Telecommand tc;

			if (!tc.ParseFromArray(buffer, received)) {
				ROS_WARN("Failed to parse Telecommand object");
				continue;
			}
				
			switch(tc.type()) {
				case lunabotics::Telecommand::SET_AUTONOMY: {
					
					ROS_INFO("%s autonomy", tc.autonomy_data().enabled() ? "Enabling" : "Disabling");
					
					std_msgs::Bool autonomyMsg;
					autonomyMsg.data = tc.autonomy_data().enabled();
					autonomyPublisher.publish(autonomyMsg);
				}
				break;
				
				case lunabotics::Telecommand::STEERING_MODE: {
					
					lunabotics::SteeringModeType type = tc.steering_mode_data().type();
					
					ROS_INFO("Switching control mode to %s", controlModeToString(type).c_str());
					
					lunabotics::ControlMode controlModeMsg;
					controlModeMsg.mode = type;
					if (type == lunabotics::ACKERMANN) {
						controlModeMsg.linear_speed_limit = tc.steering_mode_data().ackermann_steering_data().max_linear_velocity();
						controlModeMsg.smth_else = tc.steering_mode_data().ackermann_steering_data().bezier_curve_segments();
					}
					controlModePublisher.publish(controlModeMsg);
					
				}
				break;
				
				case lunabotics::Telecommand::TELEOPERATION: {
					lunabotics::Control controlMsg;
					
				    bool driveForward 	= tc.teleoperation_data().forward();
				    bool driveBackward 	= tc.teleoperation_data().backward();
				    bool driveLeft 		= tc.teleoperation_data().left();
				    bool driveRight 	= tc.teleoperation_data().right();
		
					ROS_INFO("%s%s%s%s", driveForward ? "^" : "", driveBackward ? "v" : "", driveLeft ? "<" : "", driveRight ? ">" : "");
					
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
					controlMsg.motion.linear.y = 0;
					controlMsg.motion.linear.z = 0;
					controlMsg.motion.angular.x = 0;
					controlMsg.motion.angular.y = 0;
					controlMsg.motion.angular.z = stageAngularSpeed;
					
					
					controlPublisher.publish(controlMsg);

				}
				break;
				
				case lunabotics::Telecommand::DEFINE_ROUTE: {
					float goalX = tc.define_route_data().goal().x();
					float goalY = tc.define_route_data().goal().y();
					float toleranceAngle = tc.define_route_data().heading_accuracy();
					float toleranceDistance = tc.define_route_data().position_accuracy();
					
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
				
				case lunabotics::Telecommand::REQUEST_MAP: {
					ROS_INFO("Receiving map request");
					std_msgs::Empty emptyMsg;
					mapRequestPublisher.publish(emptyMsg);
				}
				break;
				
				case lunabotics::Telecommand::ADJUST_PID: {
					lunabotics::PID pidMsg;
					pidMsg.p = tc.adjust_pid_data().p();
					pidMsg.i = tc.adjust_pid_data().i();
					pidMsg.d = tc.adjust_pid_data().d();
					pidMsg.velocity_offset = tc.adjust_pid_data().velocity_offset();
					pidMsg.velocity_multiplier = tc.adjust_pid_data().velocity_multiplier();
					pidPublisher.publish(pidMsg);
				}
				break;
				
				case lunabotics::Telecommand::ADJUST_WHEELS: {
					lunabotics::AllWheelSteering msg;
					msg.left_front_driving_vel = tc.all_wheel_control_data().driving().left_front();
					msg.right_front_driving_vel = tc.all_wheel_control_data().driving().right_front();
					msg.left_rear_driving_vel = tc.all_wheel_control_data().driving().left_rear();
					msg.right_rear_driving_vel = tc.all_wheel_control_data().driving().right_rear();
					msg.left_front_steering_ang = tc.all_wheel_control_data().steering().left_front();
					msg.right_front_steering_ang = tc.all_wheel_control_data().steering().right_front();
					msg.left_rear_steering_ang = tc.all_wheel_control_data().steering().left_rear();
					msg.right_rear_steering_ang = tc.all_wheel_control_data().steering().right_rear();
					allWheelPublisher.publish(msg);
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
