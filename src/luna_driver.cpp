#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "lunabotics/Emergency.h"
#include "lunabotics/Control.h"
#include "lunabotics/Telemetry.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include "planning/a_star_graph.h"
#include "tf/tf.h"
using namespace std;


//Locally exponentially stable when Kp > 0; Kb < 0; Ka-Kb > 0
#define Kp 	1
#define Ka	2
#define Kb	-1

int seq = 0;
nav_msgs::GetMap mapService;
ros::ServiceClient mapClient;
geometry_msgs::Pose currentPose;
ros::Publisher controlPublisher;
bool drive = false;
vector<geometry_msgs::Pose> waypoints;
vector<geometry_msgs::Pose>::iterator wayIterator;

void stop() {
	drive = false;
	lunabotics::Control controlMsg;
	controlMsg.motion.linear.x = 0;
	controlMsg.motion.angular.z = 0;
	controlPublisher.publish(controlMsg);
	waypoints.clear();
}
	

vector<a_star_node> getPath(geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, float &res)
{
	vector<a_star_node> graph;
	if (mapClient.call(mapService)) {
		stop();
		
		ROS_INFO("Got map from service (%ld nodes)", mapService.response.map.data.size());
		ROS_INFO("------------------------------------");
		for (unsigned int i = 0; i < mapService.response.map.info.height; i++) {
		    stringstream sstr;
			for (unsigned int j = 0; j < mapService.response.map.info.width; j++) {
				int8_t probability = mapService.response.map.data.at(i*mapService.response.map.info.width+j);
				sstr << static_cast<int>(probability) << " ";
			}
			ROS_INFO("%s", sstr.str().c_str());
		}
		ROS_INFO("\n");
		
		ROS_INFO("Looking for a path...");
		
		//Generate a path
		float resolution = mapService.response.map.info.resolution;
		res = resolution;
		
		int start_x = round(startPose.position.x/resolution);
		int start_y = round(startPose.position.y/resolution);
		int goal_x = round(goalPose.position.x/resolution);
		int goal_y = round(goalPose.position.y/resolution);
		
		a_star_graph pathPlan;
		graph = pathPlan.find_path(mapService.response.map.data, 
								mapService.response.map.info.width, 
								mapService.response.map.info.height,
								start_x, start_y, goal_x, goal_y);
		
		if (graph.size() == 0) {
			ROS_INFO("Path is not found");
		}			
		seq++;
	}
	else {
		ROS_ERROR("Failed to call service luna_map");
	}
	return graph;
}

void emergencyCallback(const lunabotics::Emergency& msg)
{
	//Use msg to stop driving if applicable
}

void telemetryCallback(const lunabotics::Telemetry& msg)
{    
	currentPose.position = msg.odometry.pose.pose.position;
	currentPose.orientation = msg.odometry.pose.pose.orientation;
	ROS_INFO("I am at (%.1f,%.1f)", currentPose.position.x, currentPose.position.y);
}

void autonomyCallback(const std_msgs::Bool& msg)
{
	//Use msg to toggle autonomy
	if (msg.data) {
			
		waypoints.clear();
		
		//Specify params
		geometry_msgs::Pose start;
		geometry_msgs::Pose goal;
		start.position.x = 0.1;
		start.position.y = 0.1;
		start.position.z = 0;
		start.orientation = tf::createQuaternionMsgFromYaw(0);
		goal.position.x = 0.4;
		goal.position.y = 0.4;
		goal.position.z = 0;
		goal.orientation = tf::createQuaternionMsgFromYaw(0);
		
		ROS_INFO("Requesting path between (%.1f,%.1f) and (%.1f,%.1f)",
				  start.position.x, start.position.y,
				  goal.position.x, goal.position.y);
		float resolution;
		vector<a_star_node> path = getPath(start, goal, resolution);
		
		if (path.size() > 0) {
			stringstream sstr;
			for (vector<a_star_node>::iterator it = path.begin(); it != path.end(); it++) {
				a_star_node node = *it;
				
				float x_m = node.x*resolution;
				float y_m = node.y*resolution;
				
				geometry_msgs::Pose waypoint;
				waypoint.position.x = x_m;
				waypoint.position.y = y_m;
				waypoint.position.z = 0;
				waypoint.orientation = tf::createQuaternionMsgFromYaw(0);
				waypoints.push_back(waypoint);
				sstr << "->(" << x_m << "," << y_m << ")";
			}
			wayIterator = waypoints.begin();
			ROS_INFO("Returned path: %s", sstr.str().c_str());
			geometry_msgs::Pose waypoint = waypoints.at(0);
			ROS_INFO("Heading towards (%.1f,%.1f)", waypoint.position.x, waypoint.position.y);
			drive = true;
		}
		else {
			ROS_INFO("Path is empty");
		}
	}
	else {
		stop();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_driver");
	ros::NodeHandle nodeHandle;
	ros::Subscriber emergencySubscriber = nodeHandle.subscribe("luna_alert", 256, emergencyCallback);
	ros::Subscriber autonomySubscriber = nodeHandle.subscribe("luna_auto", 256, autonomyCallback);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	controlPublisher = nodeHandle.advertise<lunabotics::Control>("luna_ctrl", 256);
	mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("luna_map");
	
	
	
	ROS_INFO("Driver ready"); 
	
	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		//Whenever needed send control message
		if (drive) {
			if (wayIterator < waypoints.end()) {
				geometry_msgs::Pose waypointPose = *wayIterator;
			
				if (isnan(waypointPose.position.x) || isnan(waypointPose.position.y)) {
					ROS_WARN("Waypoint position undetermined");
				}
				else if (isnan(currentPose.position.x) || isnan(currentPose.position.y)) {
					ROS_WARN("Current position undetermined");
				}
				else {
						
					double dx = waypointPose.position.x-currentPose.position.x;
					double dy = waypointPose.position.y-currentPose.position.y;
					double theta = tf::getYaw(waypointPose.orientation) - tf::getYaw(currentPose.orientation);
					
					//Reparametrization
					double rho = sqrt(pow(dx, 2)+pow(dy, 2));
					double beta = -atan2(dy, dx);
					double alpha = -(beta+theta);
					
					
					if (fabs(rho) < 0.05 && fabs(beta) < 0.1 && fabs(alpha) < 0.1) {
						wayIterator++;
						if (wayIterator >= waypoints.end()) {
							ROS_INFO("Route completed");
							stop();
						}
						else {
							geometry_msgs::Pose nextWaypointPose = *wayIterator;
							ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypointPose.position.x, nextWaypointPose.position.y);
						}
					}
					else {
					
						//Control law
						double v = Kp*rho;
						double w = Ka*alpha+Kb*beta;
						ROS_INFO("dx:%f dy:%f theta:%f rho:%f alpha:%f beta:%f v:%f w:%f", dx, dy, theta, rho, alpha, beta, v, w);
						
						lunabotics::Control controlMsg;
						controlMsg.motion.linear.x = v;
						controlMsg.motion.angular.z = w;
						controlPublisher.publish(controlMsg);
					}
				}
			}
			else {
				ROS_ERROR("Way iterator out of bounds");
				stop();
			}
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
