#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "utils.h"
#include "lunabotics/Emergency.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/ControlMode.h"
#include "lunabotics/Telemetry.h"
#include "lunabotics/Goal.h"
#include "lunabotics/PID.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Twist.h>
#include "planning/a_star_graph.h"
#include "planning/bezier_smooth.h"
#include "tf/tf.h"
#include "types.h"
#include <numeric>

using namespace std;

float angleAccuracy = 0.4;
float distanceAccuracy = 0.2;

//Locally exponentially stable when Kp > 0; Kb < 0; Ka-Kb > 0
#define Kp 	0.15
#define Ka	0.7
#define Kb	-0.05

enum SKID_STATE {
	STOPPED	= 0,
	DRIVING	= 1,
	TURNING	= 2
};

inline int sign(double value) {
	if (value > angleAccuracy) return 1;
	if (value < -angleAccuracy) return -1;
	return 0;
}

int seq = 0;
CTRL_MODE_TYPE controlMode;
SKID_STATE skidState;
nav_msgs::GetMap mapService;
ros::ServiceClient mapClient;
geometry_msgs::Pose currentPose;
ros::Publisher controlPublisher;
ros::Publisher controlParamsPublisher;
ros::Publisher pathPublisher;
lunabotics::PID pid;
bool drive = false;
pose_arr waypoints;
pose_arr::iterator wayIterator;
double y_err_prev = 0;
float linear_speed_limit = 1;
ros::Time y_err_time_prev;
vector<double> y_err_int;

void stop() {
	drive = false;
	lunabotics::Control controlMsg;
	controlMsg.motion.linear.x = 0;
	controlMsg.motion.angular.z = 0;
	controlPublisher.publish(controlMsg);
	lunabotics::ControlParams controlParamsMsg;
	controlParamsMsg.driving = drive;
	controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
	controlParamsPublisher.publish(controlParamsMsg);
	waypoints.clear();
	y_err_int.clear();
}

void finish_route() {
	ROS_INFO("Route completed");
	stop();
	
	//Send empty path to clear map in GUI
	nav_msgs::Path pathMsg;
	ros::Time now = ros::Time::now();
	pathMsg.header.stamp = now;
	pathMsg.header.seq = seq;
	pathMsg.header.frame_id = "1";
	seq++;
	pathPublisher.publish(pathMsg);
}

planning::path *getPath(geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, float &res)
{
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
		
		planning::path *graph = new planning::path(mapService.response.map.data, 
								mapService.response.map.info.width, 
								mapService.response.map.info.height,
								start_x, start_y, goal_x, goal_y);
								
		
		if (graph->allNodes().size() == 0) {
			ROS_INFO("Path is not found");
			planning::path *empty = new planning::path();
			return empty;
		}
		else {
			if (graph->cornerNodes().size() == 1) {
				ROS_INFO("Robot is at the goal");
			}
			return graph;
		}	
	}
	else {
		ROS_ERROR("Failed to call service luna_map");
	}
	planning::path *empty = new planning::path();
	return empty;
}

point_arr getSmoothPath(point_arr path) 
{
	unsigned int n = path.size();
	if (n >= 2) {
		point_arr firstPts;
		point_arr secondPts;
		planning::GetCurveControlPoints(path, firstPts, secondPts);
		
		point_arr bezierPts;
		
		for (unsigned int i = 0; i < n-1; i++) {
			ROS_INFO("=============== Segment %d ==============", i);
			point_arr ctrlPts;
			ctrlPts.push_back(path.at(i));
			ctrlPts.push_back(firstPts.at(i));
			ctrlPts.push_back(secondPts.at(i));
			ctrlPts.push_back(path.at(i+1));
			
			cout << "Ctrl pts: " 
			<< ctrlPts.at(0) << "; " << ctrlPts.at(1) << "; " 
			<< ctrlPts.at(2) << "; " << ctrlPts.at(3) << endl; 
			
			for (float u = 0; u < 1.0; u += 0.1) {
				geometry_msgs::Point point = planning::bezier_point(u, ctrlPts);
				bezierPts.push_back(point);
				ROS_INFO("u %f - %f,%f", u, point.x, point.y);
			}
		}
		return bezierPts;
	}
	return path;
}

void emergencyCallback(const lunabotics::Emergency& msg)
{
	//Use msg to stop driving if applicable
}

void telemetryCallback(const lunabotics::Telemetry& msg)
{    
	currentPose.position = msg.odometry.pose.pose.position;
	currentPose.orientation = msg.odometry.pose.pose.orientation;
}

void controlModeCallback(const lunabotics::ControlMode& msg)
{
	controlMode = (CTRL_MODE_TYPE)msg.mode;
	if (controlMode == ACKERMANN) {
		linear_speed_limit = msg.linear_speed_limit;
	}
	
	ROS_INFO("Switching control mode to %s", controlModeToString(controlMode).c_str());
					
}

void autonomyCallback(const std_msgs::Bool& msg)
{
	//Use msg to toggle autonomy
	if (msg.data) {
			
	}
	else {
		stop();
	}
}


void pidCallback(const lunabotics::PID& msg) 
{
	pid = msg;
}

void goalCallback(const lunabotics::Goal& msg) 
{
	waypoints.clear();
	y_err_int.clear();
	
	angleAccuracy = msg.angleAccuracy;
	distanceAccuracy = msg.distanceAccuracy;
	
	//Specify params
	geometry_msgs::Pose goal;
	goal.position = msg.point;
	goal.orientation = tf::createQuaternionMsgFromYaw(0);
	
	ROS_INFO("Requesting path between (%.1f,%.1f) and (%.1f,%.1f)",
			  currentPose.position.x, currentPose.position.y,
			  goal.position.x, goal.position.y);
	float resolution;
	planning::path *path = getPath(currentPose, goal, resolution);
	
	if (path->is_initialized()) {
		stringstream sstr;
		
		nav_msgs::Path pathMsg;
		ros::Time now = ros::Time::now();
		pathMsg.header.stamp = now;
		pathMsg.header.seq = seq;
		pathMsg.header.frame_id = "1";
		seq++;
		
		
		point_arr knots = path->cornerPoints(resolution);
		point_arr pts;
		
		if (controlMode == ACKERMANN) {
			ROS_WARN("STARTING SMOOTH PATH for %d knots", (int)knots.size());
			pts = getSmoothPath(knots);
			
			ROS_WARN("SMOOTH PATH PASSED");
		}
		else {
			pts = knots;
		}
		
		int poseSeq = 0;
		for (point_arr::iterator it = pts.begin(); it != pts.end(); it++) {
			geometry_msgs::Point pt = *it;
			
			//float x_m = node.x*resolution;
			//float y_m = node.y*resolution;
			float x_m = pt.x;
			float y_m = pt.y;
			
			geometry_msgs::Pose waypoint;
			waypoint.position = pt;
			//waypoint.position.x = x_m;
			//waypoint.position.y = y_m;
			//waypoint.position.z = 0;
			waypoint.orientation = tf::createQuaternionMsgFromYaw(0);
			waypoints.push_back(waypoint);
			sstr << "->(" << x_m << "," << y_m << ")";
			
			
			geometry_msgs::PoseStamped pose;
			pose.header.seq = poseSeq++;
			pose.header.stamp = now;
			pose.header.frame_id = "1";
			pose.pose = waypoint;
			pathMsg.poses.push_back(pose);
		}
		
		wayIterator = controlMode == ACKERMANN ? waypoints.begin() : waypoints.begin()+1;		
		ROS_INFO("Returned path: %s", sstr.str().c_str());
		geometry_msgs::Pose waypoint = waypoints.at(0);
		ROS_INFO("Heading towards (%.1f,%.1f)", (*wayIterator).position.x, (*wayIterator).position.y);
		
		pathPublisher.publish(pathMsg);
		
		drive = true;
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.driving = drive;
		controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
		controlParamsPublisher.publish(controlParamsMsg);
	}
	else {
		ROS_INFO("Path is empty");
	}
	
	delete path;
}

void controlSkid(geometry_msgs::Pose waypointPose) 
{
	double dx = waypointPose.position.x-currentPose.position.x;
	double dy = waypointPose.position.y-currentPose.position.y;
	double angle = normalize_angle(atan2(dy, dx)-tf::getYaw(currentPose.orientation));
	
	
	lunabotics::Control controlMsg;
	
	switch (skidState) {
		case STOPPED: {
			ROS_INFO("SKID: stopped        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			controlMsg.motion.linear.x = 0;
			controlMsg.motion.angular.z = 0;
			
			if (fabs(dx) < distanceAccuracy && fabs(dy) < distanceAccuracy) {
				wayIterator++;
				if (wayIterator >= waypoints.end()) {
					finish_route();
				}
				else {
					lunabotics::ControlParams controlParamsMsg;
					controlParamsMsg.driving = drive;
					controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;	
					controlParamsPublisher.publish(controlParamsMsg);
					geometry_msgs::Pose nextWaypointPose = *wayIterator;
					ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypointPose.position.x, nextWaypointPose.position.y);
				}
			}
			else if (fabs(angle) > angleAccuracy) {
				skidState = TURNING;
			}
			else if (fabs(dx) > distanceAccuracy || fabs(dy) > distanceAccuracy) {
				skidState = DRIVING;
			}				
		}	
		break;
		case DRIVING: {	
		//	ROS_INFO("SKID: driving        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);	
			if (fabs(dx) < distanceAccuracy && fabs(dy) < distanceAccuracy) {
				skidState = STOPPED;
				controlMsg.motion.linear.x = 0;
				controlMsg.motion.angular.z = 0;
			}
			else if (fabs(angle) > angleAccuracy) {
				skidState = TURNING;
			}	
			else {
				controlMsg.motion.linear.x = 1;
				controlMsg.motion.angular.z = 0;
			}
		}
		break;
		case TURNING: {
			ROS_INFO("SKID: turning        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			int direction = sign(angle);
		
			if (direction == 0) {
				skidState = STOPPED;
				controlMsg.motion.linear.x = 0;
				controlMsg.motion.angular.z = 0;
			}
			else {
				ROS_INFO("SKID: %s", direction == -1 ? "Right" : "Left");
				controlMsg.motion.linear.x = 0;
				controlMsg.motion.angular.z = 1*direction;
			}	
		}
		break;
		default: break;
	}
	
	controlPublisher.publish(controlMsg);
}

void controlAckermann(geometry_msgs::Pose waypointPose)
{
	//////////////////////////////// ARC-TURN WITH DIFFERENTIAL DRIVE TEST /////////////////////
	
	if (wayIterator >= waypoints.end()) {
		finish_route();
	}
	
	if (waypoints.size() >= 2) {
		pose_arr::iterator closestWaypoint1 = waypoints.begin();
		pose_arr::iterator closestWaypoint2 = waypoints.begin()+1;
		double dist1 = point_distance(currentPose.position, (*closestWaypoint1).position);
		double dist2 = point_distance(currentPose.position, (*closestWaypoint2).position);
		if (dist2 < dist1) {
			//Swap values to keep waypoint1 always the closest one
			double tmp_dist = dist1;
			dist1 = dist2;
			dist2 = tmp_dist;
			pose_arr::iterator tmp_waypoint = closestWaypoint1;
			closestWaypoint1 = closestWaypoint2;
			closestWaypoint2 = tmp_waypoint;
		}
		for (pose_arr::iterator it = waypoints.begin()+2; it < waypoints.end(); it++) {
			double dist = point_distance(currentPose.position, (*it).position);
			if (dist < dist1) {
				dist2 = dist1, closestWaypoint2 = closestWaypoint1;
				dist1 = dist, closestWaypoint1 = it;
			}
			else if (dist < dist2) {
				dist2 = dist, closestWaypoint2 = it;
			}
		}
		if (closestWaypoint1 == wayIterator) {
			wayIterator++;
		}
		double length = point_distance((*closestWaypoint1).position, (*closestWaypoint2).position);
		double angle = angle_between_line_and_curr_pos(length, dist1, dist2);
		double y_err = distance_to_line(dist1, angle);		
		geometry_msgs::Point closestTrajectoryPoint = closest_trajectory_point(length, dist1, angle, (*closestWaypoint1).position, (*closestWaypoint2).position);;
		
		
		ROS_INFO("local %f,%f | closest %f,%f | one %f,%f | two %f,%f | Y_err %f", currentPose.position.x,currentPose.position.y, closestTrajectoryPoint.x,closestTrajectoryPoint.y,(*closestWaypoint1).position.x,(*closestWaypoint1).position.y,(*closestWaypoint2).position.x,(*closestWaypoint2).position.y, y_err);
		
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.trajectory_point = closestTrajectoryPoint;
		controlParamsMsg.y_err = y_err;
		controlParamsMsg.driving = drive;
		controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
		controlParamsPublisher.publish(controlParamsMsg);
	/*	
		if (fabs(angle) > M_PI_2) {
			//Trajectory is not on the side. First need to reach starting point
			controlSkid(*closestWaypoint1);
		}
		else {
		*/
		
		if (wayIterator >= waypoints.end()) {
			finish_route();
			return;
		}
		
		//double closestTrajectoryPointAngle = atan2(closestTrajectoryPoint.y-currentPose.position.y, closestTrajectoryPoint.x-currentPose.position.x);
		double closestTrajectoryPointAngle = atan2((*wayIterator).position.y-currentPose.position.y, (*wayIterator).position.x-currentPose.position.x);
		double angle_diff = closestTrajectoryPointAngle - tf::getYaw(currentPose.orientation);
		angle_diff = normalize_angle(angle_diff);
		
		 //goalAngle - closestTrajectoryPointAngle;
		if (angle_diff > 0) {
			y_err *= -1;
		}
		
		
		
			//Control law
					
			ros::Time now = ros::Time::now();
			ros::Duration dt = now - y_err_time_prev;
			y_err_time_prev = now;
			if (dt.toSec() != 0 && !isnan(y_err)) {
				double d_y_err = y_err - y_err_prev / dt.toSec();
				
				if (y_err_int.size() == 10) {
					y_err_int.erase(y_err_int.begin());
				}
				y_err_int.push_back(y_err);
				double i_y_err = accumulate(y_err_int.begin(), y_err_int.end(), 0);
				
				double dw = pid.p*y_err + pid.i*i_y_err + pid.d*d_y_err;
				
				ROS_INFO("y_err:%f, d_y_err:%f, i_y_err:%f, dw:%f",y_err, d_y_err, i_y_err, dw);
				
				float v = linear_speed_limit;
				if (isnan(dw)) {
					dw = 0;
					v = 0;
				}
				else {
					//The higher angular speed, the lower linear speed is
					#pragma message("This top w is for stage only");
					double top_w = 1.57;
					v = linear_speed_limit * std::max(0.0, (top_w-fabs(dw)))/top_w;
				}
				
				lunabotics::Control controlMsg;
				controlMsg.motion.linear.x = v;
				controlMsg.motion.angular.z = dw;
				controlPublisher.publish(controlMsg);
			}
			y_err_prev = y_err;
	//	}
	}
	else {
		//No need for curvature, just straight driving
		controlSkid(waypointPose);
	}
	
	return;
	
	///////////////////////////////////////////////////////////////////
	
	
	
	
	
	
	
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
			finish_route();
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_driver");
	ros::NodeHandle nodeHandle;
	
	y_err_time_prev = ros::Time::now();
	
	pid.p = 2;
	pid.i = 0.1;
	pid.d = 1;
	
	ros::Subscriber emergencySubscriber = nodeHandle.subscribe("luna_alert", 256, emergencyCallback);
	ros::Subscriber autonomySubscriber = nodeHandle.subscribe("luna_auto", 1, autonomyCallback);
	ros::Subscriber telemetrySubscriber = nodeHandle.subscribe("luna_tm", 256, telemetryCallback);
	ros::Subscriber goalSubscriber = nodeHandle.subscribe("luna_goal", 256, goalCallback);
	ros::Subscriber pidSubscriber = nodeHandle.subscribe("luna_pid", sizeof(float)*3, pidCallback);
	ros::Subscriber controlModeSubscriber = nodeHandle.subscribe("luna_ctrl_mode", 1, controlModeCallback);
	controlPublisher = nodeHandle.advertise<lunabotics::Control>("luna_ctrl", 256);
	pathPublisher = nodeHandle.advertise<nav_msgs::Path>("luna_path", 256);
	controlParamsPublisher = nodeHandle.advertise<lunabotics::ControlParams>("luna_ctrl_params", 256);
	mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("luna_map");
	
	
	
	ROS_INFO("Driver ready"); 
	
	ros::Rate loop_rate(200);
	while (ros::ok()) {
		
		/*
		
	/////////////////////////////////////////////////////////////
	if (!drive) {
		nav_msgs::Path pathMsg;
		geometry_msgs::PoseStamped waypoint1;
		waypoint1.pose.position.x = 3;
		waypoint1.pose.position.y = 3;
		waypoint1.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		geometry_msgs::PoseStamped waypoint2;
		waypoint2.pose.position.x = 5;
		waypoint2.pose.position.y = 5;
		waypoint2.pose.orientation = tf::createQuaternionMsgFromYaw(0);
		pathMsg.poses.push_back(waypoint1);
		pathMsg.poses.push_back(waypoint2);
		pathPublisher.publish(pathMsg);
		
		
		
		geometry_msgs::Pose closestWaypoint1;ACKERMANN
		closestWaypoint1.position.x = 3;
		closestWaypoint1.position.y = 3;
		geometry_msgs::Pose closestWaypoint2;
		closestWaypoint2.position.x = 5;
		closestWaypoint2.position.y = 5;
		double dist1 = point_distance(currentPose.position, closestWaypoint1.position);
		double dist2 = point_distance(currentPose.position, closestWaypoint2.position);
		if (dist2 < dist1) {
			//Swap values to keep waypoint1 always the closest one
			double tmp_dist = dist1;
			dist1 = dist2;
			dist2 = tmp_dist;
			geometry_msgs::Pose tmp_waypoint = closestWaypoint1;
			closestWaypoint1 = closestWaypoint2;
			closestWaypoint2 = tmp_waypoint;
		}
		double length = point_distance(closestWaypoint1.position, closestWaypoint2.position);
		double angle = angle_between_line_and_curr_pos(length, dist1, dist2);
		double y_err = distance_to_line(dist1, angle);		
		geometry_msgs::Point closestTrajectoryPoint = closest_trajectory_point(length, dist1, angle, closestWaypoint1.position, closestWaypoint2.position);
		double closestTrajectoryPointAngle = atan2(closestTrajectoryPoint.y-currentPose.position.y, closestTrajectoryPoint.x-currentPose.position.x);
		//double goalAngle = atan2((*wayIterator).position.y-currentPose.position.y, (*wayIterator).position.x-currentPose.position.x);
		double angle_diff = closestTrajectoryPointAngle - tf::getYaw(currentPose.orientation);
		angle_diff = normalize_angle(angle_diff);
		
		 //goalAngle - closestTrajectoryPointAngle;
		if (angle_diff < 0) {
			y_err *= -1;
		}
		
		ROS_INFO("angle to closest %f, heading %f, diff %f", closestTrajectoryPointAngle, tf::getYaw(currentPose.orientation), angle_diff); 
		
		//ROS_INFO("local %f,%f | closest %f,%f | one %f,%f | two %f,%f | Y_err %f", currentPose.position.x,currentPose.position.y, closestTrajectoryPoint.x,closestTrajectoryPoint.y,closestWaypoint1.position.x,closestWaypoint1.position.y,closestWaypoint2.position.x,closestWaypoint2.position.y, y_err);
		
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.trajectory_point = closestTrajectoryPoint;
		controlParamsMsg.y_err = y_err;
		controlParamsMsg.driving = true;
		controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
		controlParamsPublisher.publish(controlParamsMsg);
		
		
		
		ros::spinOnce();
		loop_rate.sleep();
		
		
		continue;
	}
		
		
	/////////////////////////////////////////////////////////////
		*/
		
		
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
					switch (controlMode) {
						case ACKERMANN: controlAckermann(waypointPose); break;
						case TURN_IN_SPOT: controlSkid(waypointPose); break;
						case LATERAL: break;
						default: break;
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
