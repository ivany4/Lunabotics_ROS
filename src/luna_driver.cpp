#include "ros/ros.h"
#include <numeric>

//Topics
#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "lunabotics/Emergency.h"
#include "lunabotics/Control.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/ControlMode.h"
#include "lunabotics/State.h"
#include "lunabotics/Goal.h"
#include "lunabotics/PID.h"
#include "lunabotics/ICRControl.h"
#include "lunabotics/AllWheelCommon.h"
#include "lunabotics/RobotGeometry.h"
#include "lunabotics/AllWheelState.h"

//Transforms
#include <tf/transform_listener.h>
#include "message_filters/subscriber.h"

//Protos
#include "../protos_gen/AllWheelControl.pb.h"
#include "../protos_gen/Telemetry.pb.h"

//Helpers
#include "planning/a_star_graph.h"
#include "planning/bezier_smooth.h"
#include "control/PIDController.h"
#include "control/AllWheelPredefinedCmdController.h"
#include "geometry/geometry.h"
#include "types.h"

#define ROBOT_DIFF_DRIVE	0

float angleAccuracy = 0.4;
float distanceAccuracy = 0.2;

//Locally exponentially stable when Kp > 0; Kb < 0; Ka-Kb > 0
#define Kp 	0.15
#define Ka	0.7
#define Kb	-0.05

inline int sign(double value) {
	if (value > angleAccuracy) return 1;
	if (value < -angleAccuracy) return -1;
	return 0;
}

int seq = 0;
int bezierSegments = 20;
lunabotics::proto::SteeringModeType controlMode = lunabotics::proto::ACKERMANN;
lunabotics::proto::Telemetry::PointTurnState skidState = lunabotics::proto::Telemetry::STOPPED;
lunabotics::PredefinedCmdControllerPtr predCtrl;
lunabotics::PIDControllerPtr pidController;
lunabotics::AllWheelGeometryPtr allWheelGeometry;
lunabotics::PIDGeometry pidGeometry;
lunabotics::Pose currentPose;
lunabotics::PointArr waypoints;
lunabotics::PointArr::iterator wayIterator;
nav_msgs::GetMap mapService;
ros::ServiceClient mapClient;
ros::Publisher controlPublisher;
ros::Publisher controlParamsPublisher;
ros::Publisher pathPublisher;
ros::Publisher allWheelPublisher;
ros::Publisher allWheelCommonPublisher;
ros::Publisher geometryPublisher;
ros::Publisher ICRPublisher;
geometry_msgs::Twist velocities;
bool autonomyEnabled = false;
float linear_speed_limit = 1;
bool jointStatesAcquired = false;

void stop() {
	autonomyEnabled = false;
#if ROBOT_DIFF_DRIVE
	lunabotics controlMsg;
	controlMsg.motion.linear.x = 0;
	controlMsg.motion.linear.y = 0;
	controlMsg.motion.linear.z = 0;
	controlMsg.motion.angular.x = 0;
	controlMsg.motion.angular.y = 0;
	controlMsg.motion.angular.z = 0;
	controlPublisher.publish(controlMsg);
#else
	lunabotics::AllWheelCommon msg;
	msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
	allWheelCommonPublisher.publish(msg);
#endif
	lunabotics::ControlParams controlParamsMsg;
	controlParamsMsg.driving = autonomyEnabled;
	controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
	controlParamsPublisher.publish(controlParamsMsg);
	waypoints.clear();
}

void finish_route() {
	//ROS_INFO("Route completed");
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

lunabotics::Path *getPath(lunabotics::Pose startPose, lunabotics::Point goalPoint, float &res)
{
	if (mapClient.call(mapService)) {
		stop();
		
		unsigned int map_size = mapService.response.map.data.size();
		if (map_size > 0) {
			ROS_INFO("Got map from service (%d cells)", (int)map_size);
			ROS_INFO("Looking for a path...");
			
			//Generate a path
			float resolution = mapService.response.map.info.resolution;
			res = resolution;
			
			int start_x = round(startPose.position.x/resolution);
			int start_y = round(startPose.position.y/resolution);
			int goal_x = round(goalPoint.x/resolution);
			int goal_y = round(goalPoint.y/resolution);
			
			lunabotics::Path *graph = new lunabotics::Path(mapService.response.map.data, 
									mapService.response.map.info.width, 
									mapService.response.map.info.height,
									start_x, start_y, goal_x, goal_y);
									
			
			if (graph->allNodes().size() == 0) {
				ROS_INFO("Path is not found");
				lunabotics::Path *empty = new lunabotics::Path();
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
			ROS_ERROR("Failed to get a proper map from the service");
		}	
	}
	else {
		ROS_ERROR("Failed to call service luna_map");
	}
	lunabotics::Path *empty = new lunabotics::Path();
	return empty;
}

void emergencyCallback(const lunabotics::Emergency& msg)
{
	//Use msg to stop driving if applicable
}

void stateCallback(const lunabotics::State& msg)
{    
	//ROS_INFO("Pose updated");
	currentPose = lunabotics::Pose_from_geometry_msgs_Pose(msg.odometry.pose.pose);
	velocities = msg.odometry.twist.twist;
	pidGeometry.setLinearVelocity(velocities.linear.x);
	pidGeometry.setCurrentPose(currentPose);
}

void controlModeCallback(const lunabotics::ControlMode& msg)
{
	controlMode = (lunabotics::proto::SteeringModeType)msg.mode;
	if (controlMode == lunabotics::proto::ACKERMANN) {
		linear_speed_limit = msg.linear_speed_limit;
		bezierSegments = (int)msg.smth_else;
	}
	
	ROS_INFO("Switching control mode to %s", controlModeToString(controlMode).c_str());
}

void allWheelStateCallback(const lunabotics::AllWheelState& msg)
{
	predCtrl->giveFeedback(msg);
}

void allWheelCommonCallback(const lunabotics::AllWheelCommon& msg)
{
	predCtrl->setNewCommand((lunabotics::proto::AllWheelControl::PredefinedControlType)msg.predefined_cmd);
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
	pidController->setP(msg.p);
	pidController->setI(msg.i);
	pidController->setD(msg.d);
	pidGeometry.setVelocityMultiplier(msg.velocity_multiplier);
	pidGeometry.setVelocityOffset(msg.velocity_offset);
}

void ICRCallback(const lunabotics::ICRControl& msg) 
{		
	geometry_msgs::Point ICRMsg = msg.ICR;
	ICRPublisher.publish(ICRMsg);
	
	lunabotics::Point ICR = lunabotics::Point_from_geometry_msgs_Point(msg.ICR);
	
	float angle_front_left;
	float angle_front_right;
	float angle_rear_left;
	float angle_rear_right;
	if (allWheelGeometry->calculateAngles(ICR, angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
		if (lunabotics::validateAngles(angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
			lunabotics::AllWheelState controlMsg;
			controlMsg.steering.left_front = angle_front_left;
			controlMsg.steering.right_front = angle_front_right;
			controlMsg.steering.left_rear = angle_rear_left;
			controlMsg.steering.right_rear = angle_rear_right;
			float vel_front_left;
			float vel_front_right;
			float vel_rear_left;
			float vel_rear_right;
			if (fabs(ICR.x) < 0.001 && fabs(ICR.y) < 0.001) {
				//Point turn
				vel_front_left = vel_rear_right = msg.velocity;
				vel_front_right = vel_rear_left = -msg.velocity;
			}
			else if (!allWheelGeometry->calculateVelocities(ICR, msg.velocity, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)) {
				vel_front_left = vel_front_right = vel_rear_left = vel_rear_right = 0;			
			}
			controlMsg.driving.left_front = vel_front_left;
			controlMsg.driving.right_front = vel_front_right;
			controlMsg.driving.left_rear = vel_rear_left;
			controlMsg.driving.right_rear = vel_rear_right;
			
			
			ROS_INFO("VELOCITY %.2f | %.2f | %.2f | %.2f", vel_front_left, vel_front_right, vel_rear_left, vel_rear_right);
			
			allWheelPublisher.publish(controlMsg);
		}
		else {
			ROS_WARN("ICR position is in the dead zone, this control is not possible!");
		}
	}
}

void goalCallback(const lunabotics::Goal& msg) 
{
	waypoints.clear();
	
	angleAccuracy = msg.angleAccuracy;
	distanceAccuracy = msg.distanceAccuracy;
	
	//Specify params
	lunabotics::Point goal = lunabotics::Point_from_geometry_msgs_Point(msg.point);
		//ROS_INFO("Requesting path between (%.1f,%.1f) and (%.1f,%.1f)",
		//	  currentPose.position.x, currentPose.position.y,
		//	  goal.x, goal.y);
	float resolution;
	lunabotics::Path *path = getPath(currentPose, goal, resolution);
	
	if (path->is_initialized()) {
		std::stringstream sstr;
		
		nav_msgs::Path pathMsg;
		ros::Time now = ros::Time::now();
		pathMsg.header.stamp = now;
		pathMsg.header.seq = seq;
		pathMsg.header.frame_id = "map";
		seq++;
		
		
		lunabotics::PointArr corner_points = path->cornerPoints(resolution);
		lunabotics::PointArr pts;
		
		if (controlMode == lunabotics::proto::ACKERMANN) {
			unsigned int size = corner_points.size();
			if (size > 2) {
				lunabotics::Point startPoint = corner_points.at(0);
				lunabotics::Point endPoint = corner_points.at(size-1);
				lunabotics::IndexedPointArr closest_obstacles = path->closestObstaclePoints(resolution);
				unsigned int obst_size = closest_obstacles.size();
				
				pts.push_back(startPoint);
			
				//Get bezier quadratic curves for each point-turn
				for (unsigned int i = 1; i < size-1; i++) {
					lunabotics::Point q0, q2;
					lunabotics::Point prev = corner_points.at(i-1);
					lunabotics::Point curr = corner_points.at(i);
					lunabotics::Point next = corner_points.at(i+1);
					
					bool hasObstacle = false;
					lunabotics::Point obstaclePoint;
					
					//Since obstacle is the center of occupied cell, we want p to be at its edge
					if (obst_size > 0) {
						int start = std::min(obst_size-1, i-1);
						for (int j = start; j >= 0; j--) {
							lunabotics::IndexedPoint indexedObstacle = closest_obstacles.at(j);
							if (indexedObstacle.index == (int)i) {
								hasObstacle = true;
								obstaclePoint = indexedObstacle.point;
								break;
							}
						}
					}
					
					
					if (i == 1) {
						q0 = prev;
					}
					else {
						q0 = lunabotics::midPoint(prev, curr);
					}
					if (i == size-2) {
						q2 = next;
					}
					else {
						q2 = lunabotics::midPoint(next, curr);
					}
					
					lunabotics::PointArr curve;
					if (hasObstacle) {
						lunabotics::Point p = lunabotics::midPoint(obstaclePoint, curr);
						curve = lunabotics::trajectory_bezier(q0, curr, q2, p, bezierSegments);
						//ROS_INFO("Curve from tetragonal q0=(%f,%f) q1=(%f,%f), q2=(%f,%f), p=(%f,%f)", q0.x, q0.y, curr.x, curr.y, q2.x, q2.y, p.x, p.y);
					}
					else {
						curve = lunabotics::quadratic_bezier(q0, curr, q2, bezierSegments);
						//ROS_INFO("Curve without tetragonal q0=(%f,%f) q1=(%f,%f), q2=(%f,%f)", q0.x, q0.y, curr.x, curr.y, q2.x, q2.y);
					}
					
					//Append curve points
					pts.insert(pts.end(), curve.begin(), curve.end());
				}	
				pts.push_back(endPoint);	
			}
			else {
				pts = corner_points;
			}
		}
		else {
			pts = corner_points;
		}
		
		int poseSeq = 0;
		for (lunabotics::PointArr::iterator it = pts.begin(); it != pts.end(); it++) {
			lunabotics::Point pt = *it;
			
			//float x_m = node.x*resolution;
			//float y_m = node.y*resolution;
			//float x_m = pt.x;
			//float y_m = pt.y;
			
			waypoints.push_back(pt);
	//		sstr << "->(" << x_m << "," << y_m << ")";
			
			
			geometry_msgs::PoseStamped pose;
			pose.header.seq = poseSeq++;
			pose.header.stamp = now;
			pose.header.frame_id = "map";
			pose.pose.position = lunabotics::geometry_msgs_Point_from_Point(pt);
			pose.pose.orientation =  tf::createQuaternionMsgFromYaw(0);
			pathMsg.poses.push_back(pose);
		}
		pidGeometry.setTrajectory(pts);
		
		wayIterator = controlMode == lunabotics::proto::ACKERMANN ? waypoints.begin() : waypoints.begin()+1;		
	//	ROS_INFO("Returned path: %s", sstr.str().c_str());
		//lunabotics::Point waypoint = waypoints.at(0);
		//ROS_INFO("Heading towards (%.1f,%.1f)", (*wayIterator).position.x, (*wayIterator).position.y);
		
		pathPublisher.publish(pathMsg);
		
		autonomyEnabled = true;
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.driving = autonomyEnabled;
		controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
		controlParamsPublisher.publish(controlParamsMsg);
	}
	else {
		ROS_INFO("Path is empty");
	}
	
	delete path;
}


void controlSkidAllWheel() 
{
	if (wayIterator >= waypoints.end()) {
		//ROS_INFO("Finishing route");
		finish_route();
		return;
	}
	double dx = (*wayIterator).x-currentPose.position.x;
	double dy = (*wayIterator).y-currentPose.position.y;
	double angle = lunabotics::normalizedAngle(atan2(dy, dx)-currentPose.orientation);
	
	lunabotics::AllWheelCommon msg;

	lunabotics::ControlParams controlParamsMsg;
	controlParamsMsg.driving = autonomyEnabled;
	
	switch (skidState) {
		case lunabotics::proto::Telemetry::STOPPED: {
			//ROS_INFO("SKID: stopped dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			
			if (fabs(dx) < distanceAccuracy && fabs(dy) < distanceAccuracy) {
				wayIterator++;
				if (wayIterator >= waypoints.end()) {
					finish_route();
				}
				else {
					controlParamsMsg.has_trajectory_data = true;
					controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;	
					//lunabotics::Point nextWaypoint = *wayIterator;
					//ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypoint.position.x, nextWaypoint.position.y);
				}
			}
			else if (fabs(angle) > angleAccuracy) {
				skidState = lunabotics::proto::Telemetry::TURNING;
			}
			else if (fabs(dx) > distanceAccuracy || fabs(dy) > distanceAccuracy) {
				skidState = lunabotics::proto::Telemetry::DRIVING;
			}				
		}	
		break;
		case lunabotics::proto::Telemetry::DRIVING: {	
			//ROS_INFO("SKID: driving        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			if (fabs(dx) < distanceAccuracy && fabs(dy) < distanceAccuracy) {
				skidState = lunabotics::proto::Telemetry::STOPPED;
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
			}
			else if (fabs(angle) > angleAccuracy) {
				skidState = lunabotics::proto::Telemetry::TURNING;
			}	
			else {
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::DRIVE_FORWARD;
			}
		}
		break;
		case lunabotics::proto::Telemetry::TURNING: {
			int direction = sign(angle);
			//ROS_INFO("SKID: turning  %d (%.2f-%.2f)      dx: %.5f dy: %.5f angle: %.5f", direction, angle, angleAccuracy, dx, dy, angle);
		
			if (direction == 0) {
				skidState = lunabotics::proto::Telemetry::STOPPED;
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
			}
			else {
				//ROS_INFO("SKID: %s", direction == -1 ? "Right" : "Left");
				if (direction == -1) {
					msg.predefined_cmd = lunabotics::proto::AllWheelControl::TURN_CW;
				}
				else {
					msg.predefined_cmd = lunabotics::proto::AllWheelControl::TURN_CCW;
				}
			}	
		}
		break;
		default: break;
	}
	
	controlParamsMsg.point_turn_state = skidState;
	controlParamsMsg.has_point_turn_state = true;
	controlParamsPublisher.publish(controlParamsMsg);
	allWheelCommonPublisher.publish(msg);
}

void controlSkidDiffDrive() 
{
	if (wayIterator >= waypoints.end()) {
		finish_route();
		return;
	}
	double dx = (*wayIterator).x-currentPose.position.x;
	double dy = (*wayIterator).y-currentPose.position.y;
	double angle = lunabotics::normalizedAngle(atan2(dy, dx)-currentPose.orientation);
	
	
	lunabotics::ControlParams controlParamsMsg;
	controlParamsMsg.driving = autonomyEnabled;
	controlParamsMsg.has_point_turn_state = true;
	lunabotics::Control controlMsg;
	controlMsg.motion.linear.x = 0;
	controlMsg.motion.linear.y = 0;
	controlMsg.motion.linear.z = 0;
	controlMsg.motion.angular.x = 0;
	controlMsg.motion.angular.y = 0;
	controlMsg.motion.angular.z = 0;
					
	switch (skidState) {
		case lunabotics::proto::Telemetry::STOPPED: {
			//ROS_INFO("SKID: stopped        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			
			if (fabs(dx) < distanceAccuracy && fabs(dy) < distanceAccuracy) {
				wayIterator++;
				if (wayIterator >= waypoints.end()) {
					finish_route();
				}
				else {
					controlParamsMsg.has_trajectory_data = true;
					controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
					//lunabotics::Point nextWaypoint = *wayIterator;
					//ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypoint.position.x, nextWaypoint.position.y);
				}
			}
			else if (fabs(angle) > angleAccuracy) {
				skidState = lunabotics::proto::Telemetry::TURNING;
			}
			else if (fabs(dx) > distanceAccuracy || fabs(dy) > distanceAccuracy) {
				skidState = lunabotics::proto::Telemetry::DRIVING;
			}				
		}	
		break;
		case lunabotics::proto::Telemetry::DRIVING: {	
		//	ROS_INFO("SKID: driving        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);	
			if (fabs(dx) < distanceAccuracy && fabs(dy) < distanceAccuracy) {
				skidState = lunabotics::proto::Telemetry::STOPPED;
				controlMsg.motion.linear.x = 0;
				controlMsg.motion.angular.z = 0;
			}
			else if (fabs(angle) > angleAccuracy) {
				skidState = lunabotics::proto::Telemetry::TURNING;
			}	
			else {
				controlMsg.motion.linear.x = 1;
				controlMsg.motion.angular.z = 0;
			}
		}
		break;
		case lunabotics::proto::Telemetry::TURNING: {
			//ROS_INFO("SKID: turning        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			int direction = sign(angle);
		
			if (direction == 0) {
				skidState = lunabotics::proto::Telemetry::STOPPED;
				controlMsg.motion.linear.x = 0;
				controlMsg.motion.angular.z = 0;
			}
			else {
				//ROS_INFO("SKID: %s", direction == -1 ? "Right" : "Left");
				controlMsg.motion.linear.x = 0;
				controlMsg.motion.angular.z = 1*direction;
			}	
		}
		break;
		default: break;
	}
	
	controlParamsMsg.point_turn_state = skidState;
	controlParamsPublisher.publish(controlParamsMsg);
	controlPublisher.publish(controlMsg);
}

void controlSkid() {
	//	ROS_INFO("Control mode point-turn");
#if ROBOT_DIFF_DRIVE
	controlSkidDiffDrive();
#else
	controlSkidAllWheel();
#endif
}

void controlAckermannAllWheel()
{
	//////////////////////////////// ARC-TURN WITH ALL-WHEEL STEERING TEST /////////////////////
	
	if (wayIterator >= waypoints.end()) {
		finish_route();
		return;
	}
	
	//If suddenly skipped a waypoint, proceed with the next ones, don't get stuck with current
	double dist = lunabotics::distance(*wayIterator, currentPose.position);
	for (lunabotics::PointArr::iterator it = wayIterator+1; it < waypoints.end(); it++) {
		double newDist = lunabotics::distance(*it, currentPose.position);
		if (newDist < dist) {
			wayIterator = it;
			dist = newDist;
		}
	}
	
	if (dist < distanceAccuracy) {
		wayIterator++;
	}
	if (wayIterator >= waypoints.end()) {
		finish_route();
		return;
	}
	
	
	double dx = (*wayIterator).x-currentPose.position.x;
	double dy = (*wayIterator).y-currentPose.position.y;
	
	if (waypoints.size() >= 2) {
		
		//In the beginning turn in place towards the second waypoint (first waypoint is at the robot's position). It helps to solve problems with pid
		if (wayIterator < waypoints.begin()+2) {
			wayIterator = waypoints.begin()+1;
			double angle = lunabotics::normalizedAngle(atan2(dy, dx)-currentPose.orientation);
			if (fabs(angle) > angleAccuracy) {
				//ROS_WARN("Facing away from the trajectory. Turning in place");
			//	controlSkid();
			//	return;
			}
		}
		
		
		double y_err = pidGeometry.getReferenceDistance();
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.trajectory_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getClosestTrajectoryPoint());
		controlParamsMsg.velocity_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getReferencePoint());
		controlParamsMsg.y_err = y_err;
		controlParamsMsg.driving = autonomyEnabled;
		controlParamsMsg.t_trajectory_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getClosestTrajectoryPointInLocalFrame());
		controlParamsMsg.t_velocity_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getReferencePointInLocalFrame());
		controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
		controlParamsMsg.has_trajectory_data = true;
		controlParamsPublisher.publish(controlParamsMsg);
		
		//Control law
		
		double signal;
		if (pidController->control(y_err, signal)) {
			signal *= 10.0;
			ROS_WARN("DW %.2f", signal);
			
			double gamma1 = -signal/2;
			
			if (fabs(gamma1) < 0.00001) {
				gamma1 = 0.01;
			}
			
			double alpha = M_PI_2-gamma1;
			double offset_x = allWheelGeometry->left_front().x;
			double offset_y = tan(alpha)*offset_x;
			
			lunabotics::Point ICR = lunabotics::CreatePoint(0, offset_y);
			ICR = allWheelGeometry->point_outside_base_link(ICR);
			ICRPublisher.publish(lunabotics::geometry_msgs_Point_from_Point(ICR));
			
			ROS_INFO("Alpha %.2f offset %.2f ICR %.2f", alpha, offset_y, ICR.y);
			
			float velocity = linear_speed_limit;
			
			float angle_front_left;
			float angle_front_right;
			float angle_rear_left;
			float angle_rear_right;
			if (allWheelGeometry->calculateAngles(ICR, angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
				
				//Set angles to the ones outside dead zone
				lunabotics::validateAngles(angle_front_left, angle_front_right, angle_rear_left, angle_rear_right);
				
				lunabotics::AllWheelState controlMsg;
				controlMsg.steering.left_front = angle_front_left;
				controlMsg.steering.right_front = angle_front_right;
				controlMsg.steering.left_rear = angle_rear_left;
				controlMsg.steering.right_rear = angle_rear_right;
				float vel_front_left;
				float vel_front_right;
				float vel_rear_left;
				float vel_rear_right;
				if (!allWheelGeometry->calculateVelocities(ICR, velocity, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)) {
					vel_front_left = vel_front_right = vel_rear_left = vel_rear_right = 0;			
				}
				controlMsg.driving.left_front = vel_front_left;
				controlMsg.driving.right_front = vel_front_right;
				controlMsg.driving.left_rear = vel_rear_left;
				controlMsg.driving.right_rear = vel_rear_right;
				
				allWheelPublisher.publish(controlMsg);
			}
		}
	}
	else {
		//No need for curvature, just straight driving
		controlSkid();
	}
	return;
	
	///////////////////////////////////////////////////////////////////
	
}

void controlAckermannDiffDrive()
{
	//////////////////////////////// ARC-TURN WITH DIFFERENTIAL DRIVE TEST /////////////////////
	
	if (wayIterator >= waypoints.end()) {
		finish_route();
		return;
	}
	
	//If suddenly skipped a waypoint, proceed with the next ones, don't get stuck with current
	double dist = lunabotics::distance(*wayIterator, currentPose.position);
	for (lunabotics::PointArr::iterator it = wayIterator+1; it < waypoints.end(); it++) {
		double newDist = lunabotics::distance(*it, currentPose.position);
		if (newDist < dist) {
			wayIterator = it;
			dist = newDist;
		}
	}
	
	if (dist < distanceAccuracy) {
		wayIterator++;
	}
	if (wayIterator >= waypoints.end()) {
		finish_route();
		return;
	}
	
	
	double dx = (*wayIterator).x-currentPose.position.x;
	double dy = (*wayIterator).y-currentPose.position.y;
	
	if (waypoints.size() >= 2) {
		
		//In the beginning turn in place towards the second waypoint (first waypoint is at the robot's position). It helps to solve problems with pid
		if (wayIterator < waypoints.begin()+2) {
			wayIterator = waypoints.begin()+1;
			double angle = lunabotics::normalizedAngle(atan2(dy, dx)-currentPose.orientation);
			if (fabs(angle) > angleAccuracy) {
				//ROS_WARN("Facing away from the trajectory. Turning in place");
				controlSkid();
				return;
			}
		}
		
		
		
		double y_err = pidGeometry.getReferenceDistance();
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.trajectory_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getClosestTrajectoryPoint());
		controlParamsMsg.velocity_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getReferencePoint());
		controlParamsMsg.y_err = y_err;
		controlParamsMsg.driving = autonomyEnabled;
		controlParamsMsg.t_trajectory_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getClosestTrajectoryPointInLocalFrame());
		controlParamsMsg.t_velocity_point = lunabotics::geometry_msgs_Point_from_Point(pidGeometry.getReferencePointInLocalFrame());
		controlParamsMsg.next_waypoint_idx = wayIterator < waypoints.end() ? wayIterator-waypoints.begin()+1 : 0;
		controlParamsMsg.has_trajectory_data = true;
		controlParamsPublisher.publish(controlParamsMsg);
		
		//Control law
		
		double dw;
		if (pidController->control(y_err, dw)) {
			dw *= -3;
			ROS_WARN("DW %.2f", dw);
			
			//The higher angular speed, the lower linear speed is
			double top_w = 1.57;
			double v = linear_speed_limit * std::max(0.0, (top_w-fabs(dw)))/top_w;
			v = std::max(0.01, v);
				
			lunabotics::Control controlMsg;
			controlMsg.motion.linear.x = v;
			controlMsg.motion.linear.y = 0;
			controlMsg.motion.linear.z = 0;
			controlMsg.motion.angular.x = 0;
			controlMsg.motion.angular.y = 0;
			controlMsg.motion.angular.z = dw;
			controlPublisher.publish(controlMsg);
		}
	}
	else {
		//No need for curvature, just straight driving
		controlSkid();
	}
	return;
	
	///////////////////////////////////////////////////////////////////
	
	
	
	
	
	
	double theta = currentPose.orientation;
	
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
			lunabotics::Point nextWaypoint = *wayIterator;
			ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypoint.x, nextWaypoint.y);
		}
	}
	else {
	
		//Control law
		double v = Kp*rho;
		double w = Ka*alpha+Kb*beta;
		ROS_INFO("dx:%f dy:%f theta:%f rho:%f alpha:%f beta:%f v:%f w:%f", dx, dy, theta, rho, alpha, beta, v, w);
		
		lunabotics::Control controlMsg;
		controlMsg.motion.linear.x = v;
		controlMsg.motion.linear.y = 0;
		controlMsg.motion.linear.z = 0;
		controlMsg.motion.angular.x = 0;
		controlMsg.motion.angular.y = 0;
		controlMsg.motion.angular.z = w;
		controlPublisher.publish(controlMsg);
	}
}

void controlAckermann() 
{
//	ROS_INFO("Control mode ackermann");
	#if ROBOT_DIFF_DRIVE
		controlAckermannDiffDrive();
	#else
		controlAckermannAllWheel();
	#endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "luna_driver");
	ros::NodeHandle nodeHandle("lunabotics");
	
	ros::Subscriber emergencySubscriber = nodeHandle.subscribe("emergency", 256, emergencyCallback);
	ros::Subscriber autonomySubscriber = nodeHandle.subscribe("autonomy", 1, autonomyCallback);
	ros::Subscriber stateSubscriber = nodeHandle.subscribe("state", 256, stateCallback);
	ros::Subscriber goalSubscriber = nodeHandle.subscribe("goal", 256, goalCallback);
	ros::Subscriber pidSubscriber = nodeHandle.subscribe("pid", sizeof(float)*3, pidCallback);
	ros::Subscriber controlModeSubscriber = nodeHandle.subscribe("control_mode", 1, controlModeCallback);
	ros::Subscriber ICRSubscriber = nodeHandle.subscribe("icr", sizeof(float)*3, ICRCallback);
	ros::Subscriber allWheelCommonSubscriber = nodeHandle.subscribe("all_wheel_common", 256, allWheelCommonCallback);
	ros::Subscriber allWheelStateSubscriber = nodeHandle.subscribe("all_wheel_feeback", sizeof(float)*8, allWheelStateCallback);
	ros::Publisher allWheelStateROSPublisher = nodeHandle.advertise<lunabotics::AllWheelState>("all_wheel", 256);
	controlPublisher = nodeHandle.advertise<lunabotics::Control>("control", 256);
	pathPublisher = nodeHandle.advertise<nav_msgs::Path>("path", 256);
	ICRPublisher = nodeHandle.advertise<geometry_msgs::Point>("icr_state", sizeof(float)*2);
	allWheelPublisher = nodeHandle.advertise<lunabotics::AllWheelState>("all_wheel", sizeof(float)*8);
	controlParamsPublisher = nodeHandle.advertise<lunabotics::ControlParams>("control_params", 256);
	mapClient = nodeHandle.serviceClient<nav_msgs::GetMap>("map");
	geometryPublisher = nodeHandle.advertise<lunabotics::RobotGeometry>("geometry", sizeof(float)*2*4);
	allWheelCommonPublisher = nodeHandle.advertise<lunabotics::AllWheelCommon>("all_wheel_common", sizeof(uint32_t));
	
	
	lunabotics::Point zeroPoint = lunabotics::CreatePoint(0, 0);
	allWheelGeometry = new lunabotics::AllWheelGeometry(zeroPoint, zeroPoint, zeroPoint, zeroPoint);
		
	pidController = new lunabotics::PIDController(0.05, 0.1, 0.18);
	predCtrl = new lunabotics::PredefinedCmdController();
	
	tf::TransformListener listener;
	
	ROS_INFO("Driver ready"); 
	
	ros::Rate loop_rate(200);
	while (ros::ok()) {
#if !ROBOT_DIFF_DRIVE
		tf::StampedTransform transform;
		lunabotics::Point point;
		if (!jointStatesAcquired) {
			try {
				listener.lookupTransform("left_front_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				allWheelGeometry->set_left_front(point);
				listener.lookupTransform("right_front_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				allWheelGeometry->set_right_front(point);
				listener.lookupTransform("left_rear_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				allWheelGeometry->set_left_rear(point);
				listener.lookupTransform("right_rear_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				allWheelGeometry->set_right_rear(point);
				listener.lookupTransform("left_front_wheel", "left_front_joint", ros::Time(0), transform);
				allWheelGeometry->set_wheel_offset(fabs(transform.getOrigin().y()));
				listener.lookupTransform("left_front_wheel_radius", "left_front_wheel", ros::Time(0), transform);
				allWheelGeometry->set_wheel_radius(fabs(transform.getOrigin().x()));
				allWheelGeometry->set_wheel_width(fabs(transform.getOrigin().y()));
				
				predCtrl->setGeometry(allWheelGeometry);
				
				jointStatesAcquired = true;
			}
			catch (tf::TransformException e) {
				ROS_ERROR("%s", e.what());
			}
		}
		else {			
			lunabotics::RobotGeometry msg;
			msg.left_front_joint = lunabotics::geometry_msgs_Point_from_Point(allWheelGeometry->left_front());
			msg.left_rear_joint = lunabotics::geometry_msgs_Point_from_Point(allWheelGeometry->left_rear());
			msg.right_front_joint = lunabotics::geometry_msgs_Point_from_Point(allWheelGeometry->right_front());
			msg.right_rear_joint = lunabotics::geometry_msgs_Point_from_Point(allWheelGeometry->right_rear());
			msg.wheel_radius = allWheelGeometry->wheel_radius();
			msg.wheel_offset = allWheelGeometry->wheel_offset();
			msg.wheel_width = allWheelGeometry->wheel_width();
			geometryPublisher.publish(msg);
		}
#endif
		
		//Whenever needed send control message
		if (autonomyEnabled) {
			//ROS_INFO("autonomous");
			if (wayIterator < waypoints.end()) {
			
				if (isnan((*wayIterator).x) || isnan((*wayIterator).y)) {
					ROS_WARN("Waypoint undetermined");
				}
				else if (isnan(currentPose.position.x) || isnan(currentPose.position.y)) {
					ROS_WARN("Current position undetermined");
				}
				else {
					switch (controlMode) {
						case lunabotics::proto::ACKERMANN: controlAckermann(); break;
						case lunabotics::proto::TURN_IN_SPOT: controlSkid(); break;
						case lunabotics::proto::CRAB: 
						//	ROS_INFO("Control mode crab");
								break;
						default:
						ROS_INFO("Control mode UNKNOWN");
						 break;
					}
				}
			}
			else {
				ROS_ERROR("Way iterator out of bounds");
				stop();
			}
		}
		
		if (predCtrl->needsMoreControl()) {
			lunabotics::AllWheelState msg;
			predCtrl->control(msg);
			allWheelStateROSPublisher.publish(msg);
		}
		//ROS_INFO("beat");
		
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	delete pidController;
	delete allWheelGeometry;
	delete predCtrl;
	
	return 0;
}
