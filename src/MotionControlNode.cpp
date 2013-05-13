#include "MotionControlNode.h"
#include "lunabotics/PathTopic.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "lunabotics/ControlParams.h"
#include "lunabotics/RobotGeometry.h"

using namespace lunabotics;

#define PID_P_DIFF	0.05
#define PID_I_DIFF	0.1
#define PID_D_DIFF	0.18
#define VEC_OFFSET_DIFF	0.5
#define VEC_MULTI_DIFF	1

#define PID_P_ALL	2
#define PID_I_ALL	0.1
#define PID_D_ALL	1.5
#define VEC_OFFSET_ALL	0.5
#define VEC_MULTI_ALL 0.2



MotionControlNode::MotionControlNode(int argc, char **argv, std::string name, int frequency):
 ROSNode(argc, argv, name, frequency),
_sequence(0), _autonomyEnabled(false), _steeringMode(lunabotics::proto::ACKERMANN), 
_pointTurnMotionState( lunabotics::proto::Telemetry::STOPPED), _currentPose(), _waypointsIt(),
_waypoints(), _motionConstraints(), _minICRRadius(0.0f)
{
	
	Point zeroPoint = CreatePoint(0, 0);
	this->_currentPose.position = zeroPoint;
	this->_currentPose.orientation = 0.0f;
	
	this->_motionConstraints.point_turn_angle_accuracy = 0.1;
	this->_motionConstraints.point_turn_distance_accuracy = 0.1;
	this->_motionConstraints.bezier_segments_num = 20;
	this->_motionConstraints.lin_velocity_limit = this->_isDiffDriveRobot ? 0.33 : 0.2;
	
	this->_predefinedControl = new PredefinedCmdController();
	
	if (this->_isDiffDriveRobot) {
		this->_PID = new PIDController(PID_P_DIFF, PID_I_DIFF, PID_D_DIFF);
		this->_PIDHelper = new PIDGeometry(VEC_OFFSET_DIFF, VEC_MULTI_DIFF);
	}
	else {
		this->_PID = new PIDController(PID_P_ALL, PID_I_ALL, PID_D_ALL);
		this->_PIDHelper = new PIDGeometry(VEC_OFFSET_ALL, VEC_MULTI_ALL);
	}
	
	this->_geometryHelper = new AllWheelGeometry(zeroPoint, zeroPoint, zeroPoint, zeroPoint);
	
	this->_trajectory = new Trajectory();	
	
	//Create subscribers
	this->_subscriberEmergency = this->_nodeHandle->subscribe("emergency", 256, &MotionControlNode::callbackEmergency, this);
	this->_subscriberAutonomy = this->_nodeHandle->subscribe("autonomy", 1, &MotionControlNode::callbackAutonomy, this);
	this->_subscriberState = this->_nodeHandle->subscribe("state", 1, &MotionControlNode::callbackState, this);
	this->_subscriberGoal = this->_nodeHandle->subscribe("goal", 256, &MotionControlNode::callbackGoal, this);
	this->_subscriberPID = this->_nodeHandle->subscribe("pid", sizeof(float)*3, &MotionControlNode::callbackPID, this);
	this->_subscriberSteeringMode = this->_nodeHandle->subscribe("control_mode", 1, &MotionControlNode::callbackSteeringMode, this);
	this->_subscriberICR = this->_nodeHandle->subscribe("icr", sizeof(float)*3, &MotionControlNode::callbackICR, this);
	this->_subscriberAllWheelCommon = this->_nodeHandle->subscribe("all_wheel_common", 256, &MotionControlNode::callbackAllWheelCommon, this);
	this->_subscriberAllWheelFeedback = this->_nodeHandle->subscribe("all_wheel_feeback", sizeof(float)*8, &MotionControlNode::callbackAllWheelFeedback, this);
	
	//Create publishers
	this->_publisherDiffDriveMotion = this->_nodeHandle->advertise<geometry_msgs::Twist>("/cmd_vel", 256);
	this->_publisherPath = this->_nodeHandle->advertise<lunabotics::PathTopic>("path", 256);
	this->_publisherICR = this->_nodeHandle->advertise<geometry_msgs::Point>("icr_state", sizeof(float)*2);
	this->_publisherAllWheelMotion = this->_nodeHandle->advertise<lunabotics::AllWheelState>("all_wheel", sizeof(float)*8);
	this->_publisherControlParams = this->_nodeHandle->advertise<lunabotics::ControlParams>("control_params", 256);
	this->_publisherGeometry = this->_nodeHandle->advertise<lunabotics::RobotGeometry>("geometry", sizeof(float)*2*4);
	this->_publisherAllWheelCommon = this->_nodeHandle->advertise<lunabotics::AllWheelCommon>("all_wheel_common", sizeof(uint32_t));
	
	
	//Create service clients
	this->_clientMap = this->_nodeHandle->serviceClient<nav_msgs::GetMap>("map");
	
	ROS_INFO("Motion Control Ready");
}

MotionControlNode::~MotionControlNode()
{
	delete this->_predefinedControl;
	delete this->_PID;
	delete this->_geometryHelper;
	delete this->_trajectory;
	delete this->_PIDHelper;
}

//---------------------- CALLBACK METHODS ------------------------------

void MotionControlNode::callbackEmergency(const lunabotics::Emergency::ConstPtr &msg)
{
	//Use msg to stop driving if applicable
}

void MotionControlNode::callbackState(const lunabotics::State::ConstPtr &msg)
{    
	//ROS_INFO("Pose updated");
	this->_currentPose = lunabotics::Pose_from_geometry_msgs_Pose(msg->odometry.pose.pose);
	this->_PIDHelper->setLinearVelocity(msg->odometry.twist.twist.linear.x);
	this->_PIDHelper->setCurrentPose(this->_currentPose);

}

void MotionControlNode::callbackPID(const lunabotics::PID::ConstPtr &msg)
{
	this->_PID->setP(msg->p);
	this->_PID->setI(msg->i);
	this->_PID->setD(msg->d);
	this->_PIDHelper->setVelocityMultiplier(msg->velocity_multiplier);
	this->_PIDHelper->setVelocityOffset(msg->velocity_offset);

}

void MotionControlNode::callbackSteeringMode(const lunabotics::ControlMode::ConstPtr &msg)
{
	
	this->_steeringMode = (lunabotics::proto::SteeringModeType)msg->mode;
	if (this->_steeringMode == lunabotics::proto::ACKERMANN) {
		this->_motionConstraints.lin_velocity_limit = msg->linear_speed_limit;
		this->_motionConstraints.bezier_segments_num = (int)msg->smth_else;
	}
	
	ROS_INFO("Switching control mode to %s", steeringModeToString(this->_steeringMode).c_str());
}


void MotionControlNode::callbackAllWheelFeedback(const lunabotics::AllWheelState::ConstPtr &msg)
{
	this->_predefinedControl->giveFeedback(*msg);
}

void MotionControlNode::callbackAllWheelCommon(const lunabotics::AllWheelCommon::ConstPtr &msg)
{
	this->_predefinedControl->setNewCommand((lunabotics::proto::AllWheelControl::PredefinedControlType)msg->predefined_cmd);
}

void MotionControlNode::callbackAutonomy(const std_msgs::Bool::ConstPtr &msg)
{
	
	//Use msg to toggle autonomy
	if (msg->data) {
			
	}
	else {
		this->controlStop();
	}
	
}

void MotionControlNode::callbackICR(const lunabotics::ICRControl::ConstPtr &msg)
{		
	
	geometry_msgs::Point ICRMsg = msg->ICR;
	this->_publisherICR.publish(ICRMsg);
	
	Point ICR = Point_from_geometry_msgs_Point(msg->ICR);
	
	float angle_front_left;
	float angle_front_right;
	float angle_rear_left;
	float angle_rear_right;
	if (this->_geometryHelper->calculateAngles(ICR, angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
		if (validateAngles(angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
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
				vel_front_left = vel_rear_right = msg->velocity;
				vel_front_right = vel_rear_left = -msg->velocity;
			}
			else if (!this->_geometryHelper->calculateVelocities(ICR, msg->velocity, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)) {
				vel_front_left = vel_front_right = vel_rear_left = vel_rear_right = 0;			
			}
			controlMsg.driving.left_front = vel_front_left;
			controlMsg.driving.right_front = vel_front_right;
			controlMsg.driving.left_rear = vel_rear_left;
			controlMsg.driving.right_rear = vel_rear_right;
			
			
			ROS_INFO("VELOCITY %.2f | %.2f | %.2f | %.2f", vel_front_left, vel_front_right, vel_rear_left, vel_rear_right);
			
			this->_publisherAllWheelMotion.publish(controlMsg);
		}
		else {
			ROS_WARN("ICR position is in the dead zone, this control is not possible!");
		}
	}
	
}

void MotionControlNode::callbackGoal(const lunabotics::Goal::ConstPtr &msg)
{
	this->_waypoints.clear();
	this->_minICRRadius = 0;
	this->_motionConstraints.point_turn_angle_accuracy = msg->angleAccuracy;
	this->_motionConstraints.point_turn_distance_accuracy = msg->distanceAccuracy;
	
	//Specify params
	Point goal = Point_from_geometry_msgs_Point(msg->point);
		//ROS_INFO("Requesting path between (%.1f,%.1f) and (%.1f,%.1f)",
		//	  this->_currentPose.position.x, this->_currentPose.position.y,
		//	  goal.x, goal.y);
	float resolution;
	PathPtr path = this->findPath(this->_currentPose, goal, resolution);
	
	if (path->is_initialized()) {
		std::stringstream sstr;
		
		lunabotics::PathTopic pathMsg;
		pathMsg.path.header.stamp = ros::Time::now();
		pathMsg.path.header.seq = this->_sequence++;
		pathMsg.path.header.frame_id = "map";
		
		
		PointArr corner_points = path->cornerPoints(resolution);
		PointArr pts;
		
		
		if (this->_steeringMode == lunabotics::proto::ACKERMANN) {
			unsigned int size = corner_points.size();
			if (size > 2) {
				delete this->_trajectory;
				this->_trajectory = new Trajectory();
				Point startPoint = corner_points.at(0);
				Point endPoint = corner_points.at(size-1);
				IndexedPointArr closest_obstacles = path->closestObstaclePoints(resolution);
				unsigned int obst_size = closest_obstacles.size();
				
				pts.push_back(startPoint);
			
				//Get bezier quadratic curves for each point-turn
				for (unsigned int i = 1; i < size-1; i++) {
					Point q0, q2;
					Point prev = corner_points.at(i-1);
					Point curr = corner_points.at(i);
					Point next = corner_points.at(i+1);
					
					bool hasObstacle = false;
					Point obstaclePoint;
					
					//Since obstacle is the center of occupied cell, we want p to be at its edge
					if (obst_size > 0) {
						int start = std::min(obst_size-1, i-1);
						for (int j = start; j >= 0; j--) {
							IndexedPoint indexedObstacle = closest_obstacles.at(j);
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
						q0 = midPoint(prev, curr);
					}
					if (i == size-2) {
						q2 = next;
					}
					else {
						q2 = midPoint(next, curr);
					}
					
					TrajectorySegment s;
					if (hasObstacle) {
						Point p = midPoint(obstaclePoint, curr);
						s.curve = CreateConstrainedBezierCurve(q0, curr, q2, p, this->_motionConstraints.bezier_segments_num);
						//ROS_INFO("Curve from tetragonal q0=(%f,%f) q1=(%f,%f), q2=(%f,%f), p=(%f,%f)", q0.x, q0.y, curr.x, curr.y, q2.x, q2.y, p.x, p.y);
					}
					else {
						s.curve = new BezierCurve(q0, curr, q2, this->_motionConstraints.bezier_segments_num);
						//ROS_INFO("Curve without tetragonal q0=(%f,%f) q1=(%f,%f), q2=(%f,%f)", q0.x, q0.y, curr.x, curr.y, q2.x, q2.y);
					}
					this->_trajectory->appendSegment(s);
					lunabotics::CurveTopic curveMsg;
					curveMsg.start_idx = s.start_idx;
					curveMsg.end_idx = s.finish_idx;
					curveMsg.curvature = s.curve->maxCurvature();
					pathMsg.curves.push_back(curveMsg);
				}	
				pts = this->_trajectory->getPoints();
				pts.push_back(endPoint);
				
				ROS_WARN("Trajectory max curvature %f (Min ICR radius %f m)", this->_trajectory->maxCurvature(), 1/this->_trajectory->maxCurvature());
					
			}
			else {
				pts = corner_points;
			}
		}
		else {
			pts = corner_points;
		}
		
		int poseSeq = 0;
		for (PointArr::iterator it = pts.begin(); it != pts.end(); it++) {
			Point pt = *it;
			
			//float x_m = node.x*resolution;
			//float y_m = node.y*resolution;
			//float x_m = pt.x;
			//float y_m = pt.y;
			
			this->_waypoints.push_back(pt);
	//		sstr << "->(" << x_m << "," << y_m << ")";
			
			
			geometry_msgs::PoseStamped pose;
			pose.header.seq = poseSeq++;
			pose.header.stamp = ros::Time::now();
			pose.header.frame_id = "map";
			pose.pose.position = geometry_msgs_Point_from_Point(pt);
			pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
			pathMsg.path.poses.push_back(pose);
		}
		this->_PIDHelper->setTrajectory(pts);
		
		 this->_waypointsIt = this->_steeringMode == lunabotics::proto::ACKERMANN ? this->_waypoints.begin() : this->_waypoints.begin()+1;		
	//	ROS_INFO("Returned path: %s", sstr.str().c_str());
		//Point waypoint = this->_waypoints.at(0);
		//ROS_INFO("Heading towards (%.1f,%.1f)", (*this->_waypointsIt).x, (*this->_waypointsIt).y);
		
		this->_publisherPath.publish(pathMsg);
		
		this->_autonomyEnabled = true;
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.driving = this->_autonomyEnabled;
		controlParamsMsg.has_min_icr_radius = false;
		controlParamsMsg.next_waypoint_idx =  this->_waypointsIt < this->_waypoints.end() ?  this->_waypointsIt-this->_waypoints.begin()+1 : 0;
		this->_publisherControlParams.publish(controlParamsMsg);
	}
	else {
		ROS_INFO("Path is empty");
	}
	
	delete path;


}

//------------------- INHERITED METHODS --------------------------------

void MotionControlNode::runOnce()
{
	if (!this->_isDiffDriveRobot) {
		tf::StampedTransform transform;
		Point point;
		if (!this->_geometryHelper->geometryAcquired) {
			try {
				this->_tfListener.lookupTransform("left_front_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->_geometryHelper->set_left_front(point);
				this->_tfListener.lookupTransform("right_front_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->_geometryHelper->set_right_front(point);
				this->_tfListener.lookupTransform("left_rear_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->_geometryHelper->set_left_rear(point);
				this->_tfListener.lookupTransform("right_rear_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->_geometryHelper->set_right_rear(point);
				this->_tfListener.lookupTransform("left_front_wheel", "left_front_joint", ros::Time(0), transform);
				this->_geometryHelper->set_wheel_offset(fabs(transform.getOrigin().y()));
				this->_tfListener.lookupTransform("left_front_wheel_radius", "left_front_wheel", ros::Time(0), transform);
				this->_geometryHelper->set_wheel_radius(fabs(transform.getOrigin().x()));
				this->_geometryHelper->set_wheel_width(fabs(transform.getOrigin().y()));
				this->_geometryHelper->geometryAcquired = true;
				
				this->_predefinedControl->setGeometry(this->_geometryHelper);
			}
			catch (tf::TransformException e) {
				ROS_WARN("%s", e.what());
			}
		}
		else {			
			lunabotics::RobotGeometry msg;
			msg.left_front_joint = geometry_msgs_Point_from_Point(this->_geometryHelper->left_front());
			msg.left_rear_joint = geometry_msgs_Point_from_Point(this->_geometryHelper->left_rear());
			msg.right_front_joint = geometry_msgs_Point_from_Point(this->_geometryHelper->right_front());
			msg.right_rear_joint = geometry_msgs_Point_from_Point(this->_geometryHelper->right_rear());
			msg.wheel_radius = this->_geometryHelper->wheel_radius();
			msg.wheel_offset = this->_geometryHelper->wheel_offset();
			msg.wheel_width = this->_geometryHelper->wheel_width();
			this->_publisherGeometry.publish(msg);
		}
	}
		
	//Whenever needed send control message
	if (this->_autonomyEnabled) {
		if ( this->_waypointsIt <  this->_waypoints.end()) {
		
			if (isnan((* this->_waypointsIt).x) || isnan((* this->_waypointsIt).y)) {
				ROS_WARN("Waypoint undetermined");
			}
			else if (isnan(this->_currentPose.position.x) || isnan(this->_currentPose.position.y)) {
				ROS_WARN("Current position undetermined");
			}
			else {
				switch (this->_steeringMode) {
					case lunabotics::proto::ACKERMANN: this->controlAckermann(); break;
					case lunabotics::proto::TURN_IN_SPOT: this->controlPointTurn(); break;
					case lunabotics::proto::CRAB: this->controlCrab(); break;
					default: ROS_WARN("Unrecognized steering mode"); break;
				}
			}
		}
		else {
			ROS_ERROR("Way iterator out of bounds");
			this->controlStop();
		}
	}
	
	if (this->_predefinedControl->needsMoreControl()) {
		lunabotics::AllWheelState msg;
		this->_predefinedControl->control(msg);
		this->_publisherAllWheelMotion.publish(msg);
	}
}

//-------------------- CONTROL METHODS ---------------------------------

void MotionControlNode::finalizeRoute() 
{
	//ROS_INFO("Route completed");
	this->controlStop();
	
	//Send empty path to clear map in GUI
	lunabotics::PathTopic pathMsg;
	pathMsg.path.header.stamp = ros::Time::now();
	pathMsg.path.header.seq = this->_sequence++;
	pathMsg.path.header.frame_id = "map";
	this->_publisherPath.publish(pathMsg);
}



void MotionControlNode::controlAckermann()
{
	if (this->_isDiffDriveRobot) {
		this->controlAckermannDiffDrive();
	}
	else {
		this->controlAckermannAllWheel();
	}
}

void MotionControlNode::controlPointTurn()
{
	if ( this->_waypointsIt >=  this->_waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	double dx = (*this->_waypointsIt).x-this->_currentPose.position.x;
	double dy = (*this->_waypointsIt).y-this->_currentPose.position.y;
	double angle = normalizedAngle(atan2(dy, dx)-this->_currentPose.orientation);
	
	
	if (this->_isDiffDriveRobot) {
		this->controlPointTurnDiffDrive(dx, dy, angle);
	}
	else {
		this->controlPointTurnAllWheel(dx, dy, angle);
	}
}

void MotionControlNode::controlCrab()
{
	//TBD
}

void MotionControlNode::controlStop()
{
	this->_autonomyEnabled = false;
	
	if (this->_isDiffDriveRobot) {
		geometry_msgs::Twist msg;
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = 0;
		this->_publisherDiffDriveMotion.publish(msg);
	}
	else {
		lunabotics::AllWheelCommon msg;
		msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
		this->_publisherAllWheelCommon.publish(msg);
	}
	lunabotics::ControlParams msg;
	msg.driving = this->_autonomyEnabled;
	msg.has_min_icr_radius = false;
	msg.next_waypoint_idx =  this->_waypointsIt <  this->_waypoints.end() ?  this->_waypointsIt- this->_waypoints.begin()+1 : 0;
	this->_publisherControlParams.publish(msg);
	this->_waypoints.clear();
}


void MotionControlNode::controlAckermannAllWheel()
{
	if (this->_waypointsIt >= this->_waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	//If suddenly skipped a waypoint, proceed with the next ones, don't get stuck with current
	double dist = distance(*this->_waypointsIt, this->_currentPose.position);
	for (PointArr::iterator it = this->_waypointsIt+1; it < this->_waypoints.end(); it++) {
		double newDist = distance(*it, this->_currentPose.position);
		if (newDist < dist) {
			this->_waypointsIt = it;
			dist = newDist;
		}
	}
	
	if (dist < this->_motionConstraints.point_turn_distance_accuracy) {
		this->_waypointsIt++;
	}
	if (this->_waypointsIt >= this->_waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	
	double dx = (*this->_waypointsIt).x-this->_currentPose.position.x;
	double dy = (*this->_waypointsIt).y-this->_currentPose.position.y;
	
	if (this->_waypoints.size() >= 2) {
		
		//In the beginning turn in place towards the second waypoint (first waypoint is at the robot's position). It helps to solve problems with pid
		if (this->_waypointsIt < this->_waypoints.begin()+2) {
			this->_waypointsIt = this->_waypoints.begin()+1;
			double angle = normalizedAngle(atan2(dy, dx)-this->_currentPose.orientation);
			if (fabs(angle) > this->_motionConstraints.point_turn_angle_accuracy) {
				//ROS_WARN("Facing away from the trajectory. Turning in place");
			//	controlSkid();
			//	return;
			}
		}
		
		
		double y_err = this->_PIDHelper->getReferenceDistance();
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.trajectory_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getClosestTrajectoryPoint());
		controlParamsMsg.velocity_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getReferencePoint());
		controlParamsMsg.y_err = y_err;
		controlParamsMsg.driving = this->_autonomyEnabled;
		controlParamsMsg.t_trajectory_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getClosestTrajectoryPointInLocalFrame());
		controlParamsMsg.t_velocity_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getReferencePointInLocalFrame());
		controlParamsMsg.next_waypoint_idx = this->_waypointsIt < this->_waypoints.end() ? this->_waypointsIt-this->_waypoints.begin()+1 : 0;
		controlParamsMsg.has_trajectory_data = true;
		controlParamsMsg.has_min_icr_radius = true;
		controlParamsMsg.min_icr_radius = this->_minICRRadius;
		
		//Control law
		
		double signal;
		if (this->_PID->control(y_err, signal)) {
			signal *= 10.0;
			//ROS_WARN("DW %.2f", signal);
			
			double gamma = -signal/2;
			float gamma_abs = fabs(gamma);
			
			if (gamma_abs < 0.00001) {
				gamma = 0.01;
			}
			else if (gamma_abs > M_PI_2) {
				gamma = M_PI_2 * sign(gamma, 0.00001);
			}
			
			double alpha = M_PI_2-gamma;
			double offset_x = this->_geometryHelper->left_front().x;
			double offset_y = tan(alpha)*offset_x;
			
			Point ICR = CreatePoint(0, offset_y);
			
			float abs_offset_y = fabs(offset_y);
			
			
			ICR = this->_geometryHelper->point_outside_base_link(ICR);
			
			
			if (abs_offset_y > 0.0001 && (abs_offset_y < this->_minICRRadius || this->_minICRRadius < 0.0001)) {
				this->_minICRRadius = abs_offset_y;
				ROS_INFO("Radius %f (corrected radius %f)", abs_offset_y, ICR.y);
				controlParamsMsg.min_icr_radius = abs_offset_y;
				controlParamsMsg.has_min_icr_radius = true;
			}
			
			
			this->_publisherICR.publish(geometry_msgs_Point_from_Point(ICR));
			
			//ROS_INFO("Alpha %.2f offset %.2f ICR %.2f", alpha, offset_y, ICR.y);
			
			float velocity = this->_motionConstraints.lin_velocity_limit;
			
			float angle_front_left;
			float angle_front_right;
			float angle_rear_left;
			float angle_rear_right;
			if (this->_geometryHelper->calculateAngles(ICR, angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
				
				//Set angles to the ones outside dead zone
				validateAngles(angle_front_left, angle_front_right, angle_rear_left, angle_rear_right);
				
				lunabotics::AllWheelState controlMsg;
				controlMsg.steering.left_front = angle_front_left;
				controlMsg.steering.right_front = angle_front_right;
				controlMsg.steering.left_rear = angle_rear_left;
				controlMsg.steering.right_rear = angle_rear_right;
				float vel_front_left;
				float vel_front_right;
				float vel_rear_left;
				float vel_rear_right;
				if (!this->_geometryHelper->calculateVelocities(ICR, velocity, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)) {
					vel_front_left = vel_front_right = vel_rear_left = vel_rear_right = 0;			
				}
				controlMsg.driving.left_front = vel_front_left;
				controlMsg.driving.right_front = vel_front_right;
				controlMsg.driving.left_rear = vel_rear_left;
				controlMsg.driving.right_rear = vel_rear_right;
				
				this->_publisherAllWheelMotion.publish(controlMsg);
			}
		}
		this->_publisherControlParams.publish(controlParamsMsg);
	}
	else {
		//No need for curvature, just straight driving
		this->controlPointTurn();
	}
}

void MotionControlNode::controlAckermannDiffDrive()
{
	if (this->_waypointsIt >= this->_waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	//If suddenly skipped a waypoint, proceed with the next ones, don't get stuck with current
	double dist = distance(*this->_waypointsIt, this->_currentPose.position);
	for (PointArr::iterator it = this->_waypointsIt+1; it < this->_waypoints.end(); it++) {
		double newDist = distance(*it, this->_currentPose.position);
		if (newDist < dist) {
			this->_waypointsIt = it;
			dist = newDist;
		}
	}
	
	if (dist < this->_motionConstraints.point_turn_distance_accuracy) {
		this->_waypointsIt++;
	}
	if (this->_waypointsIt >= this->_waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	
	double dx = (*this->_waypointsIt).x-this->_currentPose.position.x;
	double dy = (*this->_waypointsIt).y-this->_currentPose.position.y;
	
	if (this->_waypoints.size() >= 2) {
		
		//In the beginning turn in place towards the second waypoint (first waypoint is at the robot's position). It helps to solve problems with pid
		if (this->_waypointsIt < this->_waypoints.begin()+2) {
			this->_waypointsIt = this->_waypoints.begin()+1;
			double angle = normalizedAngle(atan2(dy, dx)-this->_currentPose.orientation);
			if (fabs(angle) > this->_motionConstraints.point_turn_angle_accuracy) {
				//ROS_WARN("Facing away from the trajectory. Turning in place");
				this->controlPointTurn();
				return;
			}
		}
		
		
		
		double y_err = this->_PIDHelper->getReferenceDistance();
		lunabotics::ControlParams controlParamsMsg;
		controlParamsMsg.trajectory_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getClosestTrajectoryPoint());
		controlParamsMsg.velocity_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getReferencePoint());
		controlParamsMsg.y_err = y_err;
		controlParamsMsg.driving = this->_autonomyEnabled;
		controlParamsMsg.t_trajectory_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getClosestTrajectoryPointInLocalFrame());
		controlParamsMsg.t_velocity_point = geometry_msgs_Point_from_Point(this->_PIDHelper->getReferencePointInLocalFrame());
		controlParamsMsg.next_waypoint_idx = this->_waypointsIt < this->_waypoints.end() ? this->_waypointsIt-this->_waypoints.begin()+1 : 0;
		controlParamsMsg.has_trajectory_data = true;
		controlParamsMsg.has_min_icr_radius = false;
		this->_publisherControlParams.publish(controlParamsMsg);
		
		//Control law
		
		double dw;
		if (this->_PID->control(y_err, dw)) {
			dw *= -3;
			//ROS_WARN("DW %.2f", dw);
			
			//The higher angular speed, the lower linear speed is
			double top_w = 1.57;
			double v = this->_motionConstraints.lin_velocity_limit * std::max(0.0, (top_w-fabs(dw)))/top_w;
			v = std::max(0.01, v);
			v = std::min(this->_motionConstraints.lin_velocity_limit, v);
				
			geometry_msgs::Twist twistMsg;
			twistMsg.linear.x = v;
			twistMsg.linear.y = 0;
			twistMsg.linear.z = 0;
			twistMsg.angular.x = 0;
			twistMsg.angular.y = 0;
			twistMsg.angular.z = dw;
			this->_publisherDiffDriveMotion.publish(twistMsg);
		}
	}
	else {
		//No need for curvature, just straight driving
		this->controlPointTurn();
	}
}
void MotionControlNode::controlPointTurnAllWheel(double dx, double dy, double theta)
{
	lunabotics::AllWheelCommon msg;

	lunabotics::ControlParams controlParamsMsg;
	controlParamsMsg.driving = this->_autonomyEnabled;
	
	switch (this->_pointTurnMotionState) {
		case lunabotics::proto::Telemetry::STOPPED: {
			//ROS_INFO("SKID: stopped dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			
			if (fabs(dx) < this->_motionConstraints.point_turn_distance_accuracy &&
				fabs(dy) < this->_motionConstraints.point_turn_distance_accuracy) {
				this->_waypointsIt++;
				if (this->_waypointsIt >= this->_waypoints.end()) {
					this->finalizeRoute();
				}
				else {
					controlParamsMsg.has_trajectory_data = true;
					controlParamsMsg.next_waypoint_idx = this->_waypointsIt < this->_waypoints.end() ? this->_waypointsIt-this->_waypoints.begin()+1 : 0;	
					//lunabotics::Point nextWaypoint = *this->_waypointsIt;
					//ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypoint.position.x, nextWaypoint.position.y);
				}
			}
			else if (fabs(theta) > this->_motionConstraints.point_turn_angle_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
			}
			else if (fabs(dx) > this->_motionConstraints.point_turn_distance_accuracy || fabs(dy) > this->_motionConstraints.point_turn_distance_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::DRIVING;
			}				
		}	
		break;
		case lunabotics::proto::Telemetry::DRIVING: {	
			//ROS_INFO("SKID: driving        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			if (fabs(dx) < this->_motionConstraints.point_turn_distance_accuracy && fabs(dy) < this->_motionConstraints.point_turn_distance_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
			}
			else if (fabs(theta) > this->_motionConstraints.point_turn_angle_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
			}	
			else {
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::DRIVE_FORWARD;
			}
		}
		break;
		case lunabotics::proto::Telemetry::TURNING: {
			int direction = sign(theta, this->_motionConstraints.point_turn_angle_accuracy);
			//ROS_INFO("SKID: turning  %d (%.2f-%.2f)      dx: %.5f dy: %.5f angle: %.5f", direction, angle, this->_motionConstraints.point_turn_angle_accuracy, dx, dy, angle);
		
			if (direction == 0) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
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
	
	controlParamsMsg.point_turn_state = this->_pointTurnMotionState;
	controlParamsMsg.has_point_turn_state = true;
	controlParamsMsg.has_min_icr_radius = false;
	this->_publisherControlParams.publish(controlParamsMsg);
	this->_publisherAllWheelCommon.publish(msg);
}

void MotionControlNode::controlPointTurnDiffDrive(double dx, double dy, double theta)
{
	lunabotics::ControlParams controlParamsMsg;
	controlParamsMsg.driving = this->_autonomyEnabled;
	controlParamsMsg.has_point_turn_state = true;
	geometry_msgs::Twist twistMsg;
	twistMsg.linear.x = 0;
	twistMsg.linear.y = 0;
	twistMsg.linear.z = 0;
	twistMsg.angular.x = 0;
	twistMsg.angular.y = 0;
	twistMsg.angular.z = 0;
					
	switch (this->_pointTurnMotionState) {
		case lunabotics::proto::Telemetry::STOPPED: {
			//ROS_INFO("SKID: stopped        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			
			if (fabs(dx) < this->_motionConstraints.point_turn_distance_accuracy && fabs(dy) < this->_motionConstraints.point_turn_distance_accuracy) {
				 this->_waypointsIt++;
				if ( this->_waypointsIt >= this->_waypoints.end()) {
					this->finalizeRoute();
				}
				else {
					controlParamsMsg.has_trajectory_data = true;
					controlParamsMsg.next_waypoint_idx =  this->_waypointsIt < this->_waypoints.end() ?  this->_waypointsIt-this->_waypoints.begin()+1 : 0;
					//lunabotics::Point nextWaypoint = * this->_waypointsIt;
					//ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypoint.position.x, nextWaypoint.position.y);
				}
			}
			else if (fabs(theta) > this->_motionConstraints.point_turn_angle_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
			}
			else if (fabs(dx) > this->_motionConstraints.point_turn_distance_accuracy ||
					 fabs(dy) > this->_motionConstraints.point_turn_distance_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::DRIVING;
			}				
		}	
		break;
		case lunabotics::proto::Telemetry::DRIVING: {	
		//	ROS_INFO("SKID: driving        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);	
			if (fabs(dx) < this->_motionConstraints.point_turn_distance_accuracy &&
				fabs(dy) < this->_motionConstraints.point_turn_distance_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
				twistMsg.linear.x = 0;
				twistMsg.angular.z = 0;
			}
			else if (fabs(theta) > this->_motionConstraints.point_turn_angle_accuracy) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
			}	
			else {
				twistMsg.linear.x = 1;
				twistMsg.angular.z = 0;
			}
		}
		break;
		case lunabotics::proto::Telemetry::TURNING: {
			//ROS_INFO("SKID: turning        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			int direction = sign(theta, this->_motionConstraints.point_turn_angle_accuracy);
		
			if (direction == 0) {
				this->_pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
				twistMsg.linear.x = 0;
				twistMsg.angular.z = 0;
			}
			else {
				//ROS_INFO("SKID: %s", direction == -1 ? "Right" : "Left");
				twistMsg.linear.x = 0;
				twistMsg.angular.z = direction;
			}	
		}
		break;
		default: break;
	}
	
	controlParamsMsg.point_turn_state = this->_pointTurnMotionState;
	controlParamsMsg.has_min_icr_radius = false;
	this->_publisherControlParams.publish(controlParamsMsg);
	this->_publisherDiffDriveMotion.publish(twistMsg);
}


//------------------ PLANNING METHODS ----------------------------------

PathPtr MotionControlNode::findPath(Pose startPose, Point goalPoint, float &res)
{
	if (this->_clientMap.call(this->_serviceMap)) {
		this->controlStop();
		
		unsigned int map_size = this->_serviceMap.response.map.data.size();
		if (map_size > 0) {
			ROS_INFO("Got map from service (%d cells)", (int)map_size);
			ROS_INFO("Looking for a path...");
			
			//Generate a path
			float resolution = this->_serviceMap.response.map.info.resolution;
			res = resolution;
			
			int start_x = round(startPose.position.x/resolution);
			int start_y = round(startPose.position.y/resolution);
			int goal_x = round(goalPoint.x/resolution);
			int goal_y = round(goalPoint.y/resolution);
			
			PathPtr graph = new Path(this->_serviceMap.response.map.data, 
									this->_serviceMap.response.map.info.width, 
									this->_serviceMap.response.map.info.height,
									start_x, start_y, goal_x, goal_y);
									
			
			if (graph->allNodes().size() == 0) {
				ROS_INFO("Path is not found");
				return NULL;
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
	return NULL;
}

void MotionControlNode::run()
{
	//Let the parent to the trick
	ROSNode::run();
}

//----------------------- MAIN METHOD ----------------------------------

int main(int argc, char **argv)
{
	lunabotics::MotionControlNode *node = new lunabotics::MotionControlNode(argc, argv, "luna_driver", 200);
	node->run();
	delete node;
	return 0;
}
