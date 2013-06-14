#include "MotionControlNode.h"
#include "lunabotics/PathTopic.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "lunabotics/PathFollowingTelemetry.h"
#include "lunabotics/RobotGeometry.h"

using namespace lunabotics;

#define PID_P_DIFF	0.05
#define PID_I_DIFF	0.1
#define PID_D_DIFF	0.18
#define FB_OFFSET_DIFF	0.5
#define FB_MX_DIFF		1
#define FF_OFFSET_DIFF	0
#define FF_FRAC_DIFF	0

#define PID_P_ALL	2
#define PID_I_ALL	0.1
#define PID_D_ALL	1.5
#define FB_OFFSET_ALL	0.05
#define FB_MX_ALL		0.25
#define FF_OFFSET_ALL	0.05
#define FF_FRAC_ALL		0.5

#define LIN_VEL_ACCURACY	0.02
#define YAW_RATE_ACCURACY	0.01

MotionControlNode::MotionControlNode(int argc, char **argv, std::string name, int frequency):
ROSNode(argc, argv, name, frequency), cached_map(),
cached_map_up_to_date(false), 
sequence(0), autonomyEnabled(false), steeringMode(lunabotics::proto::ACKERMANN), 
pointTurnMotionState(lunabotics::proto::Telemetry::STOPPED), currentPose(), segmentsIt(), segments(),
waypointsIt(), waypoints(), desiredWaypoints(), motionConstraints(), minICRRadius(0.0f),
ackermannJustStarted(false), previousYaw(0), previousYawTime(), linearVelocity(0)
{
	
	Point zeroPoint = CreatePoint(0, 0);
	this->currentPose.position = zeroPoint;
	this->currentPose.orientation = 0.0f;
	
	this->motionConstraints.point_turn_angle_accuracy = 0.1;
	this->motionConstraints.point_turn_distance_accuracy = 0.1;
	this->motionConstraints.bezier_segments_num = 20;
	this->motionConstraints.lin_velocity_limit = this->isDiffDriveRobot ? 0.33 : 0.2;
	
	this->predefinedControl = new PredefinedCmdController();
	
	this->robotGeometry = new AllWheelGeometry(zeroPoint, zeroPoint, zeroPoint, zeroPoint);
	
	if (this->isDiffDriveRobot) {
		this->ackermannPID = new PIDController(PID_P_DIFF, PID_I_DIFF, PID_D_DIFF);
		this->pathFollowingGeometry = new PathFollowingGeometry(this->robotGeometry, FB_OFFSET_DIFF, FB_MX_DIFF, FF_OFFSET_DIFF, FF_FRAC_DIFF);
	}
	else {
		this->ackermannPID = new PIDController(PID_P_ALL, PID_I_ALL, PID_D_ALL);
		this->pathFollowingGeometry = new PathFollowingGeometry(this->robotGeometry, FB_OFFSET_ALL, FB_MX_ALL, FF_OFFSET_ALL, FF_FRAC_ALL);
	}
	
	this->pointTurnPID = new PIDController(1.5, 0.1, 0.2);
	
	
	this->trajectory = new Trajectory();	
	
	//Create subscribers
	this->subscriberEmergency = this->nodeHandle->subscribe(TOPIC_EMERGENCY, 256, &MotionControlNode::callbackEmergency, this);
	this->subscriberAutonomy = this->nodeHandle->subscribe(TOPIC_CMD_AUTONOMY, 1, &MotionControlNode::callbackAutonomy, this);
	this->subscriberState = this->nodeHandle->subscribe(TOPIC_TM_ROBOT_STATE, 1, &MotionControlNode::callbackState, this);
	this->subscriberGoal = this->nodeHandle->subscribe(TOPIC_CMD_GOAL, 256, &MotionControlNode::callbackGoal, this);
	this->subscriberPID = this->nodeHandle->subscribe(TOPIC_CMD_PATH_FOLLOWING, sizeof(float)*3, &MotionControlNode::callbackPID, this);
	this->subscriberSteeringMode = this->nodeHandle->subscribe(TOPIC_STEERING_MODE, 1, &MotionControlNode::callbackSteeringMode, this);
	this->subscriberICR = this->nodeHandle->subscribe(TOPIC_CMD_ICR, sizeof(float)*3, &MotionControlNode::callbackICR, this);
	this->subscriberCrab = this->nodeHandle->subscribe(TOPIC_CMD_CRAB, sizeof(float)*2, &MotionControlNode::callbackCrab, this);
	this->subscriberAllWheelCommon = this->nodeHandle->subscribe(TOPIC_CMD_ALL_WHEEL, 256, &MotionControlNode::callbackAllWheelCommon, this);
	this->subscriberAllWheelFeedback = this->nodeHandle->subscribe(TOPIC_TM_ALL_WHEEL, sizeof(float)*8, &MotionControlNode::callbackAllWheelFeedback, this);
	
	//Create publishers
	this->publisherDiffDriveMotion = this->nodeHandle->advertise<geometry_msgs::Twist>(TOPIC_CMD_TWIST, 256);
	this->publisherPath = this->nodeHandle->advertise<lunabotics::PathTopic>(TOPIC_TM_PATH, 256);
	this->publisherICR = this->nodeHandle->advertise<geometry_msgs::Point>(TOPIC_TM_ICR, sizeof(float)*2);
	this->publisherAllWheelMotion = this->nodeHandle->advertise<lunabotics::AllWheelState>(TOPIC_CMD_EXPLICIT_ALL_WHEEL, sizeof(float)*8);
	this->publisherPathFollowingTelemetry = this->nodeHandle->advertise<lunabotics::PathFollowingTelemetry>(TOPIC_TM_PATH_FOLLOWING, 256);
	this->publisherGeometry = this->nodeHandle->advertise<lunabotics::RobotGeometry>(TOPIC_TM_ROBOT_GEOMETRY, sizeof(float)*2*4);
	this->publisherAllWheelCommon = this->nodeHandle->advertise<lunabotics::AllWheelCommon>(TOPIC_CMD_ALL_WHEEL, sizeof(uint32_t));
	this->publisherCrab = this->nodeHandle->advertise<lunabotics::CrabControl>(TOPIC_CMD_CRAB, sizeof(float)*2);
	
	
	//Create service clients
	this->clientMap = this->nodeHandle->serviceClient<nav_msgs::GetMap>(SERVICE_MAP);
	
	ROS_INFO("Motion Control Ready");
}

MotionControlNode::~MotionControlNode()
{
	delete this->predefinedControl;
	delete this->ackermannPID;
	delete this->pointTurnPID;
	delete this->robotGeometry;
	delete this->trajectory;
	delete this->pathFollowingGeometry;
}

//---------------------- CALLBACK METHODS ------------------------------

void MotionControlNode::callbackEmergency(const lunabotics::Emergency::ConstPtr &msg)
{
	//Use msg to stop driving if applicable
}

void MotionControlNode::callbackState(const lunabotics::State::ConstPtr &msg)
{    
	//ROS_INFO("Pose updated");
	this->currentPose = lunabotics::Pose_from_geometry_msgs_Pose(msg->odometry.pose.pose);
	this->pathFollowingGeometry->setVelocity(msg->odometry.twist.twist.linear.x);
	this->pathFollowingGeometry->setCurrentPose(this->currentPose);
	this->linearVelocity = msg->odometry.twist.twist.linear.x;
}

void MotionControlNode::callbackPID(const lunabotics::PID::ConstPtr &msg)
{
	this->ackermannPID->setP(msg->p);
	this->ackermannPID->setI(msg->i);
	this->ackermannPID->setD(msg->d);
	this->pathFollowingGeometry->setFeedbackPointOffsetMultiplier(msg->feedback_multiplier);
	this->pathFollowingGeometry->setFeedbackPointOffsetMin(msg->feedback_min_offset);
	this->pathFollowingGeometry->setFeedforwardCurvatureDetectionStart(msg->feedforward_min_offset);
	this->pathFollowingGeometry->setFeedforwardPointOffsetFraction(msg->feedforward_fraction);
}

void MotionControlNode::callbackSteeringMode(const lunabotics::SteeringMode::ConstPtr &msg)
{
	
	this->steeringMode = (lunabotics::proto::SteeringModeType)msg->mode;
	this->motionConstraints.point_turn_angle_accuracy = msg->angle_accuracy;
	this->motionConstraints.point_turn_distance_accuracy = msg->distance_accuracy;
	this->motionConstraints.lin_velocity_limit = msg->linear_speed_limit;
	this->motionConstraints.bezier_segments_num = msg->bezier_segments;
	
	ROS_INFO("Switching control mode to %s", steeringModeToString(this->steeringMode).c_str());
}


void MotionControlNode::callbackAllWheelFeedback(const lunabotics::AllWheelState::ConstPtr &msg)
{
	this->predefinedControl->giveFeedback(*msg);
}

void MotionControlNode::callbackAllWheelCommon(const lunabotics::AllWheelCommon::ConstPtr &msg)
{
	this->predefinedControl->setNewCommand((lunabotics::proto::AllWheelControl::PredefinedControlType)msg->predefined_cmd);
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

void MotionControlNode::callbackCrab(const lunabotics::CrabControl::ConstPtr &msg)
{
	float angle_front_left = msg->heading;
	float angle_front_right = msg->heading;
	float angle_rear_left = msg->heading;
	float angle_rear_right = msg->heading;
	if (validateAngles(angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
		double min_angle = std::min(fabs(angle_front_left), fabs(angle_front_right));
		min_angle = std::min(min_angle, fabs(angle_rear_left));
		min_angle = std::min(min_angle, fabs(angle_rear_right));
		min_angle *= sign(angle_front_right, 0.0001);
	
		lunabotics::AllWheelState controlMsg;
		controlMsg.steering.left_front = min_angle;
		controlMsg.steering.right_front = min_angle;
		controlMsg.steering.left_rear = min_angle;
		controlMsg.steering.right_rear = min_angle;
		controlMsg.driving.left_front = msg->velocity;
		controlMsg.driving.right_front = msg->velocity;
		controlMsg.driving.left_rear = msg->velocity;
		controlMsg.driving.right_rear = msg->velocity;
		this->publisherAllWheelMotion.publish(controlMsg);
	}
}

void MotionControlNode::callbackICR(const lunabotics::ICRControl::ConstPtr &msg)
{		
	
	geometry_msgs::Point ICRMsg = msg->ICR;
	this->publisherICR.publish(ICRMsg);
	
	Point ICR = Point_from_geometry_msgs_Point(msg->ICR);
	
	float angle_front_left;
	float angle_front_right;
	float angle_rear_left;
	float angle_rear_right;
	if (this->robotGeometry->calculateAngles(ICR, angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
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
			else if (!this->robotGeometry->calculateVelocities(ICR, msg->velocity, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)) {
				vel_front_left = vel_front_right = vel_rear_left = vel_rear_right = 0;			
			}
			controlMsg.driving.left_front = vel_front_left;
			controlMsg.driving.right_front = vel_front_right;
			controlMsg.driving.left_rear = vel_rear_left;
			controlMsg.driving.right_rear = vel_rear_right;
			
			
			ROS_INFO("VELOCITY %.2f | %.2f | %.2f | %.2f", vel_front_left, vel_front_right, vel_rear_left, vel_rear_right);
			
			this->publisherAllWheelMotion.publish(controlMsg);
		}
		else {
			ROS_WARN("ICR position is in the dead zone, this control is not possible!");
		}
	}
	
}

void MotionControlNode::callbackGoal(const lunabotics::Goal::ConstPtr &msg)
{
	this->controlStop();
	this->ackermannJustStarted = true;
	this->waypoints.clear();
	this->minICRRadius = 0;
	
	lunabotics::PathTopic pathMsg;
	pathMsg.path.header.stamp = ros::Time::now();
	pathMsg.path.header.seq = this->sequence++;
	pathMsg.path.header.frame_id = "map";
	
	if (this->steeringMode == lunabotics::proto::ACKERMANN || this->steeringMode == lunabotics::proto::AUTO) {
		delete this->trajectory;
		this->trajectory = new Trajectory();
	}
	
	ROS_INFO("Getting %d waypoints cmd", (int)msg->waypoints.size());
	this->getMapIfNeeded();
	float resolution = this->cached_map.info.resolution;
	int start_x = std::min(round(this->currentPose.position.x/resolution), (double)this->cached_map.info.width-1);
	int start_y = std::min(round(this->currentPose.position.y/resolution), (double)this->cached_map.info.height-1);
	start_x = std::max(0, start_x);
	start_y = std::max(0, start_y);
	Point start = CreatePoint(start_x, start_y);
	ROS_INFO("Starting point is %d,%d", start_x, start_y);
	
	PathPtr path = new Path(this->cached_map.data, this->cached_map.info.width,
							this->cached_map.info.height, start);
	
	for (unsigned int k = 0; k < msg->waypoints.size(); k++) {
		Point goal = Point_from_geometry_msgs_Point(msg->waypoints.at(k));
		//ROS_INFO("Requesting path between (%.1f,%.1f) and (%.1f,%.1f)",
		//	  this->currentPose.position.x, this->currentPose.position.y,
		//	  goal.x, goal.y);
		
		ROS_INFO("Getting path of %d segment between (%.1f,%.1f) and (%.1f,%.1f)", k, start.x, start.y, goal.x, goal.y);
		
		path->appendGoal(goal);
		
		start = goal;
	}
	if (path->allNodes().size() == 0) {
		ROS_INFO("Path is not found");
	}
	else {
		ROS_INFO("Path got");
		
		if (path->is_initialized()) {
			ROS_INFO("Path correct");
			std::stringstream sstr;
			
			PointArr pts;
			PointArr corner_points = path->cornerPoints(resolution);
			ROS_INFO("Called corner Pts");

			//Transform into bezier curves
			if (this->steeringMode == lunabotics::proto::ACKERMANN || this->steeringMode == lunabotics::proto::AUTO) {
				unsigned int size = corner_points.size();
				if (size > 0) {
					corner_points[0] = this->currentPose.position;
				}
				if (size > 2) {
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
							s.curve = CreateConstrainedBezierCurve(q0, curr, q2, p, this->motionConstraints.bezier_segments_num);
							//ROS_INFO("Curve from tetragonal q0=(%f,%f) q1=(%f,%f), q2=(%f,%f), p=(%f,%f)", q0.x, q0.y, curr.x, curr.y, q2.x, q2.y, p.x, p.y);
						}
						else {
							s.curve = new BezierCurve(q0, curr, q2, this->motionConstraints.bezier_segments_num);
							//ROS_INFO("Curve without tetragonal q0=(%f,%f) q1=(%f,%f), q2=(%f,%f)", q0.x, q0.y, curr.x, curr.y, q2.x, q2.y);
						}
						this->trajectory->appendSegment(s);
					}
					if (this->steeringMode == lunabotics::proto::AUTO) {
						this->trajectory->updateSegmentsMetaInfo(this->robotGeometry->maxAvailableCurvature());
					}
					
					TrajectorySegmentArr segments = this->trajectory->segments();
					for (unsigned int i = 0; i < segments.size(); i++) {
						TrajectorySegment s = segments.at(i);
						lunabotics::CurveTopic curveMsg;
						curveMsg.start_idx = s.start_idx;
						curveMsg.end_idx = s.finish_idx;
						curveMsg.curvature = s.curve->maxCurvature();
						pathMsg.curves.push_back(curveMsg);
					}
					pts = this->trajectory->getPoints();
					pts.push_back(endPoint);
					this->segments = this->trajectory->segments();
					this->segmentsIt = this->segments.begin();
				}
				else {
					pts = corner_points;
				}
			}
			else {
				pts = corner_points;
			}
				
			int poseSeq = 0;
			this->waypoints = pts;
			for (PointArr::iterator it = pts.begin(); it != pts.end(); it++) {
				Point pt = *it;
				geometry_msgs::PoseStamped pose;
				pose.header.seq = poseSeq++;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = "map";
				pose.pose.position = geometry_msgs_Point_from_Point(pt);
				pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
				pathMsg.path.poses.push_back(pose);
			}
			this->pathFollowingGeometry->setPath(this->waypoints);
			
			if (this->steeringMode == lunabotics::proto::ACKERMANN) {
				ROS_WARN("Trajectory max curvature %f (Min ICR radius %f m)", this->trajectory->maxCurvature(), 1/this->trajectory->maxCurvature());
			}
			
			 this->waypointsIt = this->waypoints.begin()+1;		
		//	ROS_INFO("Returned path: %s", sstr.str().c_str());
			//Point waypoint = this->waypoints.at(0);
			//ROS_INFO("Heading towards (%.1f,%.1f)", (*this->waypointsIt).x, (*this->waypointsIt).y);
			
			this->publisherPath.publish(pathMsg);
	
			this->autonomyEnabled = true;
			lunabotics::PathFollowingTelemetry telemetryMsg;
			telemetryMsg.is_driving = this->autonomyEnabled;
			telemetryMsg.has_min_icr_radius = false;
			telemetryMsg.has_path_parts_enumeration = true;
			telemetryMsg.next_waypoint_idx =  this->waypointsIt < this->waypoints.end() ?  this->waypointsIt-this->waypoints.begin()+1 : 0;
			telemetryMsg.segment_idx = this->segmentsIt < this->segments.end() ? this->segmentsIt-this->segments.begin()+1 : 0;
			this->publisherPathFollowingTelemetry.publish(telemetryMsg);
		}
		else {
			ROS_WARN("Couldn't find a path'");
		}
	}
	delete path;
}

//------------------- INHERITED METHODS --------------------------------

void MotionControlNode::runOnce()
{
	if (!this->isDiffDriveRobot) {
		tf::StampedTransform transform;
		Point point;
		if (!this->robotGeometry->geometryAcquired) {
			try {
				this->tfListener.lookupTransform("left_front_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->robotGeometry->set_left_front(point);
				this->tfListener.lookupTransform("right_front_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->robotGeometry->set_right_front(point);
				this->tfListener.lookupTransform("left_rear_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->robotGeometry->set_left_rear(point);
				this->tfListener.lookupTransform("right_rear_joint", "base_link", ros::Time(0), transform);
				point.x = transform.getOrigin().x();
				point.y = transform.getOrigin().y();
				this->robotGeometry->set_right_rear(point);
				this->tfListener.lookupTransform("left_front_wheel", "left_front_joint", ros::Time(0), transform);
				this->robotGeometry->set_wheel_offset(fabs(transform.getOrigin().y()));
				this->tfListener.lookupTransform("left_front_wheel_radius", "left_front_wheel", ros::Time(0), transform);
				this->robotGeometry->set_wheel_radius(fabs(transform.getOrigin().x()));
				this->robotGeometry->set_wheel_width(fabs(transform.getOrigin().y()));
				this->robotGeometry->geometryAcquired = true;
				
				this->predefinedControl->setGeometry(this->robotGeometry);
			}
			catch (tf::TransformException e) {
				ROS_WARN("%s", e.what());
			}
		}
		else {			
			lunabotics::RobotGeometry msg;
			msg.left_front_joint = geometry_msgs_Point_from_Point(this->robotGeometry->left_front());
			msg.left_rear_joint = geometry_msgs_Point_from_Point(this->robotGeometry->left_rear());
			msg.right_front_joint = geometry_msgs_Point_from_Point(this->robotGeometry->right_front());
			msg.right_rear_joint = geometry_msgs_Point_from_Point(this->robotGeometry->right_rear());
			msg.wheel_radius = this->robotGeometry->wheel_radius();
			msg.wheel_offset = this->robotGeometry->wheel_offset();
			msg.wheel_width = this->robotGeometry->wheel_width();
			this->publisherGeometry.publish(msg);
		}
	}
		
	//Whenever needed send control message
	if (this->autonomyEnabled) {		
		if (this->waypointsIt < this->waypoints.end()) {
		
			if (isnan((* this->waypointsIt).x) || isnan((* this->waypointsIt).y)) {
				ROS_WARN("Waypoint undetermined");
			}
			else if (isnan(this->currentPose.position.x) || isnan(this->currentPose.position.y)) {
				ROS_WARN("Current position undetermined");
			}
			else {
				if (this->segmentsIt < this->segments.end()) {
					unsigned int waypointIdx = std::distance(this->waypoints.begin(), this->waypointsIt);
					unsigned int idx = (*this->segmentsIt).finish_idx;
					if (waypointIdx > idx) {
						this->segmentsIt++;
					}							
				}
				switch (this->steeringMode) {
					case lunabotics::proto::ACKERMANN: this->controlAckermann(); break;
					case lunabotics::proto::POINT_TURN: this->controlPointTurn(); break;
					case lunabotics::proto::AUTO: this->controlAutomatic(); break;			
					default: ROS_WARN("Unrecognized steering mode"); break;
				}
			}
		}
		else {
			ROS_ERROR("Way iterator out of bounds");
			this->controlStop();
		}
	}
	
	if (this->predefinedControl->needsMoreControl()) {
		lunabotics::AllWheelState msg;
		this->predefinedControl->control(msg);
		this->publisherAllWheelMotion.publish(msg);
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
	pathMsg.path.header.seq = this->sequence++;
	pathMsg.path.header.frame_id = "map";
	this->publisherPath.publish(pathMsg);
}



void MotionControlNode::controlAckermann()
{
	if (this->ackermannJustStarted) {
		
		double dx = (*this->waypointsIt).x-this->currentPose.position.x;
		double dy = (*this->waypointsIt).y-this->currentPose.position.y;
		double angle = normalizedAngle(atan2(dy, dx)-this->currentPose.orientation);
		
		//Turn in place only if facing more than 30 deg away from the trajectory
		if (fabs(angle) > M_PI/6) {
			this->pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
			this->controlPointTurn();
			if (this->pointTurnMotionState != lunabotics::proto::Telemetry::TURNING) {
				this->ackermannJustStarted = false;
				//Stop turning and continue
			}
			else {
				//Return and keep turning during next cycle
				return;
			}
		}
		else {
			//This line gives all-wheel rover to take time to stop rotation
			if (!this->isDiffDriveRobot && this->predefinedControl->needsMoreControl()) {
				return;
			}
			else {
				this->ackermannJustStarted = false;
			}
		}
	}
	
	if (this->isDiffDriveRobot) {
		this->controlAckermannDiffDrive();
	}
	else {
		this->controlAckermannAllWheel();
	}
}

void MotionControlNode::controlPointTurn()
{
	if (this->waypointsIt >= this->waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	double distance = lunabotics::distance(*this->waypointsIt, this->currentPose.position);
	double dx = (*this->waypointsIt).x-this->currentPose.position.x;
	double dy = (*this->waypointsIt).y-this->currentPose.position.y;
	double angle = normalizedAngle(atan2(dy, dx)-this->currentPose.orientation);
	
	
	if (this->isDiffDriveRobot) {
		this->controlPointTurnDiffDrive(distance, angle);
	}
	else {
		this->controlPointTurnAllWheel(distance, angle);
	}
}

void MotionControlNode::controlAutomatic()
{
	if (this->segmentsIt < this->segments.end()) {
		TrajectorySegment s = *this->segmentsIt;
		if (this->steeringMode == lunabotics::proto::AUTO && 
		    this->robotGeometry->maxAvailableCurvature() < s.curve->maxCurvature()) {
			this->controlPointTurn();
		}
		else {
			this->controlAckermann();
		}
	}
	else {
		this->controlStop();
	}
}

void MotionControlNode::controlStop()
{
	this->autonomyEnabled = false;
	
	if (this->isDiffDriveRobot) {
		geometry_msgs::Twist msg;
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = 0;
		this->publisherDiffDriveMotion.publish(msg);
	}
	else {
		lunabotics::AllWheelCommon msg;
		msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
		this->publisherAllWheelCommon.publish(msg);
	}
	lunabotics::PathFollowingTelemetry msg;
	msg.is_driving = this->autonomyEnabled;
	msg.has_min_icr_radius = false;
	msg.next_waypoint_idx = this->waypointsIt < this->waypoints.end() ?  this->waypointsIt- this->waypoints.begin()+1 : 0;
	msg.segment_idx = this->segmentsIt < this->segments.end() ? this->segmentsIt-this->segments.begin()+1 : 0;
	msg.has_path_parts_enumeration = true;
	this->publisherPathFollowingTelemetry.publish(msg);
	this->waypoints.clear();
	
	
	geometry_msgs::Point ICRMsg = geometry_msgs_Point_from_Point(CreateZeroPoint());
	this->publisherICR.publish(ICRMsg);
}


void MotionControlNode::controlAckermannAllWheel()
{
	if (this->waypointsIt >= this->waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	//If suddenly skipped a waypoint, proceed with the next ones, don't get stuck with current
	double dist = distance(*this->waypointsIt, this->currentPose.position);
	for (PointArr::iterator it = this->waypointsIt+1; it < this->waypoints.end(); it++) {
		double newDist = distance(*it, this->currentPose.position);
		if (newDist < dist) {
			this->waypointsIt = it;
			dist = newDist;
		}
	}
	
	if (dist < this->motionConstraints.point_turn_distance_accuracy) {
		this->waypointsIt++;
	}
	if (this->waypointsIt >= this->waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	
	double dx = (*this->waypointsIt).x-this->currentPose.position.x;
	double dy = (*this->waypointsIt).y-this->currentPose.position.y;
	
	if (this->waypoints.size() >= 2) {
		
		//In the beginning turn in place towards the second waypoint (first waypoint is at the robot's position). It helps to solve problems with pid
		if (this->waypointsIt < this->waypoints.begin()+2) {
			this->waypointsIt = this->waypoints.begin()+1;
			double angle = normalizedAngle(atan2(dy, dx)-this->currentPose.orientation);
			if (fabs(angle) > this->motionConstraints.point_turn_angle_accuracy) {
				//ROS_WARN("Facing away from the trajectory. Turning in place");
			//	controlSkid();
			//	return;
			}
		}
		
		
		double y_err = this->pathFollowingGeometry->getFeedbackError();
		double prediction = this->pathFollowingGeometry->getFeedforwardPrediction();
		double heading_error = this->pathFollowingGeometry->getHeadingError();
		lunabotics::PathFollowingTelemetry telemetryMsg;
		telemetryMsg.feedback_error = y_err;
		telemetryMsg.heading_error = heading_error;
		telemetryMsg.feedforward_prediction = prediction;
		telemetryMsg.feedforward_curve_radius = this->pathFollowingGeometry->getCurveRadius();
		telemetryMsg.feedback_path_point = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPathPoint());
		telemetryMsg.feedback_point = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPoint());
		telemetryMsg.feedback_path_point_local = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPathPointInLocalFrame());
		telemetryMsg.feedback_point_local = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPointInLocalFrame());
		telemetryMsg.feedforward_center = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedforwardCurveCenterPoint());
		telemetryMsg.next_waypoint_idx = this->waypointsIt < this->waypoints.end() ? this->waypointsIt-this->waypoints.begin()+1 : 0;
		telemetryMsg.segment_idx = this->segmentsIt < this->segments.end() ? this->segmentsIt-this->segments.begin()+1 : 0;
		telemetryMsg.has_path_following_geometry = true;
		telemetryMsg.has_path_parts_enumeration = true;
		telemetryMsg.has_min_icr_radius = true;
		telemetryMsg.min_icr_radius = this->minICRRadius;
		telemetryMsg.is_driving = this->autonomyEnabled;
		
		PointArr pts = this->pathFollowingGeometry->getCurvatureDetectionPointsInLocalFrame();
		for (unsigned int i = 0; i < pts.size(); i++) {
			telemetryMsg.feedforward_points_local.push_back(geometry_msgs_Point_from_Point(pts.at(i)));
		}
		
		//Control law
		
		double signal;
		
		//Original control
		
		if (fabs(heading_error) < 0.001) {
			lunabotics::CrabControl ctrl;
			ctrl.velocity = this->linearVelocity;
			ctrl.heading = std::min(y_err*10, M_PI_4);
			this->publisherCrab.publish(ctrl);
		}
		else {
		
			//
			if (this->ackermannPID->control(y_err, signal)) {
				signal *= 10.0;
				
				if (!isnan(prediction) && !isinf(prediction)) {
					prediction /= 10;
					signal += prediction;
				}
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
				double offset_x = this->robotGeometry->left_front().x;
				double offset_y = tan(alpha)*offset_x;
				
				Point ICR = CreatePoint(0, offset_y);
				
				float abs_offset_y = fabs(offset_y);
				
				
				ICR = this->robotGeometry->point_outside_base_link(ICR);
				
				
				if (abs_offset_y > 0.0001 && (abs_offset_y < this->minICRRadius || this->minICRRadius < 0.0001)) {
					this->minICRRadius = abs_offset_y;
					ROS_INFO("Radius %f (corrected radius %f)", abs_offset_y, ICR.y);
					telemetryMsg.min_icr_radius = abs_offset_y;
					telemetryMsg.has_min_icr_radius = true;
				}
				
				
				this->publisherICR.publish(geometry_msgs_Point_from_Point(ICR));
				
				//ROS_INFO("Alpha %.2f offset %.2f ICR %.2f", alpha, offset_y, ICR.y);
				
				float velocity = this->motionConstraints.lin_velocity_limit;
				
				float angle_front_left;
				float angle_front_right;
				float angle_rear_left;
				float angle_rear_right;
				if (this->robotGeometry->calculateAngles(ICR, angle_front_left, angle_front_right, angle_rear_left, angle_rear_right)) {
					
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
					if (!this->robotGeometry->calculateVelocities(ICR, velocity, vel_front_left, vel_front_right, vel_rear_left, vel_rear_right)) {
						vel_front_left = vel_front_right = vel_rear_left = vel_rear_right = 0;			
					}
					controlMsg.driving.left_front = vel_front_left;
					controlMsg.driving.right_front = vel_front_right;
					controlMsg.driving.left_rear = vel_rear_left;
					controlMsg.driving.right_rear = vel_rear_right;
					
					this->publisherAllWheelMotion.publish(controlMsg);
				}
			}
		}
		this->publisherPathFollowingTelemetry.publish(telemetryMsg);
	}
	//*/
	else {
		//No need for curvature, just straight driving
		this->controlPointTurn();
	}
}

void MotionControlNode::controlAckermannDiffDrive()
{
	if (this->waypointsIt >= this->waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	//If suddenly skipped a waypoint, proceed with the next ones, don't get stuck with current
	double dist = distance(*this->waypointsIt, this->currentPose.position);
	for (PointArr::iterator it = this->waypointsIt+1; it < this->waypoints.end(); it++) {
		double newDist = distance(*it, this->currentPose.position);
		if (newDist < dist) {
			this->waypointsIt = it;
			dist = newDist;
		}
	}
	
	if (dist < this->motionConstraints.point_turn_distance_accuracy) {
		this->waypointsIt++;
	}
	if (this->waypointsIt >= this->waypoints.end()) {
		this->finalizeRoute();
		return;
	}
	
	
	double dx = (*this->waypointsIt).x-this->currentPose.position.x;
	double dy = (*this->waypointsIt).y-this->currentPose.position.y;
	
	if (this->waypoints.size() >= 2) {
		
		//In the beginning turn in place towards the second waypoint (first waypoint is at the robot's position). It helps to solve problems with pid
		if (this->waypointsIt < this->waypoints.begin()+2) {
			this->waypointsIt = this->waypoints.begin()+1;
			double angle = normalizedAngle(atan2(dy, dx)-this->currentPose.orientation);
			if (fabs(angle) > this->motionConstraints.point_turn_angle_accuracy) {
				//ROS_WARN("Facing away from the trajectory. Turning in place");
				this->controlPointTurn();
				return;
			}
		}
		
		
		
		double y_err = this->pathFollowingGeometry->getFeedbackError();
		lunabotics::PathFollowingTelemetry telemetryMsg;
		telemetryMsg.feedback_path_point = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPathPoint());
		telemetryMsg.feedback_point = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPoint());
		telemetryMsg.feedback_error = y_err;
		telemetryMsg.is_driving = this->autonomyEnabled;
		telemetryMsg.feedback_path_point_local = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPathPointInLocalFrame());
		telemetryMsg.feedback_point_local = geometry_msgs_Point_from_Point(this->pathFollowingGeometry->getFeedbackPointInLocalFrame());
		telemetryMsg.next_waypoint_idx = this->waypointsIt < this->waypoints.end() ? this->waypointsIt-this->waypoints.begin()+1 : 0;
		telemetryMsg.segment_idx = this->segmentsIt < this->segments.end() ? this->segmentsIt-this->segments.begin()+1 : 0;
		telemetryMsg.has_path_following_geometry = true;
		telemetryMsg.has_min_icr_radius = false;
		telemetryMsg.has_path_parts_enumeration = true;
		this->publisherPathFollowingTelemetry.publish(telemetryMsg);
		
		//Control law
		
		double dw;
		if (this->ackermannPID->control(y_err, dw)) {
			dw *= -3;
			//ROS_WARN("DW %.2f", dw);
			
			//The higher angular speed, the lower linear speed is
			double top_w = 1.57;
			double v = this->motionConstraints.lin_velocity_limit * std::max(0.0, (top_w-fabs(dw)))/top_w;
			v = std::max(0.01, v);
			v = std::min(this->motionConstraints.lin_velocity_limit, v);
				
			geometry_msgs::Twist twistMsg;
			twistMsg.linear.x = v;
			twistMsg.linear.y = 0;
			twistMsg.linear.z = 0;
			twistMsg.angular.x = 0;
			twistMsg.angular.y = 0;
			twistMsg.angular.z = dw;
			this->publisherDiffDriveMotion.publish(twistMsg);
		}
	}
	else {
		//No need for curvature, just straight driving
		this->controlPointTurn();
	}
}
void MotionControlNode::controlPointTurnAllWheel(double distance, double theta)
{
	lunabotics::AllWheelCommon msg;
	msg.wheel_velocity = DEFAULT_WHEEL_VELOCITY;

	lunabotics::PathFollowingTelemetry telemetryMsg;
	telemetryMsg.is_driving = this->autonomyEnabled;
	bool publishCommonMsg = true;
	
	switch (this->pointTurnMotionState) {
		case lunabotics::proto::Telemetry::STOPPED: {
			//ROS_INFO("SKID: stopped dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			
			if (distance <= this->motionConstraints.point_turn_distance_accuracy) {
				this->waypointsIt++;
				if (this->waypointsIt >= this->waypoints.end()) {
					this->finalizeRoute();
				}
				else {
					telemetryMsg.has_path_parts_enumeration = true; 
					telemetryMsg.next_waypoint_idx = this->waypointsIt < this->waypoints.end() ? this->waypointsIt-this->waypoints.begin()+1 : 0;	
					telemetryMsg.segment_idx = this->segmentsIt < this->segments.end() ? this->segmentsIt-this->segments.begin()+1 : 0;
					//lunabotics::Point nextWaypoint = *this->waypointsIt;
					//ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypoint.position.x, nextWaypoint.position.y);
				}
			}
			else if (fabs(theta) > this->motionConstraints.point_turn_angle_accuracy) {
				if (fabs(this->linearVelocity) < LIN_VEL_ACCURACY) {
					this->pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
				}
			}
			else if (distance > this->motionConstraints.point_turn_distance_accuracy) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::DRIVING;
			}				
		}	
		break;
		case lunabotics::proto::Telemetry::DRIVING: {	
			//ROS_INFO("SKID: driving        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			if (distance < this->motionConstraints.point_turn_distance_accuracy) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
			}
			else if (fabs(theta) > 0.3) {
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
				if (fabs(this->linearVelocity) < LIN_VEL_ACCURACY) {
					this->pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
				}
			}	
			else {
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::DRIVE_FORWARD;
			}
		}
		break;
		case lunabotics::proto::Telemetry::TURNING: {
			
			int direction = sign(theta, this->motionConstraints.point_turn_angle_accuracy);
			
			ros::Time now = ros::Time::now();
			ros::Duration dt = now-this->previousYawTime;
			double yawRate = (this->currentPose.orientation-this->previousYaw)/dt.toSec();
			
			if (direction == 0 && fabs(yawRate) < YAW_RATE_ACCURACY) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
				msg.predefined_cmd = lunabotics::proto::AllWheelControl::STOP;
			}
			else {
				double signal;
				if (this->pointTurnPID->control(theta, signal)) {
					msg.wheel_velocity = MIN(DEFAULT_WHEEL_VELOCITY, fabs(signal));
					if (signal > 0) {
						msg.predefined_cmd = lunabotics::proto::AllWheelControl::TURN_CCW;
					}
					else {
						msg.predefined_cmd = lunabotics::proto::AllWheelControl::TURN_CW;
					}
				}
				else {
					publishCommonMsg = false;
				}
			}
			this->previousYaw = this->currentPose.orientation;
			this->previousYawTime = now;
		}
		break;
		default: break;
	}
	
	telemetryMsg.point_turn_state = this->pointTurnMotionState;
	telemetryMsg.has_point_turn_state = true;
	telemetryMsg.has_min_icr_radius = false;
	this->publisherPathFollowingTelemetry.publish(telemetryMsg);
	if (publishCommonMsg) {
		this->publisherAllWheelCommon.publish(msg);
	}
}

void MotionControlNode::controlPointTurnDiffDrive(double distance, double theta)
{
	lunabotics::PathFollowingTelemetry telemetryMsg;
	telemetryMsg.is_driving = this->autonomyEnabled;
	telemetryMsg.has_point_turn_state = true;
	geometry_msgs::Twist twistMsg;
	twistMsg.linear.x = 0;
	twistMsg.linear.y = 0;
	twistMsg.linear.z = 0;
	twistMsg.angular.x = 0;
	twistMsg.angular.y = 0;
	twistMsg.angular.z = 0;
					
	switch (this->pointTurnMotionState) {
		case lunabotics::proto::Telemetry::STOPPED: {
			//ROS_INFO("SKID: stopped        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			
			if (distance < this->motionConstraints.point_turn_distance_accuracy) {
				 this->waypointsIt++;
				if ( this->waypointsIt >= this->waypoints.end()) {
					this->finalizeRoute();
				}
				else {
					telemetryMsg.has_path_parts_enumeration = true;
					telemetryMsg.next_waypoint_idx =  this->waypointsIt < this->waypoints.end() ?  this->waypointsIt-this->waypoints.begin()+1 : 0;
					telemetryMsg.segment_idx = this->segmentsIt < this->segments.end() ? this->segmentsIt-this->segments.begin()+1 : 0;
					//lunabotics::Point nextWaypoint = * this->waypointsIt;
					//ROS_INFO("Waypoint reached. Now heading towards (%.1f,%.1f)", nextWaypoint.position.x, nextWaypoint.position.y);
				}
			}
			else if (fabs(theta) > this->motionConstraints.point_turn_angle_accuracy) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
			}
			else if (distance > this->motionConstraints.point_turn_distance_accuracy) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::DRIVING;
			}				
		}	
		break;
		case lunabotics::proto::Telemetry::DRIVING: {	
		//	ROS_INFO("SKID: driving        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);	
			if (distance < this->motionConstraints.point_turn_distance_accuracy) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
				twistMsg.linear.x = 0;
				twistMsg.angular.z = 0;
			}
			else if (fabs(theta) > this->motionConstraints.point_turn_angle_accuracy) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::TURNING;
			}	
			else {
				twistMsg.linear.x = 1;
				twistMsg.angular.z = 0;
			}
		}
		break;
		case lunabotics::proto::Telemetry::TURNING: {
			//ROS_INFO("SKID: turning        dx: %.5f dy: %.5f angle: %.5f", dx, dy, angle);
			int direction = sign(theta, this->motionConstraints.point_turn_angle_accuracy);
		
			if (direction == 0) {
				this->pointTurnMotionState = lunabotics::proto::Telemetry::STOPPED;
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
	
	telemetryMsg.point_turn_state = this->pointTurnMotionState;
	telemetryMsg.has_min_icr_radius = false;
	this->publisherPathFollowingTelemetry.publish(telemetryMsg);
	this->publisherDiffDriveMotion.publish(twistMsg);
}


//------------------ PLANNING METHODS ----------------------------------

bool MotionControlNode::getMapIfNeeded()
{
	if (!this->cached_map_up_to_date) {
		if (!this->clientMap.call(this->serviceMap)) {
			ROS_WARN("Failed to get a map from the service");
			return false;
		}
		else {
			this->cached_map = this->serviceMap.response.map;
			this->cached_map_up_to_date = true;
			ROS_INFO("Got map from service (%d cells)", (int)this->cached_map.data.size());
		}
	}
	return true;
}

void MotionControlNode::run()
{
	//Let the parent do the trick
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
