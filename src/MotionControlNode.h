#ifndef _MOTION_CONTROL_NODE_H_
#define _MOTION_CONTROL_NODE_H_

#include "ROSNode.h"
#include "types.h"
#include "planning/planning.h"
#include "control/control.h"
#include "geometry/geometry.h"

//Transforms
#include <tf/transform_listener.h>
#include "message_filters/subscriber.h"

//Protos
#include "../protos_gen/AllWheelControl.pb.h"
#include "../protos_gen/Telemetry.pb.h"

//Topics and Services
#include "lunabotics/Emergency.h"
#include "lunabotics/Goal.h"
#include "lunabotics/ICRControl.h"
#include "lunabotics/AllWheelCommon.h"
#include "lunabotics/AllWheelState.h"
#include "lunabotics/SteeringMode.h"
#include "lunabotics/CrabControl.h"
#include "lunabotics/PID.h"
#include "lunabotics/State.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

namespace lunabotics {

struct MotionConstraints {
	double lin_velocity_limit;
	int bezier_segments_num;
	double point_turn_angle_accuracy;
	double point_turn_distance_accuracy;
};


class MotionControlNode : ROSNode {
private:
	ros::Publisher publisherDiffDriveMotion;
	ros::Publisher publisherAllWheelMotion;
	ros::Publisher publisherPath;
	ros::Publisher publisherICR;
	ros::Publisher publisherPathFollowingTelemetry;
	ros::Publisher publisherGeometry;
	ros::Publisher publisherAllWheelCommon;

	ros::Subscriber subscriberEmergency;
	ros::Subscriber subscriberAutonomy;
	ros::Subscriber subscriberState;
	ros::Subscriber subscriberGoal;
	ros::Subscriber subscriberPID;
	ros::Subscriber subscriberSteeringMode;
	ros::Subscriber subscriberICR;
	ros::Subscriber subscriberCrab;
	ros::Subscriber subscriberAllWheelCommon;
	ros::Subscriber subscriberAllWheelFeedback;
	
	ros::ServiceClient clientMap;

	nav_msgs::GetMap serviceMap;
	
	nav_msgs::OccupancyGrid cached_map;
	bool cached_map_up_to_date;
	
	int sequence;
	bool autonomyEnabled;
	lunabotics::proto::SteeringModeType steeringMode;
	lunabotics::proto::Telemetry::PointTurnState pointTurnMotionState;
	lunabotics::Pose currentPose;
	lunabotics::TrajectorySegmentArr::iterator segmentsIt;
	lunabotics::TrajectorySegmentArr segments;
	lunabotics::PointArr::iterator waypointsIt;
	lunabotics::PointArr waypoints;
	lunabotics::PointArr desiredWaypoints;
	MotionConstraints motionConstraints;
	double minICRRadius;
	bool ackermannJustStarted;
	double previousYaw;
	ros::Time previousYawTime;
	double linearVelocity;
	
	lunabotics::PredefinedCmdControllerPtr predefinedControl;
	lunabotics::PIDControllerPtr ackermannPID;
	lunabotics::PIDControllerPtr pointTurnPID;
	lunabotics::AllWheelGeometryPtr robotGeometry;
	lunabotics::PathFollowingGeometryPtr pathFollowingGeometry;
	lunabotics::TrajectoryPtr trajectory;
	
	tf::TransformListener tfListener;
	
	//Topic callbacks
	void callbackGoal(const lunabotics::Goal::ConstPtr &msg);
	void callbackICR(const lunabotics::ICRControl::ConstPtr &msg);
	void callbackAutonomy(const std_msgs::Bool::ConstPtr &msg);
	void callbackAllWheelCommon(const lunabotics::AllWheelCommon::ConstPtr &msg);
	void callbackAllWheelFeedback(const lunabotics::AllWheelState::ConstPtr &msg);
	void callbackSteeringMode(const lunabotics::SteeringMode::ConstPtr &msg);
	void callbackPID(const lunabotics::PID::ConstPtr &msg);
	void callbackCrab(const lunabotics::CrabControl::ConstPtr &msg);
	void callbackState(const lunabotics::State::ConstPtr &msg);
	void callbackEmergency(const lunabotics::Emergency::ConstPtr &msg);
	
	//Control techniques
	void controlAckermann();
	void controlPointTurn();
	void controlAutomatic();
	void controlStop();
	
	void controlAckermannAllWheel();
	void controlAckermannDiffDrive();
	void controlPointTurnAllWheel(double distance, double theta);
	void controlPointTurnDiffDrive(double distance, double theta);
	
	void finalizeRoute();
	
	//Path planning
	bool getMapIfNeeded();
	
	
	void runOnce();
public:
	MotionControlNode(int argc, char **argv, std::string name, int frequency);
	~MotionControlNode();
	
	void run();
};




}



#endif
