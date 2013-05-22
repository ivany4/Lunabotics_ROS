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
#include "lunabotics/ControlMode.h"
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
	ros::Publisher _publisherDiffDriveMotion;
	ros::Publisher _publisherAllWheelMotion;
	ros::Publisher _publisherPath;
	ros::Publisher _publisherICR;
	ros::Publisher _publisherControlParams;
	ros::Publisher _publisherGeometry;
	ros::Publisher _publisherAllWheelCommon;

	ros::Subscriber _subscriberEmergency;
	ros::Subscriber _subscriberAutonomy;
	ros::Subscriber _subscriberState;
	ros::Subscriber _subscriberGoal;
	ros::Subscriber _subscriberPID;
	ros::Subscriber _subscriberSteeringMode;
	ros::Subscriber _subscriberICR;
	ros::Subscriber _subscriberCrab;
	ros::Subscriber _subscriberAllWheelCommon;
	ros::Subscriber _subscriberAllWheelFeedback;
	
	ros::ServiceClient _clientMap;

	nav_msgs::GetMap _serviceMap;
	
	nav_msgs::OccupancyGrid _cached_map;
	bool _cached_map_up_to_date;
	
	int _sequence;
	bool _autonomyEnabled;
	lunabotics::proto::SteeringModeType _steeringMode;
	lunabotics::proto::Telemetry::PointTurnState _pointTurnMotionState;
	lunabotics::Pose _currentPose;
	lunabotics::PointArr::iterator _waypointsIt;
	lunabotics::PointArr _waypoints;
	lunabotics::PointArr _desiredWaypoints;
	MotionConstraints _motionConstraints;
	double _minICRRadius;
	bool _ackermannJustStarted;
	
	lunabotics::PredefinedCmdControllerPtr _predefinedControl;
	lunabotics::PIDControllerPtr _PID;
	lunabotics::AllWheelGeometryPtr _geometryHelper;
	lunabotics::PIDGeometryPtr _PIDHelper;
	lunabotics::TrajectoryPtr _trajectory;
	
	tf::TransformListener _tfListener;
	
	//Topic callbacks
	void callbackGoal(const lunabotics::Goal::ConstPtr &msg);
	void callbackICR(const lunabotics::ICRControl::ConstPtr &msg);
	void callbackAutonomy(const std_msgs::Bool::ConstPtr &msg);
	void callbackAllWheelCommon(const lunabotics::AllWheelCommon::ConstPtr &msg);
	void callbackAllWheelFeedback(const lunabotics::AllWheelState::ConstPtr &msg);
	void callbackSteeringMode(const lunabotics::ControlMode::ConstPtr &msg);
	void callbackPID(const lunabotics::PID::ConstPtr &msg);
	void callbackCrab(const lunabotics::CrabControl::ConstPtr &msg);
	void callbackState(const lunabotics::State::ConstPtr &msg);
	void callbackEmergency(const lunabotics::Emergency::ConstPtr &msg);
	
	//Control techniques
	void controlAckermann();
	void controlPointTurn();
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
