#ifndef _ALL_WHEEL_STEERING_PLUGIN_H_
#define _ALL_WHEEL_STEERING_PLUGIN_H_

#include <gazebo.hh>
#include "ros/ros.h"
//#include <lunabotics/AllWheelSteering.h>
#include "../../msg_gen/cpp/include/lunabotics/AllWheelSteering.h"

namespace gazebo
{

	class AllWheelSteeringPlugin : public ModelPlugin
	{
	public:
		AllWheelSteeringPlugin();
		~AllWheelSteeringPlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	private:
		bool LoadParams(sdf::ElementPtr _sdf);
		bool FindJointByParam(sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string _param);
		void ROSCallback(const lunabotics::AllWheelSteering::ConstPtr& msg);
		void OnUpdate();
	
		// Pointer to the model
		physics::ModelPtr model;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
	
		physics::JointPtr leftFrontWheelSteeringJoint;
		physics::JointPtr rightFrontWheelSteeringJoint;
		physics::JointPtr leftRearWheelSteeringJoint;
		physics::JointPtr rightRearWheelSteeringJoint;
		physics::JointPtr leftFrontWheelDrivingJoint;
		physics::JointPtr rightFrontWheelDrivingJoint;
		physics::JointPtr leftRearWheelDrivingJoint;
		physics::JointPtr rightRearWheelDrivingJoint;
		
		// ROS Nodehandle
		ros::NodeHandle* node;
	
		// ROS Subscriber
		ros::Subscriber sub;
	};
	
};
#endif
