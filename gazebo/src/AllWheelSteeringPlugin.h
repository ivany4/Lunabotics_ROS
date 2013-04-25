#ifndef _ALL_WHEEL_STEERING_PLUGIN_H_
#define _ALL_WHEEL_STEERING_PLUGIN_H_

#include <gazebo.hh>
#include <common/common.hh>
#include "ros/ros.h"
#include "lunabotics/AllWheelStateROS.h"
#include "../../src/control/PIDController.h"

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
		void ROSCallback(const lunabotics::AllWheelStateROS::ConstPtr& msg);
		void OnUpdate();
		double DrivingFromSteeringVelocity(double steeringVel);
		
		
		
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
		
		double leftFrontSteeringAngle;
		double rightFrontSteeringAngle;
		double leftRearSteeringAngle;
		double rightRearSteeringAngle;
		
		double leftFrontDrivingSpeed;
		double rightFrontDrivingSpeed;
		double leftRearDrivingSpeed;
		double rightRearDrivingSpeed;
		
		double wheelRadius;
		double linkShoulder;
		
		lunabotics::control::PIDControllerPtr leftFrontPID;
		lunabotics::control::PIDControllerPtr rightFrontPID;
		lunabotics::control::PIDControllerPtr leftRearPID;
		lunabotics::control::PIDControllerPtr rightRearPID;
		
		// ROS Nodehandle
		ros::NodeHandle* node;
	
		// ROS Subscriber
		ros::Subscriber sub;
		
		// Publishers
		ros::Publisher wheelStatePublisher;
	};
	
};
#endif
