#ifndef _ALL_WHEEL_STEERING_PLUGIN_H_
#define _ALL_WHEEL_STEERING_PLUGIN_H_

#include <gazebo.hh>
#include <common/common.hh>
#include "ros/ros.h"
#include "lunabotics/AllWheelState.h"
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
		bool LoadParams();
		void ROSCallback(const lunabotics::AllWheelState::ConstPtr& msg);
		void OnUpdate();
		double DrivingFromSteeringVelocity(double steeringVel);
		
		// Pointer to the model
		physics::ModelPtr model;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
	
		physics::JointPtr leftFrontSteeringJoint;
		physics::JointPtr leftRearSteeringJoint;
		physics::JointPtr rightFrontSteeringJoint;
		physics::JointPtr rightRearSteeringJoint;
		physics::JointPtr leftFrontDrivingJoint;
		physics::JointPtr rightFrontDrivingJoint;
		physics::JointPtr leftRearDrivingJoint;
		physics::JointPtr rightRearDrivingJoint;
		
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
		
		lunabotics::PIDControllerPtr leftFrontPID;
		lunabotics::PIDControllerPtr rightFrontPID;
		lunabotics::PIDControllerPtr leftRearPID;
		lunabotics::PIDControllerPtr rightRearPID;
		
		// ROS Nodehandle
		ros::NodeHandle* node;
	
		// ROS Subscriber
		ros::Subscriber sub;
		
		// Publishers
		ros::Publisher wheelStatePublisher;
	};
	
};
#endif
