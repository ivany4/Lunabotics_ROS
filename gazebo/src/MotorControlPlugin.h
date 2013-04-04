#ifndef ROOMBA_MOTOR_CONTROLLER_PLUGIN_H
#define ROOMBA_MOTOR_CONTROLLER_PLUGIN_H

#include <gazebo.hh>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace gazebo
{

	class ROSMotorControllerPlugin : public ModelPlugin
	{
	public:
		ROSMotorControllerPlugin();
		~ROSMotorControllerPlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	private:
		bool LoadParams(sdf::ElementPtr _sdf);
		bool FindJointByParam(sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string _param);
		void ROSCallback(const geometry_msgs::Twist::ConstPtr& msg);
		void OnUpdate();
	
		// Pointer to the model
		physics::ModelPtr model;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
	
		physics::JointPtr leftWheelJoint;
		physics::JointPtr rightWheelJoint;
		double torque;
		double wheelSeparation;
		double wheelRadius;
	
		// ROS Nodehandle
		ros::NodeHandle* node;
	
		// ROS Subscriber
		ros::Subscriber sub;
	};
	
};
#endif
