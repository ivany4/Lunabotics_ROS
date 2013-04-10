#ifndef _LUNABOTICS_DIFF_DRIVE_PLUGIN_H_
#define _LUNABOTICS_DIFF_DRIVE_PLUGIN_H_

#include <gazebo.hh>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace gazebo
{

	class LunaboticsDiffDrivePlugin : public ModelPlugin
	{
	public:
		LunaboticsDiffDrivePlugin();
		~LunaboticsDiffDrivePlugin();
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
