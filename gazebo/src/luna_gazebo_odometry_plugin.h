#ifndef LUNA_GAZEBO_ODOMETRY_PLUGIN_H
#define LUNA_GAZEBO_ODOMETRY_PLUGIN_H

#include <gazebo.hh>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

namespace gazebo
{

	class LunaOdometryPlugin : public ModelPlugin
	{
	public:
		LunaOdometryPlugin();
		~LunaOdometryPlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	private:
		void OnUpdate();
	
		// Pointer to the model
		physics::ModelPtr model;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
	
		// ROS Nodehandle
		ros::NodeHandle* node;
	
		// ROS Subscriber
		ros::Publisher pub;
	};
	
};
#endif
