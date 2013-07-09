#ifndef _LUNABOTICS_ODOMETRY_PLUGIN_H_
#define _LUNABOTICS_ODOMETRY_PLUGIN_H_

#include <gazebo.hh>
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include "tf/transform_broadcaster.h"

namespace gazebo
{

	class LunaboticsOdometryPlugin : public ModelPlugin
	{
	public:
		LunaboticsOdometryPlugin();
		~LunaboticsOdometryPlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	private:
		void OnUpdate();
		
		// Pointer to the model
		physics::ModelPtr model;
		physics::LinkPtr baseLink;
		ros::Time previousTime;
		
		math::Pose previousPose;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
	
		// ROS Nodehandle
		ros::NodeHandle* node;
	
		// ROS Subscriber
		ros::Publisher pub;
		tf::TransformBroadcaster tfBroadcaster;
	};
	
};
#endif
