#ifndef _TRANSFORM_SERVER_PLUGIN_H_
#define _TRANSFORM_SERVER_PLUGIN_H_

#include <gazebo.hh>
#include <common/common.hh>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

namespace gazebo
{

	class TransformServerPlugin : public ModelPlugin
	{
	public:
		TransformServerPlugin();
		~TransformServerPlugin();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	private:
		bool LoadParams();
		void OnUpdate();
		
		tf::Transform leftFrontT;
		tf::Transform rightFrontT;
		tf::Transform leftRearT;
		tf::Transform rightRearT;
		tf::Transform wheelOffsetT;
		tf::Transform wheelRadiusT;
		tf::Transform lidarT;
		tf::Transform odomT;
		
		// Pointer to the model
		physics::ModelPtr model;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
		
		physics::LinkPtr lidarLink;
		physics::JointPtr leftFrontJoint;
		physics::JointPtr leftRearJoint;
		physics::JointPtr rightFrontJoint;
		physics::JointPtr rightRearJoint;
		physics::JointPtr leftFrontWheelJoint;
		
		// ROS Nodehandle
		ros::NodeHandle* node;
		
		// Publishers
		tf::TransformBroadcaster tfBroadcaster;
	};
	
};
#endif
