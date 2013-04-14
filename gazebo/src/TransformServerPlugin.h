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
		bool LoadParams(sdf::ElementPtr _sdf);
		bool FindLinkByName(sdf::ElementPtr _sdf, physics::LinkPtr &_link, const std::string _name);
		void OnUpdate();
		
		// Pointer to the model
		physics::ModelPtr model;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
		
		physics::LinkPtr lidarLink;
		
		// ROS Nodehandle
		ros::NodeHandle* node;
		
		// Publishers
		tf::TransformBroadcaster tfBroadcaster;
	};
	
};
#endif
