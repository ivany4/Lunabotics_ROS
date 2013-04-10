#ifndef _CLOCK_SERVER_H_
#define _CLOCK_SERVER_H_

#include <gazebo.hh>
#include <common/common.hh>
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"

namespace gazebo
{
	class ClockServerPlugin : public WorldPlugin
	{
	public:
		ClockServerPlugin();
		~ClockServerPlugin();
		void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
	private:
		void OnUpdate();
		
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
		
		// ROS Nodehandle
		ros::NodeHandle* node;
		
		// Publishers
		ros::Publisher clockPublisher;
	};
	
};
#endif
