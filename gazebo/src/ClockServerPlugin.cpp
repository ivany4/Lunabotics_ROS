#include "ClockServerPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <stdio.h>

namespace gazebo
{   
	ClockServerPlugin::ClockServerPlugin() {
		std::string name = "gazebo_clock_server";
	    int argc = 0;
		ros::init(argc, NULL, name);
	}
	
	ClockServerPlugin::~ClockServerPlugin() {
		delete this->node;
	}
	
	void ClockServerPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
		ROS_INFO("Loading Clock World Plugin");
		this->node = new ros::NodeHandle("~");
	
		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ClockServerPlugin::OnUpdate, this));

		// ROS Publisher
		this->clockPublisher = this->node->advertise<rosgraph_msgs::Clock>("/clock", 256);
		
		ROS_INFO("\n\n===========\n\nClock server ready\n\n===========\n\n");
	}
	
	void ClockServerPlugin::OnUpdate() 
	{		
		rosgraph_msgs::Clock clockMsg;
		ros::Time now(common::Time::GetWallTime().Double());
		clockMsg.clock = now;
		this->clockPublisher.publish(clockMsg);		
		ros::spinOnce();
	}
		
	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(ClockServerPlugin);
}
