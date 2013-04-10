#ifndef _LUNABOTICS_LIDAR_PLUGIN_H_
#define _LUNABOTICS_LIDAR_PLUGIN_H_

#include <gazebo.hh>
#include "ros/ros.h"

namespace gazebo
{

	class LunaboticsLidarPlugin : public SensorPlugin
	{
	public:
		LunaboticsLidarPlugin();
		~LunaboticsLidarPlugin();
		void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
	private:
		void OnUpdate();
	
		sensors::RaySensorPtr sensor;
	
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;
	
		// ROS Nodehandle
		ros::NodeHandle* node;
	
		// ROS Subscriber
		ros::Publisher pub;
	};
	
};
#endif
