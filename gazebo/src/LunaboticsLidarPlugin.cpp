#include "LunaboticsLidarPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <common/common.hh>
#include <stdio.h>
#include <sensor_msgs/LaserScan.h>

uint32_t seq = 0;

namespace gazebo
{   
	LunaboticsLidarPlugin::LunaboticsLidarPlugin() {
		std::string name = "gazebo_interface";
	    int argc = 0;
		ros::init(argc, NULL, name);
	}
	
	LunaboticsLidarPlugin::~LunaboticsLidarPlugin() {
		delete this->node;
	}
	
	void LunaboticsLidarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
		ROS_INFO("Loading Lidar Sensor Plugin");
		this->sensor = boost::shared_dynamic_cast<sensors::RaySensor>(_sensor);
		
		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LunaboticsLidarPlugin::OnUpdate, this));

		this->node = new ros::NodeHandle("~");

		// ROS Publisher
		this->pub = this->node->advertise<sensor_msgs::LaserScan>("/base_scan", 1000);
		
		if (!pub) {
			ROS_ERROR("Could not advertise /base_scan!");
		}
	}
	
	
	// Called by the world update start event
	void LunaboticsLidarPlugin::OnUpdate() {

		ros::Time now = ros::Time::now();

		sensor_msgs::LaserScan msg;
		msg.header.seq = seq++;
		msg.header.stamp = now;
		msg.header.frame_id = "lidar";
		msg.angle_min = this->sensor->GetAngleMin().Degree();
		msg.angle_max = this->sensor->GetAngleMax().Degree();
		msg.angle_increment = this->sensor->GetAngleResolution();
		msg.time_increment = 0;
		msg.scan_time = 1.0/this->sensor->GetUpdateRate();
		msg.range_min = this->sensor->GetRangeMin();
		msg.range_max = this->sensor->GetRangeMax();
		std::vector<double> ranges;
		this->sensor->GetRanges(ranges);
		for (std::vector<double>::iterator it = ranges.begin(); it < ranges.end(); it++) {
			msg.ranges.push_back((float)(*it));
		}
		this->pub.publish(msg);
            
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_SENSOR_PLUGIN(LunaboticsLidarPlugin);
}
