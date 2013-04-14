#include "LunaboticsOdometryPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

namespace gazebo
{   
	LunaboticsOdometryPlugin::LunaboticsOdometryPlugin() {
		std::string name = "gazebo_odometry_plugin";
	    int argc = 0;
		ros::init(argc, NULL, name);
	}
	
	LunaboticsOdometryPlugin::~LunaboticsOdometryPlugin() {
		delete this->node;
	}
	
	void LunaboticsOdometryPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		ROS_INFO("Loading Odometry Plugin");
		
		this->model = _parent;
	
		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LunaboticsOdometryPlugin::OnUpdate, this));

		this->node = new ros::NodeHandle("~");

		// ROS Publisher
		this->pub = this->node->advertise<nav_msgs::Odometry>("/odom", 1000);
		
		if (!pub) {
			ROS_ERROR("Could not advertise /odom!");
		}
	}
	
	
	// Called by the world update start event
	void LunaboticsOdometryPlugin::OnUpdate() {
		
		std::vector<double> rangesgz;

		math::Vector3 p = model->GetWorldPose().pos;
		math::Quaternion r = model->GetWorldPose().rot;
		
		geometry_msgs::Point p2ros;
		p2ros.x=p.x; p2ros.y=p.y; p2ros.z=p.z;
		geometry_msgs::Quaternion r2ros;
		r2ros.x=r.x; r2ros.y=r.y; r2ros.z=r.z; r2ros.w=r.w;

		nav_msgs::Odometry odom2ros;
		odom2ros.header.stamp=ros::Time::now();
		odom2ros.header.frame_id="/odom";
		odom2ros.child_frame_id="/base_link";
		odom2ros.pose.pose.position=p2ros;
		odom2ros.pose.pose.orientation=r2ros;
		//odom2ros.pose.covariance= //covariance 6x6 matrix
		
		
		//Quaternion to Euler angles
		static double qw=r.w; 
		static double qx=r.x;
		static double qy=r.y;
		static double qz=r.z;
		double roll = atan2(  2*(qw*qx+qy*qz),  1-2*(qx*qx+qy*qy)  );
		double pitch = asin(2*(qw*qy-qz*qx));              
		double yaw = atan2(  2*(qw*qz+qx*qy),  1-2*(qy*qy+qz*qz)  );

		//Pub velocities
        math::Vector3 angularVelocity = this->model->GetRelativeAngularVel();
        math::Vector3 linearVelocity = this->model->GetRelativeLinearVel();
            
		double linearVel=0;
		if (cos(yaw) == 0) {
			linearVel=linearVelocity.y/sin(yaw);
		}
		else {
			linearVel=linearVelocity.x/cos(yaw);
		}
		//odom2ros.twist.covariance=//covariance 6x6 matrix
		odom2ros.twist.twist.linear.x=linearVel;
		odom2ros.twist.twist.angular.z=angularVelocity.z;
		
		this->pub.publish(odom2ros);
            
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(LunaboticsOdometryPlugin);
}
