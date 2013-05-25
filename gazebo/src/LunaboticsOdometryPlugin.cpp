#include "LunaboticsOdometryPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include "../../src/topics.h"

namespace gazebo
{   
	LunaboticsOdometryPlugin::LunaboticsOdometryPlugin() {
		std::string name = "gazebo_interface";
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
		this->pub = this->node->advertise<nav_msgs::Odometry>(TOPIC_TM_ODOMETRY, 1000);
		
		if (!pub) {
			ROS_ERROR("Could not advertise %s!", TOPIC_TM_ODOMETRY);
		}
	}
	
	
	// Called by the world update start event
	void LunaboticsOdometryPlugin::OnUpdate() {
		ros::Time now = ros::Time::now();
		
		math::Vector3 p = model->GetWorldPose().pos;
		math::Quaternion r = model->GetWorldPose().rot;
	
	
		//Publish the /base_link to /odom transform needed for gmapping
	
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(p.x, p.y, p.z));
		transform.setRotation(tf::Quaternion(r.x, r.y, r.z, r.w));
		this->tfBroadcaster.sendTransform(tf::StampedTransform(transform, now, "base_link", "odom"));
		
            
	
	
	
	
	
	
	
	
	
	
	
	
	
	
		nav_msgs::Odometry odom2ros;
		odom2ros.header.stamp = now;
		odom2ros.header.frame_id = "odom";
		odom2ros.child_frame_id = "base_link";
		odom2ros.pose.pose.position.x = p.x;
		odom2ros.pose.pose.position.y = p.y;
		odom2ros.pose.pose.position.z = p.z;
		odom2ros.pose.pose.orientation.x = r.x;
		odom2ros.pose.pose.orientation.y = r.y;
		odom2ros.pose.pose.orientation.z = r.z;
		odom2ros.pose.pose.orientation.w = r.w;
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
