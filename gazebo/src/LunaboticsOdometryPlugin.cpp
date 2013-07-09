#include "LunaboticsOdometryPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>
#include "../../src/topics.h"
#include "GazeboUtils.h"

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
		
		if (findLinkByName(this->model, this->baseLink, "chassis")) {
			
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
	}
	
	
	// Called by the world update start event
	void LunaboticsOdometryPlugin::OnUpdate() {
		ros::Time now = ros::Time::now();
		
		math::Pose currentPose = model->GetWorldPose();
		
		math::Vector3 p = currentPose.pos;
		math::Quaternion r = currentPose.rot;
		
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
		double qw=r.w; 
		double qx=r.x;
		double qy=r.y;
		double qz=r.z;
		double roll = atan2(  2*(qw*qx+qy*qz),  1-2*(qx*qx+qy*qy)  );
		double pitch = asin(2*(qw*qy-qz*qx));              
		double yaw = atan2(  2*(qw*qz+qx*qy),  1-2*(qy*qy+qz*qz)  );


		//Pub velocities
		
		//Goes crazy in Lunarena
        //math::Vector3 angularVelocity = this->model->GetRelativeAngularVel();
        //math::Vector3 linearVelocity = this->model->GetRelativeLinearVel();
		//double xRate = linearVelocity.x;
		//double yRate = linearVelocity.y;
		//double yawRate = angularVelocity.z;
        
		math::Vector3 p2 = this->previousPose.pos;
		math::Quaternion r2 = this->previousPose.rot;

		qw=r2.w; 
		qx=r2.x;
		qy=r2.y;
		qz=r2.z;
		double roll2 = atan2(  2*(qw*qx+qy*qz),  1-2*(qx*qx+qy*qy)  );
		double pitch2 = asin(2*(qw*qy-qz*qx));              
		double yaw2 = atan2(  2*(qw*qz+qx*qy),  1-2*(qy*qy+qz*qz)  );
		
		ros::Duration dt = now-this->previousTime;
		double yawRate = (yaw-yaw2)/dt.toSec();
		double xRate = (p.x-p2.x)/dt.toSec();
		double yRate = (p.y-p2.y)/dt.toSec();
		double motionOrientation = atan2(yRate, xRate);
		double longitudinalAngle = yaw-motionOrientation;
		
		double posRate = sqrt(pow(xRate, 2)+pow(yRate, 2));
		
		double linearVel = cos(longitudinalAngle)*posRate;
		
		//double linearVel = sqrt(pow(linearVelocity.x, 2)+pow(linearVelocity.y, 2));
		
		//odom2ros.twist.covariance=//covariance 6x6 matrix
		odom2ros.twist.twist.linear.x=linearVel;
		odom2ros.twist.twist.angular.z=yawRate;
		
		this->previousPose = currentPose;
		this->previousTime = now;
		
		this->pub.publish(odom2ros);
		
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(LunaboticsOdometryPlugin);
}
