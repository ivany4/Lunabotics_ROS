#include "TransformServerPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <stdio.h>
#include <numeric>

namespace gazebo
{   
	TransformServerPlugin::TransformServerPlugin() {
		std::string name = "gazebo_interface";
	    int argc = 0;
		ros::init(argc, NULL, name);
	}
	
	TransformServerPlugin::~TransformServerPlugin() {
		delete this->node;
	}
	
	void TransformServerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		ROS_INFO("Loading Transform Server Plugin");
		this->model = _parent;
	
		if (this->LoadParams(_sdf)) {
			this->node = new ros::NodeHandle("lunabotics");
	
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TransformServerPlugin::OnUpdate, this));
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	bool TransformServerPlugin::LoadParams(sdf::ElementPtr _sdf) {
		bool success = false;
		
		if (this->FindLinkByName(_sdf, this->lidarLink, "hokuyo::link") &&
		    this->FindLinkByName(_sdf, this->leftFrontConnectorLink, "left_front_connector") &&
		    this->FindLinkByName(_sdf, this->rightFrontConnectorLink, "right_front_connector") &&
		    this->FindLinkByName(_sdf, this->leftRearConnectorLink, "left_rear_connector") &&
		    this->FindLinkByName(_sdf, this->rightRearConnectorLink, "right_rear_connector") &&
		    this->FindLinkByName(_sdf, this->leftFrontWheelLink, "left_front_wheel")) {
			
			tf::Transform t;
			math::Pose pose = this->lidarLink->GetRelativePose();
			t.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
			t.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));
			this->lidarT = t;
			
			tf::Transform t2;
			t2.setRotation(tf::Quaternion(0,0,0,1));
			pose = this->rightFrontConnectorLink->GetRelativePose();
			t2.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->rightFrontT = t2;
			
			pose = this->leftRearConnectorLink->GetRelativePose();
			t2.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->leftRearT = t2;
			
			pose = this->rightRearConnectorLink->GetRelativePose();
			t2.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->rightRearT = t2;
			
			pose = this->leftFrontConnectorLink->GetRelativePose();
			t2.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->leftFrontT = t2;
			
			math::Pose wheelPose = this->leftFrontWheelLink->GetRelativePose();
			t2.setOrigin(tf::Vector3(0, sqrt(pow(pose.pos.x-wheelPose.pos.x,2)+pow(pose.pos.y-wheelPose.pos.y,2)), 0));
			this->wheelOffsetT = t2;
			
			math::Box bb = this->leftFrontWheelLink->GetBoundingBox();
			t2.setOrigin(tf::Vector3(bb.GetSize().GetMax()*0.5,0,0));
			this->wheelRadiusT = t2;		
			
			success = true;
		}
		return success;
	}
	
	bool TransformServerPlugin::FindLinkByName(sdf::ElementPtr _sdf, physics::LinkPtr &_link, const std::string _name) {
		_link = this->model->GetLink(_name);
		if (!_link) {
			gzerr << "joint by name [" << _name << "] not found in model\n";
			return false;
		}
		return true;
	}
	
	// Called by the world update start event
	void TransformServerPlugin::OnUpdate() 
	{		
		if (this->node->ok()) {
			ros::Time now = ros::Time::now();
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->lidarT, now, "lidar", "base_link"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->leftFrontT, now, "base_link", "left_front_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->rightFrontT, now, "base_link", "right_front_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->leftRearT, now, "base_link", "left_rear_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->rightRearT, now, "base_link", "right_rear_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->wheelOffsetT, now, "left_front_joint", "left_front_wheel"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->wheelRadiusT, now, "left_front_wheel", "left_front_wheel_radius"));
		}
		
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(TransformServerPlugin);
}
