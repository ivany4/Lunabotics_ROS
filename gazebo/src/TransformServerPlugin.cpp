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
		
		if (this->FindLinkByName(_sdf, this->lidarLink, "hokuyo::link") && this->FindLinkByName(_sdf, this->leftFrontConnectorLink, "left_front_connector") && this->FindLinkByName(_sdf, this->rightFrontConnectorLink, "right_front_connector") && this->FindLinkByName(_sdf, this->leftRearConnectorLink, "left_rear_connector") && this->FindLinkByName(_sdf, this->rightRearConnectorLink, "right_rear_connector")) {
			
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
			tf::Transform transform;
			
			math::Pose pose = this->lidarLink->GetRelativePose();
			transform.setOrigin(tf::Vector3(pose.pos.x, pose.pos.y, pose.pos.z));
			transform.setRotation(tf::Quaternion(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(transform, now, "lidar", "base_link"));
			
			pose = this->leftFrontConnectorLink->GetRelativePose();
			transform.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(transform, now, "base_link", "left_front_joint"));
			
			pose = this->rightFrontConnectorLink->GetRelativePose();
			transform.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(transform, now, "base_link", "right_front_joint"));
			
			pose = this->leftRearConnectorLink->GetRelativePose();
			transform.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(transform, now, "base_link", "left_rear_joint"));
			
			pose = this->rightRearConnectorLink->GetRelativePose();
			transform.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(transform, now, "base_link", "right_rear_joint"));
		}
		
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(TransformServerPlugin);
}
