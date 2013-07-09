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
	
		if (this->LoadParams()) {
			this->node = new ros::NodeHandle("lunabotics");
	
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&TransformServerPlugin::OnUpdate, this));
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	bool TransformServerPlugin::LoadParams() {
		bool success = false;
		
		if (this->FindLinkByName(this->lidarLink, "hokuyo::link") &&
		    this->FindJointByName(this->leftFrontJoint, "left_front_steering_hinge") &&
		    this->FindJointByName(this->leftRearJoint, "left_rear_steering_hinge") &&
		    this->FindJointByName(this->rightFrontJoint, "right_front_steering_hinge") &&
		    this->FindJointByName(this->rightRearJoint, "right_rear_steering_hinge") &&
		    this->FindJointByName(this->leftFrontWheelJoint, "left_front_driving_hinge")) {
			
			
			ROS_INFO("============== Getting transforms ======================\n\n");
			
			//math::Vector3 modelPos = this->model->GetWorldPose().pos;
			math::Pose modelPose = this->model->GetRelativePose();
			
			ROS_INFO("--> model pose pos %.3f,%.3f,%.3f", modelPose.pos.x, modelPose.pos.y, modelPose.pos.z);
			ROS_INFO("--> model pose rot %.3f,%.3f,%.3f", modelPose.rot.x, modelPose.rot.y, modelPose.rot.z);
			
			this->model->SetRelativePose(math::Pose(math::Vector3(0,0,0), math::Quaternion(0,0,0,1)));
			
			
			//Left front steering joint
			math::Vector3 leftFrontPos = this->leftFrontJoint->GetAnchor(0);
			
			tf::Transform t;
			t.setRotation(tf::Quaternion(0,0,0,1));
			t.setOrigin(tf::Vector3(leftFrontPos.x, leftFrontPos.y, leftFrontPos.z));
			this->leftFrontT = t;
			
			ROS_INFO("--> left front %.3f,%.3f,%.3f", leftFrontPos.x, leftFrontPos.y, leftFrontPos.z);
			
			//Right front steering joint
			math::Vector3 jointPos = this->rightFrontJoint->GetAnchor(0);
			t.setOrigin(tf::Vector3(jointPos.x, jointPos.y, jointPos.z));
			this->rightFrontT = t;
			
			ROS_INFO("--> right front %.3f,%.3f,%.3f", jointPos.x, jointPos.y, jointPos.z);
			
			//Left rear steering joint
			jointPos = this->leftRearJoint->GetAnchor(0);
			t.setOrigin(tf::Vector3(jointPos.x, jointPos.y, jointPos.z));
			this->leftRearT = t;
			
			ROS_INFO("--> left rear %.3f,%.3f,%.3f", jointPos.x, jointPos.y, jointPos.z);
			
			//Right rear steering joint
			jointPos = this->rightRearJoint->GetAnchor(0);
			t.setOrigin(tf::Vector3(jointPos.x, jointPos.y, jointPos.z));
			this->rightRearT = t;
			
			ROS_INFO("--> right rear %.3f,%.3f,%.3f", jointPos.x, jointPos.y, jointPos.z);
			
			//Left front wheel
			jointPos = this->leftFrontWheelJoint->GetAnchor(0);
			math::Vector3 wheelOffset = jointPos-leftFrontPos;
			t.setOrigin(tf::Vector3(wheelOffset.x, wheelOffset.y, wheelOffset.z));
			this->wheelOffsetT = t;
			ROS_INFO("--> wheel offset %.3f,%.3f,%.3f", wheelOffset.x, fabs(wheelOffset.y), wheelOffset.z);
			
			math::Box bb = this->leftFrontWheelJoint->GetJointLink(0)->GetBoundingBox();
			math::Vector3 size = bb.GetSize();
			t.setOrigin(tf::Vector3(size.GetMax()*0.5,size.GetMin(),0));
			this->wheelRadiusT = t;	
			ROS_INFO("--> wheel radius %.3f,%.3f", size.GetMax()*0.5, size.GetMin());
			
			//Lidar
			math::Pose pose = this->lidarLink->GetRelativePose();
			t.setOrigin(tf::Vector3(-pose.pos.x, -pose.pos.y, -pose.pos.z));
			t.setRotation(tf::Quaternion(-pose.rot.x, -pose.rot.y, -pose.rot.z, pose.rot.w));
			this->lidarT = t;
			
			//Odometry
			t.setOrigin(tf::Vector3(0,0,0));
			t.setRotation(tf::Quaternion(0,0,0,1));
			this->odomT = t;

			this->model->SetRelativePose(modelPose);
					
			success = true;
		}
		return success;
	}
	
	bool TransformServerPlugin::FindLinkByName(physics::LinkPtr &_link, const std::string _name) {
		_link = this->model->GetLink(_name);
		if (!_link) {
			gzerr << "link by name [" << _name << "] not found in model\n";
			return false;
		}
		return true;
	}
	
	bool TransformServerPlugin::FindJointByName(physics::JointPtr &_joint, const std::string _name) {
		_joint = this->model->GetJoint(_name);
		if (!_joint) {
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
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->lidarT, now, "base_link", "base_scan"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->leftFrontT, now, "base_link", "left_front_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->rightFrontT, now, "base_link", "right_front_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->leftRearT, now, "base_link", "left_rear_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->rightRearT, now, "base_link", "right_rear_joint"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->wheelOffsetT, now, "left_front_joint", "left_front_wheel"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->wheelRadiusT, now, "left_front_wheel", "left_front_wheel_radius"));
			this->tfBroadcaster.sendTransform(tf::StampedTransform(this->odomT, now, "odom", "base_link"));
		}
		
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(TransformServerPlugin);
}
