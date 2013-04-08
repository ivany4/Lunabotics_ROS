#include "AllWheelSteeringPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

namespace gazebo
{   
	AllWheelSteeringPlugin::AllWheelSteeringPlugin() {
		std::string name = "gazebo_all_wheel_drive";
	    int argc = 0;
		ros::init(argc, NULL, name);
	}
	
	AllWheelSteeringPlugin::~AllWheelSteeringPlugin() {
		delete this->node;
	}
	
	void AllWheelSteeringPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		this->model = _parent;
	
		if (this->LoadParams(_sdf)) {
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AllWheelSteeringPlugin::OnUpdate, this));
	
			this->node = new ros::NodeHandle("lunabotics");
	
			// ROS Listener
			this->sub = this->node->subscribe<lunabotics::AllWheelSteering>("all_wheel", 100, &AllWheelSteeringPlugin::ROSCallback, this);
			
			if (!this->sub) {
				ROS_ERROR("Could not instantiate subscriber for /lunabotics/all_wheel!");
			}
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	void AllWheelSteeringPlugin::ROSCallback(const lunabotics::AllWheelSteering::ConstPtr& msg) {
		ROS_INFO("============ Got command to adjust wheels =============");
		
		ROS_INFO("Left front wheel %f,%f", msg->left_front_driving_vel, msg->left_front_steering_ang);
		ROS_INFO("Right front wheel %f,%f", msg->right_front_driving_vel, msg->right_front_steering_ang);
		ROS_INFO("Left rear wheel %f,%f", msg->left_rear_driving_vel, msg->left_rear_steering_ang);
		ROS_INFO("Right rear wheel %f,%f", msg->right_rear_driving_vel, msg->right_rear_steering_ang);


		this->leftFrontWheelDrivingJoint->SetVelocity(0, msg->left_front_driving_vel);
		this->rightFrontWheelDrivingJoint->SetVelocity(0, msg->right_front_driving_vel);
		this->leftRearWheelDrivingJoint->SetVelocity(0, msg->left_rear_driving_vel);
		this->rightRearWheelDrivingJoint->SetVelocity(0, msg->right_rear_driving_vel);
		
		this->leftFrontWheelDrivingJoint->SetMaxForce(0, 5.0);
		this->rightFrontWheelDrivingJoint->SetMaxForce(0, 5.0);
		this->leftRearWheelDrivingJoint->SetMaxForce(0, 5.0);
		this->rightRearWheelDrivingJoint->SetMaxForce(0, 5.0);

		this->leftFrontWheelSteeringJoint->SetVelocity(0, msg->left_front_steering_ang*100);
		this->rightFrontWheelSteeringJoint->SetVelocity(0, msg->right_front_steering_ang*100);
		this->leftRearWheelSteeringJoint->SetVelocity(0, msg->left_rear_steering_ang*100);
		this->rightRearWheelSteeringJoint->SetVelocity(0, msg->right_rear_steering_ang*100);
/*
 * ROS_INFO("Got a command for the motors: [%f,%f]", msg->linear.x, msg->angular.z);
		double leftWheelLinearVel = msg->linear.x - msg->angular.z * this->wheelSeparation / 2.0;
		double rightWheelLinearVel = msg->linear.x + msg->angular.z * this->wheelSeparation / 2.0;
				
		double leftWheelAngularVel = leftWheelLinearVel / this->wheelRadius;
		double rightWheelAngularVel = rightWheelLinearVel / this->wheelRadius;
		
		
		ROS_INFO("Got cmd_vel[%f,%f]. Setting motor velocities [%f,%f]", msg->linear.x, msg->angular.z, leftWheelAngularVel, rightWheelAngularVel);
		
		this->leftWheelJoint->SetVelocity(0, leftWheelAngularVel);
		this->rightWheelJoint->SetVelocity(0, rightWheelAngularVel);
		
		this->leftWheelJoint->SetMaxForce(0, this->torque);
		this->rightWheelJoint->SetMaxForce(0, this->torque);
		*/
	}
	
	bool AllWheelSteeringPlugin::LoadParams(sdf::ElementPtr _sdf) {
		bool success = false;
		
		if (this->FindJointByParam(_sdf, this->rightRearWheelSteeringJoint, "right_rear_steering_hinge") && this->FindJointByParam(_sdf, this->leftRearWheelSteeringJoint, "left_rear_steering_hinge") && this->FindJointByParam(_sdf, this->rightFrontWheelSteeringJoint, "right_front_steering_hinge") && this->FindJointByParam(_sdf, this->leftFrontWheelSteeringJoint, "left_front_steering_hinge") && this->FindJointByParam(_sdf, this->rightRearWheelDrivingJoint, "right_rear_driving_hinge") && this->FindJointByParam(_sdf, this->leftRearWheelDrivingJoint, "left_rear_driving_hinge") && this->FindJointByParam(_sdf, this->rightFrontWheelDrivingJoint, "right_front_driving_hinge") && this->FindJointByParam(_sdf, this->leftFrontWheelDrivingJoint, "left_front_driving_hinge")) {
			/*
			if (_sdf->HasElement("torque")) {
				this->torque = _sdf->GetElement("torque")->GetValueDouble();
			}
			else {
				gzwarn << "No torque value set for the DiffDrive plugin. Setting to default value " << DEFAULT_TORQUE << "\n";
				this->torque = DEFAULT_TORQUE;
			}
			
			
			this->wheelSeparation = this->leftWheelJoint->GetAnchor(0).Distance(this->rightWheelJoint->GetAnchor(0));
			ROS_INFO("Wheel separation %f m", this->wheelSeparation);
			
			physics::EntityPtr parent = boost::shared_dynamic_cast<physics::Entity>(this->leftWheelJoint->GetChild());
			
			math::Box bb = parent->GetBoundingBox();
			// This assumes that the largest dimension of the wheel is the diameter
			this->wheelRadius = bb.GetSize().GetMax() * 0.5;
			ROS_INFO("Wheel radius %f m", this->wheelRadius);
			*/
			success = true;
		}
		return success;
	}
	
	bool AllWheelSteeringPlugin::FindJointByParam(sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string _param) {
		if (!_sdf->HasElement(_param)) {
		  gzerr << "param [" << _param << "] not found\n";
		  return false;
		}
		else {
			_joint = this->model->GetJoint(_sdf->GetElement(_param)->GetValueString());
	
			if (!_joint) {
				gzerr << "joint by name [" << _sdf->GetElement(_param)->GetValueString() << "] not found in model\n";
				return false;
			}
		}
		return true;
	}
	
	// Called by the world update start event
	void AllWheelSteeringPlugin::OnUpdate() {
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(AllWheelSteeringPlugin);
}
