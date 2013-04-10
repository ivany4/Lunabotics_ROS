#include "LunaboticsDiffDrivePlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <common/common.hh>
#include <stdio.h>

#define DEFAULT_TORQUE	5.0

namespace gazebo
{   
	LunaboticsDiffDrivePlugin::LunaboticsDiffDrivePlugin() {
		std::string name = "gazebo_diff_drive";
	    int argc = 0;
		ros::init(argc, NULL, name);
	}
	
	LunaboticsDiffDrivePlugin::~LunaboticsDiffDrivePlugin() {
		delete this->node;
	}
	
	void LunaboticsDiffDrivePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		ROS_INFO("Loading Diff Drive Model Plugin");
		this->model = _parent;
	
		if (this->LoadParams(_sdf)) {
			this->leftWheelJoint->SetMaxForce(0, this->torque);
			this->rightWheelJoint->SetMaxForce(0, this->torque);
	
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LunaboticsDiffDrivePlugin::OnUpdate, this));
	
			this->node = new ros::NodeHandle("~");
	
			// ROS Listener
			this->sub = this->node->subscribe<geometry_msgs::Twist>("/cmd_vel", 100, &LunaboticsDiffDrivePlugin::ROSCallback, this);
			
			if (!sub) {
				ROS_ERROR("Could not instantiate subscriber for /cmd_vel!");
			}
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	void LunaboticsDiffDrivePlugin::ROSCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		ROS_INFO("Got a command for the motors: [%f,%f]", msg->linear.x, msg->angular.z);

		double leftWheelLinearVel = msg->linear.x - msg->angular.z * this->wheelSeparation*0.5;
		double rightWheelLinearVel = msg->linear.x + msg->angular.z * this->wheelSeparation*0.5;
				
		double leftWheelAngularVel = leftWheelLinearVel / this->wheelRadius;
		double rightWheelAngularVel = rightWheelLinearVel / this->wheelRadius;
		
		
		ROS_INFO("Got cmd_vel[%f,%f]. Setting motor velocities [%f,%f]", msg->linear.x, msg->angular.z, leftWheelAngularVel, rightWheelAngularVel);
		
		this->leftWheelJoint->SetVelocity(0, leftWheelAngularVel);
		this->rightWheelJoint->SetVelocity(0, rightWheelAngularVel);
		
	}
	
	bool LunaboticsDiffDrivePlugin::LoadParams(sdf::ElementPtr _sdf) {
		bool success = false;
		
		if (this->FindJointByParam(_sdf, this->leftWheelJoint, "left_wheel_hinge") && this->FindJointByParam(_sdf, this->rightWheelJoint, "right_wheel_hinge")) {
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
			
			success = true;
		}
		return success;
	}
	
	bool LunaboticsDiffDrivePlugin::FindJointByParam(sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string _param) {
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
	void LunaboticsDiffDrivePlugin::OnUpdate() {
		ros::spinOnce();
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(LunaboticsDiffDrivePlugin);
}
