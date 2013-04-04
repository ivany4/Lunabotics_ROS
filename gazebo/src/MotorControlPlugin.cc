#include "MotorControlPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <sensors/sensors.hh>
#include <common/common.hh>
#include <stdio.h>

#define DEFAULT_TORQUE	5.0

namespace gazebo
{   
	ROSMotorControllerPlugin::ROSMotorControllerPlugin() {
		std::string name = "luna_gazebo_diff_drive";
	    int argc = 0;
		ros::init(argc, NULL, name);
	}
	
	ROSMotorControllerPlugin::~ROSMotorControllerPlugin() {
		delete this->node;
	}
	
	void ROSMotorControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		this->model = _parent;
	
		if (this->LoadParams(_sdf)) {
			// testing to see if race condition exists
			//ROS_INFO("Left and right joint angles: %.2f, %.2f", this->leftWheelJoint->GetAngle(0), this->rightWheelJoint->GetAngle(0));
	
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ROSMotorControllerPlugin::OnUpdate, this));
	
			this->node = new ros::NodeHandle("~");
	
			// ROS Listener
			this->sub = this->node->subscribe<geometry_msgs::Twist>("/cmd_vel", 100, &ROSMotorControllerPlugin::ROSCallback, this);
			
			if (!sub) {
				ROS_ERROR("Could not instantiate subscriber for /cmd_vel!");
			}
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	void ROSMotorControllerPlugin::ROSCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		ROS_INFO("Got a command for the motors: [%f,%f]", msg->linear.x, msg->angular.z);
//*
		double leftWheelLinearVel = msg->linear.x - msg->angular.z * this->wheelSeparation / 2.0;
		double rightWheelLinearVel = msg->linear.x + msg->angular.z * this->wheelSeparation / 2.0;
				
		double leftWheelAngularVel = leftWheelLinearVel / this->wheelRadius;
		double rightWheelAngularVel = rightWheelLinearVel / this->wheelRadius;
		
		
		ROS_INFO("Got cmd_vel[%f,%f]. Setting motor velocities [%f,%f]", msg->linear.x, msg->angular.z, leftWheelAngularVel, rightWheelAngularVel);
		
	//	this->leftWheelJoint->SetVelocity(0, leftWheelAngularVel);
	//	this->rightWheelJoint->SetVelocity(0, rightWheelAngularVel);
		this->leftWheelJoint->SetVelocity(0, leftWheelAngularVel);
		this->rightWheelJoint->SetVelocity(0, rightWheelAngularVel);
		
		this->leftWheelJoint->SetMaxForce(0, this->torque);
		this->rightWheelJoint->SetMaxForce(0, this->torque);
		/*/ 
		this->leftWheelJoint->SetForce(0, msg->linear.x);
		this->rightWheelJoint->SetForce(0, msg->angular.z);
		//*/
	}
	
	bool ROSMotorControllerPlugin::LoadParams(sdf::ElementPtr _sdf) {
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
	
	bool ROSMotorControllerPlugin::FindJointByParam(sdf::ElementPtr _sdf, physics::JointPtr &_joint, std::string _param) {
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
	void ROSMotorControllerPlugin::OnUpdate() {
		ros::spinOnce();
	/*
	//ros::spinOnce();
	unsigned int n = this->laser->GetRangeCount();
	double min_dist = 1e6;
	for (unsigned int i = 0; i < n; ++i)
	{
	  if (this->laser->GetRange(i) < min_dist)
	  {
		min_dist = this->laser->GetRange(i);
	  }
	}
	
	double target_dist = 2.0;
	
	if (min_dist < this->laser->GetRangeMax())
	{
	  double torque = .2*this->gain*( min_dist - target_dist );
	  if (torque < -1) torque = -1;
	  else if (torque > 1) torque = 1;
	
	  if (this->leftWheelJoint->GetVelocity(0) > 10 && torque > 0)
	  {
		torque = 0;
	  }
	  else if (this->leftWheelJoint->GetVelocity(0) < -10 && torque < 0)
	  {
		torque = 0;
	  }
	  // Make sure the jerk is not too bad.
	  this->leftWheelJoint->SetForce(0, torque);
	  this->rightWheelJoint->SetForce(0, torque);
	//      ROS_INFO("Min distance: %f. Torque: %f. Velocity %f %f %f.", min_dist, torque, this->leftWheelJoint->GetVelocity(0), this->leftWheelJoint->GetVelocity(1), this->leftWheelJoint->GetVelocity(2));
	}
	*/
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ROSMotorControllerPlugin);
}
