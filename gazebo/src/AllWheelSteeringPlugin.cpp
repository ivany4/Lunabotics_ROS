#include "AllWheelSteeringPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <stdio.h>
#include <numeric>

#define Kp	2
#define Ki	0.01
#define Kd	1

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
			this->node = new ros::NodeHandle("lunabotics");
	
			common::Time now = common::Time::GetWallTime();
			this->leftFrontPID.prev_time = now;
			this->rightFrontPID.prev_time = now;
			this->leftRearPID.prev_time = now;
			this->rightRearPID.prev_time = now;
			
			this->leftFrontWheelSteeringJoint->SetMaxForce(0, 5.0);
			this->rightFrontWheelSteeringJoint->SetMaxForce(0, 5.0);
			this->leftRearWheelSteeringJoint->SetMaxForce(0, 5.0);
			this->rightRearWheelSteeringJoint->SetMaxForce(0, 5.0);
			
			this->leftFrontWheelDrivingJoint->SetMaxForce(0, 5.0);
			this->rightFrontWheelDrivingJoint->SetMaxForce(0, 5.0);
			this->leftRearWheelDrivingJoint->SetMaxForce(0, 5.0);
			this->rightRearWheelDrivingJoint->SetMaxForce(0, 5.0);
		
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AllWheelSteeringPlugin::OnUpdate, this));
	
			// ROS Listener
			this->sub = this->node->subscribe<lunabotics::AllWheelSteering>("all_wheel", 100, &AllWheelSteeringPlugin::ROSCallback, this);
			
			if (!this->sub) {
				ROS_ERROR("Could not instantiate subscriber for /lunabotics/all_wheel!");
			}
#include <numeric>
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	void AllWheelSteeringPlugin::ROSCallback(const lunabotics::AllWheelSteering::ConstPtr& msg) {
		ROS_INFO("============ Got command to adjust wheels =============");
		/*
		ROS_INFO("Left front wheel %f,%f", msg->left_front_driving_vel, msg->left_front_steering_ang);
		ROS_INFO("Right front wheel %f,%f", msg->right_front_driving_vel, msg->right_front_steering_ang);
		ROS_INFO("Left rear wheel %f,%f", msg->left_rear_driving_vel, msg->left_rear_steering_ang);
		ROS_INFO("Right rear wheel %f,%f", msg->right_rear_driving_vel, msg->right_rear_steering_ang);
		*/
		
		this->leftFrontSteeringAngle = msg->left_front_steering_ang;
		this->rightFrontSteeringAngle = msg->right_front_steering_ang;
		this->leftRearSteeringAngle = msg->left_rear_steering_ang;
		this->rightRearSteeringAngle = msg->right_rear_steering_ang;
		
		this->leftFrontDrivingSpeed = msg->left_front_driving_vel;
		this->rightFrontDrivingSpeed = msg->right_front_driving_vel;
		this->leftRearDrivingSpeed = msg->left_rear_driving_vel;
		this->rightRearDrivingSpeed = msg->right_rear_driving_vel;
	}
	
	bool AllWheelSteeringPlugin::LoadParams(sdf::ElementPtr _sdf) {
		bool success = false;
		
		if (this->FindJointByParam(_sdf, this->rightRearWheelSteeringJoint, "right_rear_steering_hinge") && this->FindJointByParam(_sdf, this->leftRearWheelSteeringJoint, "left_rear_steering_hinge") && this->FindJointByParam(_sdf, this->rightFrontWheelSteeringJoint, "right_front_steering_hinge") && this->FindJointByParam(_sdf, this->leftFrontWheelSteeringJoint, "left_front_steering_hinge") && this->FindJointByParam(_sdf, this->rightRearWheelDrivingJoint, "right_rear_driving_hinge") && this->FindJointByParam(_sdf, this->leftRearWheelDrivingJoint, "left_rear_driving_hinge") && this->FindJointByParam(_sdf, this->rightFrontWheelDrivingJoint, "right_front_driving_hinge") && this->FindJointByParam(_sdf, this->leftFrontWheelDrivingJoint, "left_front_driving_hinge")) {
			
			physics::EntityPtr wheel = boost::shared_dynamic_cast<physics::Entity>(this->leftRearWheelDrivingJoint->GetChild());
			math::Box bb = wheel->GetBoundingBox();
			// This assumes that the largest dimension of the wheel is the diameter
			this->wheelRadius = bb.GetSize().GetMax() * 0.5;
			ROS_INFO("Wheel radius %f m", this->wheelRadius);
			
			//Shoulder is distance between two joints
			math::Vector3 drivingJointAnchor = this->leftRearWheelDrivingJoint->GetAnchor(2);
			physics::EntityPtr link = boost::shared_dynamic_cast<physics::Entity>(this->rightFrontWheelSteeringJoint->GetChild());
			bb = link->GetBoundingBox();
			//Assume that shoulder is longes edge minus offset of steering joint
			//////////////////////////////////////////////////////
			//TODO: Needs to be done nicely
			/////////////////////////////////////////////////////
			this->linkShoulder = bb.GetSize().GetMax();//-drivingJointAnchor.GetLength();
			
			ROS_INFO("Shoulder %f m", this->linkShoulder);
			
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
		
		
		////DEBUG
		
		//ros::spinOnce();
		//return;
		/////////////
		
		double leftFrontSpeedCompensation = 0;
		double rightFrontSpeedCompensation = 0;
		double leftRearSpeedCompensation = 0;
		double rightRearSpeedCompensation = 0;
		
		double actualLeftFrontSteeringAngle = this->leftFrontWheelSteeringJoint->GetAngle(2).Radian();
		double actualRightFrontSteeringAngle = this->rightFrontWheelSteeringJoint->GetAngle(2).Radian();
		double actualLeftRearSteeringAngle = this->leftRearWheelSteeringJoint->GetAngle(2).Radian();
		double actualRightRearSteeringAngle = this->rightRearWheelSteeringJoint->GetAngle(2).Radian();
		
		if (actualLeftFrontSteeringAngle != this->leftFrontSteeringAngle) {
			//PID here
			double err = actualLeftFrontSteeringAngle-this->leftFrontSteeringAngle;
			double signal = this->CalculatePID(err, this->leftFrontPID);
			this->leftFrontWheelSteeringJoint->SetVelocity(0, signal);	
			leftFrontSpeedCompensation = -this->DrivingFromSteeringVelocity(signal);		
		}
		
		if (actualRightFrontSteeringAngle != this->rightFrontSteeringAngle) {
			//PID here
			double err = actualRightFrontSteeringAngle-this->rightFrontSteeringAngle;
			double signal = this->CalculatePID(err, this->rightFrontPID);
			this->rightFrontWheelSteeringJoint->SetVelocity(0, signal);
			rightFrontSpeedCompensation = this->DrivingFromSteeringVelocity(signal);
		//	ROS_INFO("From: %f To: %f Err: %f Sig: %f Prev_err: %f Drive: %f", actualRightFrontSteeringAngle, this->rightFrontSteeringAngle, err, signal, this->rightFrontPID.prev_err, rightFrontSpeedCompensation);
		}
		
		if (actualLeftRearSteeringAngle != this->leftRearSteeringAngle) {
			//PID here
			double err = actualLeftRearSteeringAngle-this->leftRearSteeringAngle;
			double signal = this->CalculatePID(err, this->leftRearPID);
			this->leftRearWheelSteeringJoint->SetVelocity(0, signal);
			leftRearSpeedCompensation = -this->DrivingFromSteeringVelocity(signal);
		}
		
		if (actualRightRearSteeringAngle != this->rightRearSteeringAngle) {
			//PID here
			double err = actualRightRearSteeringAngle-this->rightRearSteeringAngle;
			double signal = this->CalculatePID(err, this->rightRearPID);
			this->rightRearWheelSteeringJoint->SetVelocity(0, signal);
			rightRearSpeedCompensation = this->DrivingFromSteeringVelocity(signal);
		}
		
		
		this->leftFrontWheelDrivingJoint->SetVelocity(0, this->leftFrontDrivingSpeed+leftFrontSpeedCompensation);
		this->rightFrontWheelDrivingJoint->SetVelocity(0, this->rightFrontDrivingSpeed+rightFrontSpeedCompensation);
		this->leftRearWheelDrivingJoint->SetVelocity(0, this->leftRearDrivingSpeed+leftRearSpeedCompensation);
		this->rightRearWheelDrivingJoint->SetVelocity(0, this->rightRearDrivingSpeed+rightRearSpeedCompensation);
		
		
		ros::spinOnce();
	}
	
	double AllWheelSteeringPlugin::CalculatePID(double err, PIDData &data)
	{
		common::Time now = common::Time::GetWallTime();		
		common::Time duration = now - data.prev_time;
		double dt = duration.Double();
		
		if (dt == 0) {
			return 0;
		}
				
		if (data.integral.size() == 10) {
			data.integral.erase(data.integral.begin());
		}
		data.integral.push_back(err);
				
		double p_term = err;
		double i_term = accumulate(data.integral.begin(), data.integral.end(), 0);
		double d_term = (err - data.prev_err) / dt;
		
		double signal = Kp*p_term + Ki*i_term + Kd*d_term;
		
		data.prev_err = err;
		data.prev_time = now;
		
		if (isnan(signal) || isinf(signal)) {
			signal = 0;
		}
		else {
			signal *= -1;
		}
		
		return signal;
		
	}
	
	double AllWheelSteeringPlugin::DrivingFromSteeringVelocity(double steeringVel)
	{
		return this->linkShoulder/this->wheelRadius * steeringVel;
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(AllWheelSteeringPlugin);
}
