#include "AllWheelSteeringPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <stdio.h>
#include <numeric>
#include "std_msgs/Empty.h"

#define Kp	2
#define Ki	0.1
#define Kd	1.5

namespace gazebo
{   
	AllWheelSteeringPlugin::AllWheelSteeringPlugin() {
		std::string name = "gazebo_interface";
	    int argc = 0;
		ros::init(argc, NULL, name);
		this->leftFrontPID = NULL;
		this->rightFrontPID = NULL;
		this->leftRearPID = NULL;
		this->rightRearPID = NULL;
	}
	
	AllWheelSteeringPlugin::~AllWheelSteeringPlugin() {
		delete this->node;
		delete this->leftFrontPID;
		delete this->rightFrontPID;
		delete this->leftRearPID;
		delete this->rightRearPID;
	}
	
	void AllWheelSteeringPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
		ROS_INFO("Loading All Wheel Steering Model Plugin");
		this->model = _parent;
	
		if (this->LoadParams(_sdf)) {
			this->node = new ros::NodeHandle("lunabotics");
	
			this->leftFrontPID = new lunabotics::control::PIDController(Kp, Ki, Kd);
			this->rightFrontPID = new lunabotics::control::PIDController(Kp, Ki, Kd);
			this->leftRearPID = new lunabotics::control::PIDController(Kp, Ki, Kd);
			this->rightRearPID = new lunabotics::control::PIDController(Kp, Ki, Kd);
			
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
			this->sub = this->node->subscribe<lunabotics::AllWheelStateROS>("all_wheel", 100, &AllWheelSteeringPlugin::ROSCallback, this);
			
			if (!this->sub) {
				ROS_ERROR("Could not instantiate subscriber for /lunabotics/all_wheel!");
			}
			
			// ROS Publisher
			this->wheelStatePublisher = this->node->advertise<lunabotics::AllWheelStateROS>("all_wheel_feeback", sizeof(float)*8);
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	void AllWheelSteeringPlugin::ROSCallback(const lunabotics::AllWheelStateROS::ConstPtr& msg) {		
		this->leftFrontSteeringAngle = msg->steering.left_front;
		this->rightFrontSteeringAngle = msg->steering.right_front;
		this->leftRearSteeringAngle = msg->steering.left_rear;
		this->rightRearSteeringAngle = msg->steering.right_rear;
		
		this->leftFrontDrivingSpeed = msg->driving.left_front;
		this->rightFrontDrivingSpeed = msg->driving.right_front;
		this->leftRearDrivingSpeed = msg->driving.left_rear;
		this->rightRearDrivingSpeed = msg->driving.right_rear;
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
	void AllWheelSteeringPlugin::OnUpdate() 
	{		
		double leftFrontSpeedCompensation = 0;
		double rightFrontSpeedCompensation = 0;
		double leftRearSpeedCompensation = 0;
		double rightRearSpeedCompensation = 0;
		
		double actualLeftFrontSteeringAngle = this->leftFrontWheelSteeringJoint->GetAngle(2).Radian();
		double actualRightFrontSteeringAngle = this->rightFrontWheelSteeringJoint->GetAngle(2).Radian();
		double actualLeftRearSteeringAngle = this->leftRearWheelSteeringJoint->GetAngle(2).Radian();
		double actualRightRearSteeringAngle = this->rightRearWheelSteeringJoint->GetAngle(2).Radian();
		
		
		lunabotics::AllWheelStateROS msg;	
		msg.steering.left_front = actualLeftFrontSteeringAngle;
		msg.steering.right_front = actualRightFrontSteeringAngle;
		msg.steering.left_rear = actualLeftRearSteeringAngle;
		msg.steering.right_rear = actualRightRearSteeringAngle;
		msg.driving.left_front = this->leftFrontWheelDrivingJoint->GetVelocity(0);
		msg.driving.right_front = this->rightFrontWheelDrivingJoint->GetVelocity(0);
		msg.driving.left_rear = this->leftRearWheelDrivingJoint->GetVelocity(0);
		msg.driving.right_rear = this->rightRearWheelDrivingJoint->GetVelocity(0);
		this->wheelStatePublisher.publish(msg);
			
		//Left front pid
		double signal = -this->leftFrontPID->control(actualLeftFrontSteeringAngle-this->leftFrontSteeringAngle);
		this->leftFrontWheelSteeringJoint->SetVelocity(0, signal);	
		double compenstation = -this->DrivingFromSteeringVelocity(signal);
		this->leftFrontWheelDrivingJoint->SetVelocity(0, this->leftFrontDrivingSpeed+compenstation);
		
		//Right front PID
		signal = -this->rightFrontPID->control(actualRightFrontSteeringAngle-this->rightFrontSteeringAngle);
		this->rightFrontWheelSteeringJoint->SetVelocity(0, signal);	
		compenstation = this->DrivingFromSteeringVelocity(signal);
		this->rightFrontWheelDrivingJoint->SetVelocity(0, this->rightFrontDrivingSpeed+compenstation);
		
		//Left rear PID
		signal = -this->leftRearPID->control(actualLeftRearSteeringAngle-this->leftRearSteeringAngle);
		this->leftRearWheelSteeringJoint->SetVelocity(0, signal);	
		compenstation = -this->DrivingFromSteeringVelocity(signal);
		this->leftRearWheelDrivingJoint->SetVelocity(0, this->leftRearDrivingSpeed+compenstation);
		
		//Right rear PID
		signal = -this->rightRearPID->control(actualRightRearSteeringAngle-this->rightRearSteeringAngle);
		this->rightRearWheelSteeringJoint->SetVelocity(0, signal);	
		compenstation = this->DrivingFromSteeringVelocity(signal);
		this->rightRearWheelDrivingJoint->SetVelocity(0, this->rightRearDrivingSpeed+compenstation);
		
		ros::spinOnce();
	}
	
	
	double AllWheelSteeringPlugin::DrivingFromSteeringVelocity(double steeringVel)
	{
		return this->linkShoulder/this->wheelRadius * steeringVel;
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(AllWheelSteeringPlugin);
}
