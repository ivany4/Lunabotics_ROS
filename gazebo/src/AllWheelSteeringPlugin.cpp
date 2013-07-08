#include "AllWheelSteeringPlugin.h"

#include <boost/bind.hpp>
#include <physics/physics.hh>
#include <stdio.h>
#include <numeric>
#include "std_msgs/Empty.h"
#include "../../src/topics.h"
#include "../../src/types.h"

#define Kp	4
#define Ki	0.1
#define Kd	1.5

#define STEERING_TORQUE	16.5
#define DRIVING_TORQUE 22

#define STEERING_RPM_MAX 40
#define DRIVING_RPM_MAX 60

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
	
			this->leftFrontPID = new lunabotics::PIDController(Kp, Ki, Kd);
			this->rightFrontPID = new lunabotics::PIDController(Kp, Ki, Kd);
			this->leftRearPID = new lunabotics::PIDController(Kp, Ki, Kd);
			this->rightRearPID = new lunabotics::PIDController(Kp, Ki, Kd);
			
			this->leftFrontSteeringJoint->SetMaxForce(0, STEERING_TORQUE);
			this->rightFrontSteeringJoint->SetMaxForce(0, STEERING_TORQUE);
			this->leftRearSteeringJoint->SetMaxForce(0, STEERING_TORQUE);
			this->rightRearSteeringJoint->SetMaxForce(0, STEERING_TORQUE);
			
			this->leftFrontDrivingJoint->SetMaxForce(0, DRIVING_TORQUE);
			this->rightFrontDrivingJoint->SetMaxForce(0, DRIVING_TORQUE);
			this->leftRearDrivingJoint->SetMaxForce(0, DRIVING_TORQUE);
			this->rightRearDrivingJoint->SetMaxForce(0, DRIVING_TORQUE);
		
			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AllWheelSteeringPlugin::OnUpdate, this));
	
			// ROS Listener
			this->sub = this->node->subscribe<lunabotics::AllWheelState>(TOPIC_CMD_EXPLICIT_ALL_WHEEL, 100, &AllWheelSteeringPlugin::ROSCallback, this);
			
			if (!this->sub) {
				ROS_ERROR("Could not instantiate subscriber for %s!", TOPIC_CMD_EXPLICIT_ALL_WHEEL);
			}
			
			// ROS Publisher
			this->wheelStatePublisher = this->node->advertise<lunabotics::AllWheelState>(TOPIC_TM_ALL_WHEEL, sizeof(float)*8);
		}
		else {
			ROS_WARN("Could not load the model!");
		}
	}
	
	void AllWheelSteeringPlugin::ROSCallback(const lunabotics::AllWheelState::ConstPtr& msg) {		
		this->leftFrontSteeringAngle = msg->steering.left_front;
		this->rightFrontSteeringAngle = msg->steering.right_front;
		this->leftRearSteeringAngle = msg->steering.left_rear;
		this->rightRearSteeringAngle = msg->steering.right_rear;
		
		this->leftFrontDrivingSpeed = msg->driving.left_front;
		this->rightFrontDrivingSpeed = msg->driving.right_front;
		this->leftRearDrivingSpeed = msg->driving.left_rear;
		this->rightRearDrivingSpeed = msg->driving.right_rear;
		
		//ROS_INFO("Steering %.2f %.2f %.2f %.2f  Driving %.2f %.2f %.2f %.2f", msg->steering.left_front, msg->steering.right_front, msg->steering.left_rear, msg->steering.right_rear, msg->driving.left_front, msg->driving.right_front, msg->driving.left_rear, msg->driving.right_rear);
	}
	
	bool AllWheelSteeringPlugin::LoadParams(sdf::ElementPtr _sdf) {
		bool success = false;
		
		if (this->FindJointByName(_sdf, this->leftFrontSteeringJoint, "left_front_steering_hinge") &&
		    this->FindJointByName(_sdf, this->rightFrontSteeringJoint, "right_front_steering_hinge") &&
		    this->FindJointByName(_sdf, this->leftRearSteeringJoint, "left_rear_steering_hinge") &&
		    this->FindJointByName(_sdf, this->rightRearSteeringJoint, "right_rear_steering_hinge") &&
		    this->FindJointByName(_sdf, this->leftFrontDrivingJoint, "left_front_driving_hinge") &&
		    this->FindJointByName(_sdf, this->rightFrontDrivingJoint, "right_front_driving_hinge") &&
		    this->FindJointByName(_sdf, this->leftRearDrivingJoint, "left_rear_driving_hinge") &&
		    this->FindJointByName(_sdf, this->rightRearDrivingJoint, "right_rear_driving_hinge")) {
		
			ROS_INFO("============== Getting transforms ======================\n\n");
			
			//Left front wheel
			math::Vector3 steeringPos = this->leftFrontSteeringJoint->GetAnchor(0);
			math::Vector3 drivingPos = this->leftFrontDrivingJoint->GetAnchor(0);
			math::Vector3 wheelOffset = drivingPos-steeringPos;
			this->linkShoulder = wheelOffset.y;
			math::Box bb = this->leftFrontDrivingJoint->GetJointLink(0)->GetBoundingBox();
			this->wheelRadius = bb.GetSize().GetMax() * 0.5;
			
			ROS_INFO("--> wheel radius %.3f", this->wheelRadius);
			ROS_INFO("--> link shoulder %.3f", this->linkShoulder);
			
			success = true;
		}
		return success;
	}
	
	bool AllWheelSteeringPlugin::FindJointByName(sdf::ElementPtr _sdf, physics::JointPtr &_joint, const std::string _name) {
		_joint = this->model->GetJoint(_name);
		if (!_joint) {
			gzerr << "joint by name [" << _name << "] not found in model\n";
			return false;
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
		
		double actualLeftFrontSteeringAngle = this->leftFrontSteeringJoint->GetAngle(2).Radian();
		double actualRightFrontSteeringAngle = this->rightFrontSteeringJoint->GetAngle(2).Radian();
		double actualLeftRearSteeringAngle = this->leftRearSteeringJoint->GetAngle(2).Radian();
		double actualRightRearSteeringAngle = this->rightRearSteeringJoint->GetAngle(2).Radian();
		
		
		lunabotics::AllWheelState msg;	
		msg.steering.left_front = actualLeftFrontSteeringAngle;
		msg.steering.right_front = actualRightFrontSteeringAngle;
		msg.steering.left_rear = actualLeftRearSteeringAngle;
		msg.steering.right_rear = actualRightRearSteeringAngle;
		msg.driving.left_front = this->leftFrontDrivingJoint->GetVelocity(0);
		msg.driving.right_front = this->rightFrontDrivingJoint->GetVelocity(0);
		msg.driving.left_rear = this->leftRearDrivingJoint->GetVelocity(0);
		msg.driving.right_rear = this->rightRearDrivingJoint->GetVelocity(0);
		this->wheelStatePublisher.publish(msg);
			
		double signal;
			
		//Left front pid
		if (this->leftFrontPID->control(actualLeftFrontSteeringAngle-this->leftFrontSteeringAngle, signal)) {
			signal *= -1.0;
			
			if (fabs(signal) > STEERING_RPM_MAX) {
				signal = lunabotics::sign(signal)*STEERING_RPM_MAX;
			}
			
			//Attempt to fake stiffness of the motor to be resistant to external torques
			//this->leftFrontSteeringJoint->SetForce(0, signal);
			this->leftFrontSteeringJoint->SetVelocity(0, signal);	
			double compenstation = -this->DrivingFromSteeringVelocity(signal);
			double drivingVel = this->leftFrontDrivingSpeed+compenstation;
			
			if (fabs(drivingVel) > DRIVING_RPM_MAX) {
				drivingVel = lunabotics::sign(drivingVel)*DRIVING_RPM_MAX;
			}
			
			this->leftFrontDrivingJoint->SetVelocity(0, drivingVel);
		}
		
		//Right front PID
		if (this->rightFrontPID->control(actualRightFrontSteeringAngle-this->rightFrontSteeringAngle, signal)) {
			signal *= -1.0;
			
			if (fabs(signal) > STEERING_RPM_MAX) {
				signal = lunabotics::sign(signal)*STEERING_RPM_MAX;
			}
			
			//this->rightFrontSteeringJoint->SetForce(0, signal);
			this->rightFrontSteeringJoint->SetVelocity(0, signal);	
			double compenstation = this->DrivingFromSteeringVelocity(signal);
			double drivingVel = this->rightFrontDrivingSpeed+compenstation;
			
			if (fabs(drivingVel) > DRIVING_RPM_MAX) {
				drivingVel = lunabotics::sign(drivingVel)*DRIVING_RPM_MAX;
			}
			
			this->rightFrontDrivingJoint->SetVelocity(0, drivingVel);
		}
		
		//Left rear PID
		if (this->leftRearPID->control(actualLeftRearSteeringAngle-this->leftRearSteeringAngle, signal)) {
			signal *= -1.0;
			
			if (fabs(signal) > STEERING_RPM_MAX) {
				signal = lunabotics::sign(signal)*STEERING_RPM_MAX;
			}
			//this->leftRearSteeringJoint->SetForce(0, signal);
			this->leftRearSteeringJoint->SetVelocity(0, signal);	
			double compenstation = -this->DrivingFromSteeringVelocity(signal);
			double drivingVel = this->leftRearDrivingSpeed+compenstation;
			
			if (fabs(drivingVel) > DRIVING_RPM_MAX) {
				drivingVel = lunabotics::sign(drivingVel)*DRIVING_RPM_MAX;
			}
			
			this->leftRearDrivingJoint->SetVelocity(0, drivingVel);
		}
		
		//Right rear PID
		if (this->rightRearPID->control(actualRightRearSteeringAngle-this->rightRearSteeringAngle, signal)) {
			signal *= -1.0;
			
			if (fabs(signal) > STEERING_RPM_MAX) {
				signal = lunabotics::sign(signal)*STEERING_RPM_MAX;
			}
			
			//this->rightRearSteeringJoint->SetForce(0, signal);
			this->rightRearSteeringJoint->SetVelocity(0, signal);	
			double compenstation = this->DrivingFromSteeringVelocity(signal);
			double drivingVel = this->rightRearDrivingSpeed+compenstation;
			
			if (fabs(drivingVel) > DRIVING_RPM_MAX) {
				drivingVel = lunabotics::sign(drivingVel)*DRIVING_RPM_MAX;
			}
			
			this->rightRearDrivingJoint->SetVelocity(0, drivingVel);
		}
		
		ros::spinOnce();
	}
	
	
	double AllWheelSteeringPlugin::DrivingFromSteeringVelocity(double steeringVel)
	{
		return this->linkShoulder/this->wheelRadius * steeringVel;
	}
	
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(AllWheelSteeringPlugin);
}
