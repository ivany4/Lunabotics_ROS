#include "AllWheelPredefinedCmdController.h"
#include "ros/ros.h"
using namespace lunabotics;


#define STEERING_ACCURACY	0.05
#define DRIVING_ACCURACY	0.05

PredefinedCmdController::PredefinedCmdController(): 
final_state(),
has_final_state(false),
_current_cmd(),
state_reached(false),
force_state(false),
_geometry(NULL),
previousState(),
angular_velocity(DEFAULT_WHEEL_VELOCITY),
awaitingState(AwaitingNothing),
needPreviousState(false)
{
}

PredefinedCmdController::~PredefinedCmdController()
{
	delete this->_geometry;
}

bool PredefinedCmdController::needsMoreControl()
{
	return this->force_state || !this->isFinalState();
}

bool PredefinedCmdController::isFinalState()
{
	return this->awaitingState == AwaitingNothing || this->force_state;
}

void PredefinedCmdController::setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd)
{
	this->setNewCommand(cmd, true);
}

void PredefinedCmdController::setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd, bool safeSwitch)
{
	if ((cmd == lunabotics::proto::AllWheelControl::TURN_CCW || cmd == lunabotics::proto::AllWheelControl::TURN_CW) && !this->_geometry) {
		ROS_WARN("Can't perform turning command since robot geometry is undefined");
	}
	else {
		if (cmd != this->_current_cmd || cmd == lunabotics::proto::AllWheelControl::STOP) { //Always allow stop
			this->_current_cmd = cmd;
			if (safeSwitch) {
				this->setAwaitingState(AwaitingStop);
				this->needPreviousState = true;
				this->removeFinalState();
				this->state_reached = false;
				this->force_state = false;
			}
			else {
				this->force_state = true;
			}
		}
	}
}

void PredefinedCmdController::abort()
{
	this->_current_cmd = lunabotics::proto::AllWheelControl::STOP;
	this->setAwaitingState(AwaitingNothing);
}

void PredefinedCmdController::giveFeedback(lunabotics::AllWheelState state)
{
	switch(this->awaitingState) {
		case AwaitingStop: {
			this->state_reached =
				fabs(state.driving.left_front) <= DRIVING_ACCURACY &&
				fabs(state.driving.right_front) <= DRIVING_ACCURACY &&
				fabs(state.driving.left_rear) <= DRIVING_ACCURACY &&
				fabs(state.driving.right_rear) <= DRIVING_ACCURACY;
		}
		break;
		
		case AwaitingSteer: {
			this->state_reached = this->has_final_state &&
				fabs(state.steering.left_front-this->final_state.steering.left_front) <= STEERING_ACCURACY &&
				fabs(state.steering.right_front-this->final_state.steering.right_front) <= STEERING_ACCURACY &&
				fabs(state.steering.left_rear-this->final_state.steering.left_rear) <= STEERING_ACCURACY &&
				fabs(state.steering.right_rear-this->final_state.steering.right_rear) <= STEERING_ACCURACY &&
				fabs(state.driving.left_front) <= DRIVING_ACCURACY &&
				fabs(state.driving.right_front) <= DRIVING_ACCURACY &&
				fabs(state.driving.left_rear) <= DRIVING_ACCURACY &&
				fabs(state.driving.right_rear) <= DRIVING_ACCURACY;
		}
		break;
		
		default: break;
	}
	this->previousState = state;
}

void PredefinedCmdController::setGeometry(AllWheelGeometryPtr geometry)
{
	if (this->_geometry) {
		delete this->_geometry;
	}
	this->_geometry = new AllWheelGeometry(geometry);
}

void PredefinedCmdController::setWheelVelocity(double velocity)
{
	this->angular_velocity = fabs(velocity);
}

bool PredefinedCmdController::control(lunabotics::AllWheelState &signal)
{	
	if (this->force_state) {
		signal = this->stateForCommand(this->_current_cmd);
	}
	else if (this->needsMoreControl()) {
				
		signal = this->standByState();
		lunabotics::AllWheelState desiredState = this->stateForCommand(this->_current_cmd);
		
		switch (this->awaitingState) {
			case AwaitingStop:
				this->copySteeringAngles(this->previousState, signal);
				this->removeFinalState();
			break;
			
			case AwaitingSteer:
				this->copySteeringAngles(desiredState, signal);
				this->setFinalState(signal);
			break;
			
			case AwaitingDrive:
				signal = desiredState;
				this->state_reached = true;
				this->removeFinalState();
			break;
				
			case AwaitingNothing:
				signal = desiredState;
				this->removeFinalState();
			break;
		}
				
		if (this->state_reached) {
			switch(this->awaitingState) {
				case AwaitingStop: this->setAwaitingState(AwaitingSteer); break;
				case AwaitingSteer: this->setAwaitingState(AwaitingDrive); break;
				default: this->setAwaitingState(AwaitingNothing); break;
			}
			this->state_reached = false;
		}
		
		return true;
	}
	return false;
}


void PredefinedCmdController::setAwaitingState(AwaitingState newState)
{
	if (this->awaitingState != newState) {
		this->awaitingState = newState;
		//ROS_INFO("Now waiting %s", this->awaitingState == AwaitingStop ? "until stops" :
		// this->awaitingState == AwaitingSteer ? "until steers" :
		// this->awaitingState == AwaitingDrive ? "until drives" : "nothing");
	}
}

void PredefinedCmdController::setFinalState(lunabotics::AllWheelState finalState)
{
	//ROS_INFO("Setting final state with angle %f", finalState.steering.left_front);
	this->final_state = finalState;
	this->has_final_state = true;
}

void PredefinedCmdController::removeFinalState()
{
	if (this->has_final_state) {
		//ROS_INFO("Removing final state");
		this->has_final_state = false;
	}
}


lunabotics::AllWheelState PredefinedCmdController::stateForCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd)
{
	lunabotics::AllWheelState result = this->standByState();
	switch (cmd) {
		case lunabotics::proto::AllWheelControl::CRAB_LEFT: {
			result.steering.left_front = -M_PI_2;
			result.steering.right_front = M_PI_2;
			result.steering.left_rear = M_PI_2;
			result.steering.right_rear = -M_PI_2;
			result.driving.left_front = -this->angular_velocity;
			result.driving.right_front = this->angular_velocity;
			result.driving.left_rear = this->angular_velocity;
			result.driving.right_rear = -this->angular_velocity;
		}
		break;
		
		case lunabotics::proto::AllWheelControl::CRAB_RIGHT: {
			result.steering.left_front = -M_PI_2;
			result.steering.right_front = M_PI_2;
			result.steering.left_rear = M_PI_2;
			result.steering.right_rear = -M_PI_2;
			result.driving.left_front = this->angular_velocity;
			result.driving.right_front = -this->angular_velocity;
			result.driving.left_rear = -this->angular_velocity;
			result.driving.right_rear = this->angular_velocity;
		}
		break;
		
		case lunabotics::proto::AllWheelControl::TURN_CCW: {
			float lf, rf, lr, rr;
			Point ICR; ICR.x = 0; ICR.y = 0;
			if (this->_geometry->calculateAngles(ICR, lf, rf, lr, rr)) {
				result.steering.left_front = lf;
				result.steering.right_front = rf;
				result.steering.left_rear = lr;
				result.steering.right_rear = rr;
				result.driving.left_front = -this->angular_velocity;
				result.driving.right_front = this->angular_velocity;
				result.driving.left_rear = -this->angular_velocity;
				result.driving.right_rear = this->angular_velocity;
			}
		}
		break;
		
		case lunabotics::proto::AllWheelControl::TURN_CW: {
			float lf, rf, lr, rr;
			Point ICR; ICR.x = 0; ICR.y = 0;
			if (this->_geometry->calculateAngles(ICR, lf, rf, lr, rr)) {
				result.steering.left_front = lf;
				result.steering.right_front = rf;
				result.steering.left_rear = lr;
				result.steering.right_rear = rr;
				result.driving.left_front = this->angular_velocity;
				result.driving.right_front = -this->angular_velocity;
				result.driving.left_rear = this->angular_velocity;
				result.driving.right_rear = -this->angular_velocity;
			}
		}
		break;
		
		case lunabotics::proto::AllWheelControl::DRIVE_FORWARD: {	
			result.driving.left_front = this->angular_velocity;
			result.driving.right_front = this->angular_velocity;
			result.driving.left_rear = this->angular_velocity;
			result.driving.right_rear = this->angular_velocity;
		}
		break;
		
		case lunabotics::proto::AllWheelControl::DRIVE_BACKWARD: {
			result.driving.left_front = -this->angular_velocity;
			result.driving.right_front = -this->angular_velocity;
			result.driving.left_rear = -this->angular_velocity;
			result.driving.right_rear = -this->angular_velocity;
		}
		break;
		
		case lunabotics::proto::AllWheelControl::STOP: {
			//Leave standby
		}
		break;
		
		default:
		break;
	}
	return result;
}

lunabotics::AllWheelState PredefinedCmdController::standByState()
{
	lunabotics::AllWheelState result;
	result.steering.left_front = 0;
	result.steering.right_front = 0;
	result.steering.left_rear = 0;
	result.steering.right_rear = 0;	
	result.driving.left_front = 0;
	result.driving.right_front = 0;
	result.driving.left_rear = 0;
	result.driving.right_rear = 0;
	return result;
}

void PredefinedCmdController::copySteeringAngles(lunabotics::AllWheelState source, lunabotics::AllWheelState &destination)
{
	destination.steering.left_front = source.steering.left_front;
	destination.steering.right_front = source.steering.right_front;
	destination.steering.left_rear = source.steering.left_rear;
	destination.steering.right_rear = source.steering.right_rear;
}

void PredefinedCmdController::copyDrivingVelocities(lunabotics::AllWheelState source, lunabotics::AllWheelState &destination)
{
	destination.driving.left_front = source.driving.left_front;
	destination.driving.right_front = source.driving.right_front;
	destination.driving.left_rear = source.driving.left_rear;
	destination.driving.right_rear = source.driving.right_rear;
}
