#include "AllWheelPredefinedCmdController.h"
#include "ros/ros.h"
using namespace lunabotics;


#define STEERING_ACCURACY	0.05
#define DRIVING_ACCURACY	0.05

PredefinedCmdController::PredefinedCmdController(): 
final_state(),
has_final_state(false),
_current_cmd(),
is_final_state(true),
state_reached(false),
_geometry(NULL),
previousState(),
angular_velocity(DEFAULT_WHEEL_VELOCITY),
awaitingState(AwaitingNone),
needPreviousState(false)
{
}

PredefinedCmdController::~PredefinedCmdController()
{
	delete this->_geometry;
}

bool PredefinedCmdController::needsMoreControl()
{
	return !this->is_final_state;
}

void PredefinedCmdController::setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd)
{
	if ((cmd == lunabotics::proto::AllWheelControl::TURN_CCW || cmd == lunabotics::proto::AllWheelControl::TURN_CW) && !this->_geometry) {
		ROS_WARN("Can't perform turning command since robot geometry is undefined");
	}
	else {
		if (cmd != this->_current_cmd || cmd == lunabotics::proto::AllWheelControl::STOP) { //Always allow stop
			this->_current_cmd = cmd;
			this->setAwaitingState(AwaitingDriving);
			this->needPreviousState = true;
			this->is_final_state = false;
			this->removeFinalState();
			this->state_reached = false;
		}
	}
}

void PredefinedCmdController::giveFeedback(lunabotics::AllWheelState state)
{
	switch(this->awaitingState) {
		case AwaitingDriving: {
			this->state_reached =
				fabs(state.driving.left_front) <= DRIVING_ACCURACY &&
				fabs(state.driving.right_front) <= DRIVING_ACCURACY &&
				fabs(state.driving.left_rear) <= DRIVING_ACCURACY &&
				fabs(state.driving.right_rear) <= DRIVING_ACCURACY;
		}
		break;
		
		case AwaitingSteering: {
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
		
		case AwaitingNone: {
		}
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
	if (this->needsMoreControl()) {
				
		if (this->awaitingState == AwaitingDriving || this->awaitingState == AwaitingSteering) {
			signal.driving.left_front = 0;
			signal.driving.right_front = 0;
			signal.driving.left_rear = 0;
			signal.driving.right_rear = 0;
			if (this->awaitingState == AwaitingDriving) {
				signal.steering.left_front = this->previousState.steering.left_front;
				signal.steering.right_front = this->previousState.steering.right_front;
				signal.steering.left_rear = this->previousState.steering.left_rear;
				signal.steering.right_rear = this->previousState.steering.right_rear;
			}
		}
		
		switch (this->_current_cmd) {
			case lunabotics::proto::AllWheelControl::CRAB_LEFT: {
				if (this->awaitingState != AwaitingDriving) {
					signal.steering.left_front = -M_PI_2;
					signal.steering.right_front = M_PI_2;
					signal.steering.left_rear = M_PI_2;
					signal.steering.right_rear = -M_PI_2;
				}
				if (this->awaitingState == AwaitingNone) {
					signal.driving.left_front = -this->angular_velocity;
					signal.driving.right_front = this->angular_velocity;
					signal.driving.left_rear = this->angular_velocity;
					signal.driving.right_rear = -this->angular_velocity;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::CRAB_RIGHT: {
				if (this->awaitingState != AwaitingDriving) {
					signal.steering.left_front = -M_PI_2;
					signal.steering.right_front = M_PI_2;
					signal.steering.left_rear = M_PI_2;
					signal.steering.right_rear = -M_PI_2;
				}
				if (this->awaitingState == AwaitingNone) {
					signal.driving.left_front = this->angular_velocity;
					signal.driving.right_front = -this->angular_velocity;
					signal.driving.left_rear = -this->angular_velocity;
					signal.driving.right_rear = this->angular_velocity;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::TURN_CCW: {
				float lf, rf, lr, rr;
				Point ICR; ICR.x = 0; ICR.y = 0;
				if (this->_geometry->calculateAngles(ICR, lf, rf, lr, rr)) {
					
					if (this->awaitingState != AwaitingDriving) {
						signal.steering.left_front = lf;
						signal.steering.right_front = rf;
						signal.steering.left_rear = lr;
						signal.steering.right_rear = rr;
					}
					if (this->awaitingState == AwaitingNone) {
						signal.driving.left_front = -this->angular_velocity;
						signal.driving.right_front = this->angular_velocity;
						signal.driving.left_rear = -this->angular_velocity;
						signal.driving.right_rear = this->angular_velocity;
					}
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::TURN_CW: {
				float lf, rf, lr, rr;
				Point ICR; ICR.x = 0; ICR.y = 0;
				if (this->_geometry->calculateAngles(ICR, lf, rf, lr, rr)) {
					if (this->awaitingState != AwaitingDriving) {
						signal.steering.left_front = lf;
						signal.steering.right_front = rf;
						signal.steering.left_rear = lr;
						signal.steering.right_rear = rr;
					}
					if (this->awaitingState == AwaitingNone) {
						signal.driving.left_front = this->angular_velocity;
						signal.driving.right_front = -this->angular_velocity;
						signal.driving.left_rear = this->angular_velocity;
						signal.driving.right_rear = -this->angular_velocity;
					}
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::DRIVE_FORWARD: {
				if (this->awaitingState != AwaitingDriving) {
					signal.steering.left_front = 0;
					signal.steering.right_front = 0;
					signal.steering.left_rear = 0;
					signal.steering.right_rear = 0;
				}
				if (this->awaitingState == AwaitingNone) {
					signal.driving.left_front = this->angular_velocity;
					signal.driving.right_front = this->angular_velocity;
					signal.driving.left_rear = this->angular_velocity;
					signal.driving.right_rear = this->angular_velocity;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::DRIVE_BACKWARD: {
				if (this->awaitingState != AwaitingDriving) {
					signal.steering.left_front = 0;
					signal.steering.right_front = 0;
					signal.steering.left_rear = 0;
					signal.steering.right_rear = 0;
				}
				if (this->awaitingState == AwaitingNone) {
					signal.driving.left_front = -this->angular_velocity;
					signal.driving.right_front = -this->angular_velocity;
					signal.driving.left_rear = -this->angular_velocity;
					signal.driving.right_rear = -this->angular_velocity;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::STOP: {
				if (this->awaitingState != AwaitingDriving) {
					signal.steering.left_front = 0;
					signal.steering.right_front = 0;
					signal.steering.left_rear = 0;
					signal.steering.right_rear = 0;
				}
				if (this->awaitingState == AwaitingNone) {
					signal.driving.left_front = 0;
					signal.driving.right_front = 0;
					signal.driving.left_rear = 0;
					signal.driving.right_rear = 0;
				}
			}
			break;
			
			default:
			break;
		}
		
		if (this->awaitingState == AwaitingDriving) {
			this->removeFinalState();
		}
		else {
			this->setFinalState(signal);
		}
		
		this->is_final_state = this->awaitingState == AwaitingNone;
		
		if (this->state_reached) {
			switch(this->awaitingState) {
				case AwaitingDriving: this->setAwaitingState(AwaitingSteering); break;
				case AwaitingSteering: this->setAwaitingState(AwaitingNone); break;
				default: this->setAwaitingState(AwaitingNone); break;
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
		//ROS_INFO("Now awaiting %s", this->awaitingState == AwaitingDriving ?
		//		"driving" : this->awaitingState == AwaitingSteering ? "steering" : "nothing");
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
