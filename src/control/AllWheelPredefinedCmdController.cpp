#include "AllWheelPredefinedCmdController.h"
#include "ros/ros.h"

#define STEERING_ACCURACY	0.01

lunabotics::control::PredefinedCmdController::PredefinedCmdController()
{
	this->_cnt = -1;
	this->_expecting_result = false;
	this->_state_updated = true;
	this->_geometry = NULL;
}

lunabotics::control::PredefinedCmdController::~PredefinedCmdController()
{
	delete this->_geometry;
}

bool lunabotics::control::PredefinedCmdController::needsMoreControl()
{
	return this->_state_updated;
}

void lunabotics::control::PredefinedCmdController::setNewCommand(lunabotics::proto::AllWheelControl::PredefinedControlType cmd)
{
	if ((cmd == lunabotics::proto::AllWheelControl::TURN_CCW || cmd == lunabotics::proto::AllWheelControl::TURN_CW) && !this->_geometry) {
		ROS_WARN("Can't perform turning command since robot geometry is undefined");
	}
	else {
		this->_current_cmd = cmd;
		this->_cnt = -1;
		this->_expecting_result = true;
		this->incrementCnt();
	}
}

void lunabotics::control::PredefinedCmdController::incrementCnt()
{
	this->_cnt++;
	if (this->_cnt > 1) {
		this->_cnt = -1;
		this->_state_updated = false;
		ROS_INFO("Sequence completed");
	}
	else {
		this->_state_updated = true;
	}
}

void lunabotics::control::PredefinedCmdController::giveFeedback(lunabotics::AllWheelState state)
{
	if (this->_expecting_result) {
		ROS_INFO("Expecting %.2f, %.2f, %.2f, %.2f", state.steering.left_front, state.steering.right_front, state.steering.left_rear, state.steering.right_rear);
		ROS_INFO("Getting %.2f, %.2f, %.2f, %.2f", this->_expected_state.steering.left_front, this->_expected_state.steering.right_front, this->_expected_state.steering.left_rear, this->_expected_state.steering.right_rear);
		if (fabs(state.steering.left_front-this->_expected_state.steering.left_front) <= STEERING_ACCURACY &&
		fabs(state.steering.right_front-this->_expected_state.steering.right_front) <= STEERING_ACCURACY &&
		fabs(state.steering.left_rear-this->_expected_state.steering.left_rear) <= STEERING_ACCURACY &&
		fabs(state.steering.right_rear-this->_expected_state.steering.right_rear) <= STEERING_ACCURACY) {
			ROS_INFO("Reached correct steering angles");
			this->_expecting_result = false;
			this->incrementCnt();
		}
	}
}

void lunabotics::control::PredefinedCmdController::setGeometry(geometry::AllWheelGeometryPtr geometry)
{
	if (this->_geometry) {
		delete this->_geometry;
	}
	this->_geometry = new geometry::AllWheelGeometry(geometry);
}

bool lunabotics::control::PredefinedCmdController::control(lunabotics::AllWheelState &signal)
{
	if (this->needsMoreControl()) {	
		this->_state_updated = false;
		switch (this->_current_cmd) {
			case lunabotics::proto::AllWheelControl::CRAB_LEFT: {
				signal.steering.left_front = -M_PI_2;
				signal.steering.right_front = M_PI_2;
				signal.steering.left_rear = M_PI_2;
				signal.steering.right_rear = -M_PI_2;
				switch (this->_cnt) {
					case 0: {
						signal.driving.left_front = 0;
						signal.driving.right_front = 0;
						signal.driving.left_rear = 0;
						signal.driving.right_rear = 0;
						this->_expected_state = signal;
					}
					break;
					
					case 1: {
						signal.driving.left_front = -3;
						signal.driving.right_front = 3;
						signal.driving.left_rear = 3;
						signal.driving.right_rear = -3;
						this->_expecting_result = false;
					}
					break;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::CRAB_RIGHT: {
				signal.steering.left_front = -M_PI_2;
				signal.steering.right_front = M_PI_2;
				signal.steering.left_rear = M_PI_2;
				signal.steering.right_rear = -M_PI_2;
				switch (this->_cnt) {
					case 0: {
						signal.driving.left_front = 0;
						signal.driving.right_front = 0;
						signal.driving.left_rear = 0;
						signal.driving.right_rear = 0;
						this->_expected_state = signal;
					}
					break;
					
					case 1: {
						signal.driving.left_front = 3;
						signal.driving.right_front = -3;
						signal.driving.left_rear = -3;
						signal.driving.right_rear = 3;
						this->_expecting_result = false;
					}
					break;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::TURN_CCW: {
				float lf, rf, lr, rr;
				point_t ICR; ICR.x = 0; ICR.y = 0;
				if (this->_geometry->calculateAngles(ICR, lf, rf, lr, rr)) {
					signal.steering.left_front = lf;
					signal.steering.right_front = rf;
					signal.steering.left_rear = lr;
					signal.steering.right_rear = rr;
					switch (this->_cnt) {
						case 0: {
							signal.driving.left_front = 0;
							signal.driving.right_front = 0;
							signal.driving.left_rear = 0;
							signal.driving.right_rear = 0;
							this->_expected_state = signal;
						}
						break;
						
						case 1: {
							signal.driving.left_front = -3;
							signal.driving.right_front = 3;
							signal.driving.left_rear = -3;
							signal.driving.right_rear = 3;
							this->_expecting_result = false;
						}
						break;
					}
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::TURN_CW: {
				float lf, rf, lr, rr;
				point_t ICR; ICR.x = 0; ICR.y = 0;
				if (this->_geometry->calculateAngles(ICR, lf, rf, lr, rr)) {
					signal.steering.left_front = lf;
					signal.steering.right_front = rf;
					signal.steering.left_rear = lr;
					signal.steering.right_rear = rr;
					switch (this->_cnt) {
						case 0: {
							signal.driving.left_front = 0;
							signal.driving.right_front = 0;
							signal.driving.left_rear = 0;
							signal.driving.right_rear = 0;
							this->_expected_state = signal;
						}
						break;
						
						case 1: {
							signal.driving.left_front = 3;
							signal.driving.right_front = -3;
							signal.driving.left_rear = 3;
							signal.driving.right_rear = -3;
							this->_expecting_result = false;
						}
						break;
					}
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::DRIVE_FORWARD: {
				signal.steering.left_front = 0;
				signal.steering.right_front = 0;
				signal.steering.left_rear = 0;
				signal.steering.right_rear = 0;
				switch (this->_cnt) {
					case 0: {
						signal.driving.left_front = 0;
						signal.driving.right_front = 0;
						signal.driving.left_rear = 0;
						signal.driving.right_rear = 0;
						this->_expected_state = signal;
					}
					break;
					
					case 1: {
						signal.driving.left_front = 3;
						signal.driving.right_front = 3;
						signal.driving.left_rear = 3;
						signal.driving.right_rear = 3;
						this->_expecting_result = false;
					}
					break;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::DRIVE_BACKWARD: {
				signal.steering.left_front = 0;
				signal.steering.right_front = 0;
				signal.steering.left_rear = 0;
				signal.steering.right_rear = 0;
				switch (this->_cnt) {
					case 0: {
						signal.driving.left_front = 0;
						signal.driving.right_front = 0;
						signal.driving.left_rear = 0;
						signal.driving.right_rear = 0;
						this->_expected_state = signal;
					}
					break;
					
					case 1: {
						signal.driving.left_front = -3;
						signal.driving.right_front = -3;
						signal.driving.left_rear = -3;
						signal.driving.right_rear = -3;
						this->_expecting_result = false;
					}
					break;
				}
			}
			break;
			
			case lunabotics::proto::AllWheelControl::STOP: {
				signal.driving.left_front = 0;
				signal.driving.right_front = 0;
				signal.driving.left_rear = 0;
				signal.driving.right_rear = 0;
				signal.steering.left_front = 0;
				signal.steering.right_front = 0;
				signal.steering.left_rear = 0;
				signal.steering.right_rear = 0;
				this->_expecting_result = false;
			}
			break;
			
			default:
			break;
		}
		return true;
	}
	return false;
}
