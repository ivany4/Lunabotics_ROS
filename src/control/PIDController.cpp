#include "PIDController.h"
#include <numeric>

using namespace lunabotics;

PIDController::PIDController(): Kp(0), Ki(0), Kd(0), integralBufferSize(10), integral(), prev_error(0), prev_time(ros::Time::now()) {
}

PIDController::PIDController(double Kp, double Ki, double Kd): Kp(Kp), Ki(Ki), Kd(Kd), integralBufferSize(10), integral(), prev_error(0), prev_time(ros::Time::now()) {
}

PIDController::~PIDController() {
}

bool PIDController::control(double error, double &signal) {
	ros::Time now = ros::Time::now();		
	ros::Duration duration = now - this->prev_time;
	
	bool result = false;
	
	if (!duration.isZero()) {
		double dt = duration.toSec();
				
		if (this->integral.size() >= this->integralBufferSize) {
			this->integral.erase(this->integral.begin());
		}
		this->integral.push_back(error);
				
		double p_term = this->Kp * error;
		double i_term = this->Ki * accumulate(this->integral.begin(), this->integral.end(), 0);
		double d_term = this->Kd * (error-this->prev_error)/dt;
		
		signal = p_term + i_term + d_term;
		
		//ROS_INFO("P:%.2f I:%.2f D:%.2f ERR:%.2f DT:%.2f PREV_ERR:%.2f SGN:%.2f", this->Kp, this->Ki, this->Kd, error, dt, this->prev_error, signal);
		
		this->prev_error = error;
		this->prev_time = now;
		
		result = !isnan(signal) && !isinf(signal);
	}
	return result;
}
		

void PIDController::setP(double p)
{
	this->Kp = p;
}

void PIDController::setI(double i)
{
	this->Ki = i;
}

void PIDController::setD(double d)
{
	this->Kd = d;
}
