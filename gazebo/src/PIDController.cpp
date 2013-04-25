#include "PIDController.h"
#include <numeric>

lunabotics::control::PIDController::PIDController(): Kp(0), Ki(0), Kd(0), integralBufferSize(10), integral(), prev_error(0), prev_time(ros::Time::now()) {
}

lunabotics::control::PIDController::PIDController(double Kp, double Ki, double Kd): Kp(Kp), Ki(Ki), Kd(Kd), integralBufferSize(10), integral(), prev_error(0), prev_time(ros::Time::now()) {
}

lunabotics::control::PIDController::~PIDController() {
}

double lunabotics::control::PIDController::control(double error) {
	ros::Time now = ros::Time::now();		
	ros::Duration duration = now - this->prev_time;
	uint64_t diff = duration.toNSec();
	
	if (diff == 0) {
		return 0;
	}
	double dt = diff/1000000.0;
			
	if (this->integral.size() == this->integralBufferSize) {
		this->integral.erase(this->integral.begin());
	}
	this->integral.push_back(error);
			
	double p_term = this->Kp * error;
	double i_term = this->Ki * accumulate(this->integral.begin(), this->integral.end(), 0);
	double d_term = this->Kd * (error-this->prev_error)/dt;
	
	double signal = p_term + i_term + d_term;
	
	this->prev_error = error;
	this->prev_time = now;
	
	if (isnan(signal) || isinf(signal)) {
		signal = 0;
	}	
	return signal;
}
		
