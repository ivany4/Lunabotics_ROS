#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

#include "ros/ros.h"

namespace lunabotics {
	namespace control {
		
		
		class PIDController {
			public:
				double Kp;
				double Ki;
				double Kd;
				bool control(double error, double &signal);
				unsigned int integralBufferSize;
				PIDController();
				PIDController(double Kp, double Ki, double Kd);
				~PIDController();
				void setP(double p);
				void setI(double i);
				void setD(double d);
			
			private:
				std::vector<double> integral;
				double prev_error;
				ros::Time prev_time;
		};
		
		typedef PIDController * PIDControllerPtr;
	}
}

#endif
