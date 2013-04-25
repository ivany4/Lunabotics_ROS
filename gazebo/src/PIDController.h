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
				double control(double error);
				unsigned int integralBufferSize;
				PIDController();
				PIDController(double Kp, double Ki, double Kd);
				~PIDController();
			
			private:
				std::vector<double> integral;
				double prev_error;
				ros::Time prev_time;
		};
		
	}
}

#endif
