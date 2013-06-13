#include "utils.h"
#include "ros/ros.h"

using namespace lunabotics;

std::vector<double> lowpassFilter(std::vector<double> values, float dt, float smoothing)
{
	std::vector<double> output;
	if (values.size() > 0) {
		double alpha = dt/(smoothing+dt);
		output.push_back(values.at(0));
		for (int i = 1; i < values.size(); i++) {
			double val = alpha * values.at(i) + (1-alpha) * output.at(i-1);
			output.push_back(val);
		}
	}
	return output;
}
