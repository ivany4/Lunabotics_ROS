#ifndef _UTILS_H_
#define _UTILS_H_
#include <vector>

namespace lunabotics {
	std::vector<double> lowpassFilter(std::vector<double> values, float dt, float smoothing);
}

#endif
