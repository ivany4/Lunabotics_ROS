#include "types.h"

std::string controlModeToString(CTRL_MODE_TYPE type)
{
	switch (type) {
		case ACKERMANN: return "Ackermann";
		case TURN_IN_SPOT: return "'Turn in spot'";
		case LATERAL: return "Lateral";
	}
	return "Undefined";
}

point_t rotate_point(point_t point, double angle, ROTATION_DIRECTION dir)
{
	point_t result;
	result.x = point.x*cos(angle)+point.y*sin(angle)*dir;
	result.y = -point.x*sin(angle)*dir+point.y*cos(angle);
	return result;
}
