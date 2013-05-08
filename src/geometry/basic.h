#ifndef _BASIC_H_
#define _BASIC_H_

#include "../types.h"

namespace lunabotics {
namespace geometry {
	double distanceBetweenPoints(point_t p1, point_t p2);
	double normalizedAngle(double angle);
	point_t rotatePoint(point_t point, double angle, ROTATION_DIRECTION dir);
	point_t midPoint(point_t p1, point_t p2);
}
}

#endif //_BASIC_H_
