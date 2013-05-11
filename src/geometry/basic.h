#ifndef _BASIC_H_
#define _BASIC_H_

#include "../types.h"

namespace lunabotics {
	double distance(Point p1, Point p2);
	double normalizedAngle(double angle);
	Point rotatePoint(Point point, double angle, ROTATION_DIRECTION dir);
	Point midPoint(Point p1, Point p2);
	double areaOfTriangle(Point p0, Point p1, Point p2);
	bool in_circle(Point p, Point center, double radius);
	bool in_triangle(Point p, double edge1, double edge2, double theta);
}

#endif //_BASIC_H_
