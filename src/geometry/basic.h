#ifndef _BASIC_H_
#define _BASIC_H_

#include "../types.h"

namespace lunabotics {
	double distance(Point p1, Point p2);
	double normalizedAngle(double angle);
	Point rotatePoint(Point point, double angle, ROTATION_DIRECTION dir);
	Point rotatePoint(Point point, double angle);
	Point midPoint(Point p1, Point p2);
	double areaOfTriangle(Point p0, Point p1, Point p2);
	bool in_circle(Point p, Point center, double radius);
	bool in_triangle(Point p, double edge1, double edge2, double theta);
	bool in_rectangle(Point p, Rect r);
	int point_on_line(Point p, Line l);
	bool point_inside_convex(Point p, PointArr convex);
	double angleFromTriangle(Point point1, Point point2, Point point3); //Angle at point1 
	double angleFromTriangle(double edge1, double edge2, double edge3); //Angle between edge1 and edge2
	double intersectionY(double line_x, Line l);
	double intersectionX(double line_y, Line l);
	bool line_crosses_square(Line l, Rect s);
}

#endif //_BASIC_H_
