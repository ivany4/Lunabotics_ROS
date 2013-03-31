#include "basic.h"

double geometry::normalizedAngle(double angle)
{
	double res = fmod(angle, M_PI*2);
	if (res < -M_PI) {
		res = M_PI*2-res;
	}
	else if (res > M_PI) {
		res = -(M_PI*2-res);
	}
	return res;
}

double geometry::distanceBetweenPoints(point_t p1, point_t p2)
{
	return sqrt(pow(p2.x-p1.x, 2)+pow(p2.y-p1.y, 2));
}

point_t geometry::rotatePoint(point_t point, double angle, ROTATION_DIRECTION dir)
{
	point_t result;
	result.x = point.x*cos(angle)+point.y*sin(angle)*dir;
	result.y = -point.x*sin(angle)*dir+point.y*cos(angle);
	return result;
}

point_t geometry::midPoint(point_t p1, point_t p2)
{
	point_t p;
	p.x = (p1.x+p2.x)/2;
	p.y = (p1.y+p2.y)/2;
	return p;
}
