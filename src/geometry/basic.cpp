#include "basic.h"
using namespace lunabotics;


double lunabotics::normalizedAngle(double angle)
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

double lunabotics::distance(Point p1, Point p2)
{
	return sqrt(pow(p2.x-p1.x, 2)+pow(p2.y-p1.y, 2));
}

Point lunabotics::rotatePoint(Point point, double angle, ROTATION_DIRECTION dir)
{
	return CreatePoint(point.x*cos(angle)+point.y*sin(angle)*dir, -point.x*sin(angle)*dir+point.y*cos(angle));
}

Point lunabotics::midPoint(Point p1, Point p2)
{
	return CreatePoint((p1.x+p2.x)/2, (p1.y+p2.y)/2);
}

double lunabotics::areaOfTriangle(Point A, Point B, Point C)
{
	return fabs((A.x*(B.y-C.y)+B.x*(C.y-A.y)+C.x*(A.y-B.y))/2);
}

bool lunabotics::in_circle(Point p, Point center, double radius)
{
	return distance(p, center) <= radius;
}

bool lunabotics::in_triangle(Point p, double edge1, double edge2, double theta)
{
	bool condition_1 = p.y > 0 && p.y < edge2*sin(theta);
	bool condition_2 = p.y*cos(theta) + p.x*sin(theta) > 0;
	bool condition_3 = p.y*(edge1 + edge2*cos(theta)) + p.x*edge2*sin(theta) - edge1*edge2*sin(theta) < 0;
	return condition_1 && condition_2 && condition_3;
}


//Angle at point1 
double lunabotics::angleFromTriangle(Point point1, Point point2, Point point3)
{
	double edge1 = distance(point1, point3);
	double edge2 = distance(point1, point2);
	double edge3 = distance(point2, point3);
	return angleFromTriangle(edge1, edge2, edge3);
}

//Angle between edge1 and edge2
double lunabotics::angleFromTriangle(double edge1, double edge2, double edge3)
{
	//Cosine rule
	return acos((pow(edge2,2)+pow(edge1,2)-pow(edge3,2))/(2*edge2*edge1));
}
