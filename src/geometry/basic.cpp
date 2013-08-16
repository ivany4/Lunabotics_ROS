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

Point lunabotics::rotatePoint(Point point, double angle)
{
	return rotatePoint(point, angle, CCW);
}

Point lunabotics::midPoint(Point p1, Point p2)
{
	return CreatePoint((p1.x+p2.x)/2, (p1.y+p2.y)/2);
}

double lunabotics::areaOfTriangle(Point A, Point B, Point C)
{
	return fabs((A.x*(B.y-C.y)+B.x*(C.y-A.y)+C.x*(A.y-B.y))/2);
}

//Given that the base is p1-p3
double lunabotics::heightOfTriangle(Triangle t, bool allow_inside_only)
{
	double area = areaOfTriangle(t.p1, t.p2, t.p3);
	double base = distance(t.p1, t.p3);
	double height = 2*area/base;
	if (allow_inside_only) {
		Point intersection;
		if (height_base_intersection(t.p2, CreateLine(t.p1, t.p3), height, intersection)) {
			if (!in_triangle(t, intersection)) {
				height = std::min(distance(t.p2, t.p1), distance(t.p2, t.p3));
			}
		}
	}
	return height;
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

bool lunabotics::in_triangle(Triangle t, Point p)
{
	PointArr convex;
	convex.push_back(t.p1);
	convex.push_back(t.p2);
	convex.push_back(t.p3);
	return point_inside_convex(p, convex);
}

void lunabotics::getLineProperties(Line l, double &k, double &b, bool &is_vertical)
{
	if (l.p1.x == l.p2.x) {
		is_vertical = true;
		return;
	}
	else {
		k = (l.p1.y-l.p2.y)/(l.p1.x-l.p2.x);
		b = l.p1.y-k*l.p1.x;
	}
}

bool lunabotics::solve_quadratic_polynomium(double a, double b, double c, double &x1, double &x2)
{
	double D = pow(b, 2)-4*a*c;
	if (D < 0) return false;
	
	x1 = (-b+sqrt(D))/(2*a);
	x2 = (-b-sqrt(D))/(2*a);
	
	return true;
}

bool lunabotics::height_base_intersection(Point peak, Line base, double height, Point &intersection)
{
	double k, bb;
	bool is_vertical;
	getLineProperties(base, k, bb, is_vertical);
	if (is_vertical) {
		intersection = CreatePoint(base.p1.x, peak.y);
		return true;
	}
	
	double a = pow(k, 2)+1;
	double b = 2*(peak.x-k*(peak.y-bb));
	double c = pow((peak.y-bb), 2)-pow(height, 2);
	
	double x1, x2;
	
	if (solve_quadratic_polynomium(a, b, c, x1, x2)) {
		double y1 = k*x1+b;
		double y2 = k*x2+b;
		
		Point p1 = CreatePoint(x1, y1);
		Point p2 = CreatePoint(x2, y2);
		
		double dist1 = distance(p1, base.p1)+distance(p1, base.p2);
		double dist2 = distance(p2, base.p1)+distance(p2, base.p2);
		
		intersection = dist1 < dist2 ? p1 : p2;
		return true;		
	}
	return false;		
}


int lunabotics::point_on_line(Point p, Line l)
{
	//Determinant of a matrix (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax)
	//If det == 0, point lies on the line, else on one side of the line
	return (l.p2.x-l.p1.x)*(p.y-l.p1.y)-(l.p2.y-l.p1.y)*(p.x-l.p1.x);
}

bool lunabotics::point_inside_convex(Point p, PointArr convex)
{
	for (unsigned int i = 1, j = 0; j < convex.size() && i < convex.size(); i++, j++) {
		Point pt1 = convex.at(i);
		Point pt2 = convex.at(j);
		if (((pt1.y <= p.y && p.y < pt2.y) || (pt2.y <= p.y && p.y < pt1.y) ) &&
		    (p.x < (pt2.x-pt1.x)*(p.y-pt1.y)/(pt2.y-pt1.y)+pt1.x)) {
				return true;
		}
	}
	return false;
}



bool lunabotics::in_rectangle(Point p, Rect r)
{
	PointArr convex;
	convex.push_back(r.left_front);
	convex.push_back(r.right_front);
	convex.push_back(r.right_rear);
	convex.push_back(r.left_rear);		
	return point_inside_convex(p, convex);
	/*
	Line top = CreateLine(r.left_front, r.right_front);
	Line bottom = CreateLine(r.left_rear, r.right_rear);
	Line left = CreateLine(r.left_front, r.left_rear);
	Line right = CreateLine(r.right_front, r.right_rear);
	
	if (point_on_line(p, top) <= 0 && //Below top
		point_on_line(p, bottom) >= 0 && //Above bottom
		point_on_line(p, left) <= 0 && //Right from left edge
		point_on_line(p, right) >= 0) { //Left from right edge
		return true;
	}
	return false;*/
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

double lunabotics::intersectionY(double line_x, Line l)
{
    if (l.p2.x == l.p1.x) return DBL_MAX;
    return l.p1.y + (line_x - l.p1.x)*(l.p2.y - l.p1.y)/(l.p2.x - l.p1.x);
}

double lunabotics::intersectionX(double line_y, Line l)
{
    if (l.p2.x == l.p1.x) return DBL_MAX;
    return l.p1.x + (line_y - l.p1.y)*(l.p2.y - l.p1.y)/(l.p2.x - l.p1.x);
}

bool lunabotics::line_crosses_square(Line l, Rect r)
{
	double min_x = std::min(std::min(std::min(r.left_front.x, r.right_front.x), r.left_rear.x), r.right_rear.x);
	double min_y = std::min(std::min(std::min(r.left_front.y, r.right_front.y), r.left_rear.y), r.right_rear.y);
	double max_x = std::max(std::max(std::max(r.left_front.x, r.right_front.x), r.left_rear.x), r.right_rear.x);
	double max_y = std::max(std::max(std::max(r.left_front.y, r.right_front.y), r.left_rear.y), r.right_rear.y);
	
    int intersections = 0;
    if(intersectionX(min_y, l) < max_x && intersectionX(min_y, l) > min_x) return true;
    if(intersectionX(max_y, l) < max_x && intersectionX(max_y, l) > min_x) return true;
    if(intersectionY(min_x, l) < max_y && intersectionY(min_x, l) > min_y) return true;
    if(intersectionY(max_x, l) < max_y && intersectionY(max_x, l) > min_y) return true;
    return intersections;
}

double lunabotics::dot_product(Point A, Point B, Point C)
{
	return dot_product(B-A, C-B);
}
double lunabotics::dot_product(Point A, Point B)
{
	return A.x*B.x+A.y*B.y;
}

double lunabotics::cross_product(Point A, Point B, Point C)
{
	Point AB = B-A;
	Point AC = C-A;
	return AB.x*AC.y-AB.y*AC.x;
}

double lunabotics::linePointDist(Line AB, Point C, bool isSegment)
{
	double dist = cross_product(AB.p1,AB.p2,C) / distance(AB.p1, AB.p2);
	if(isSegment){
		double dot1 = dot_product(AB.p1,AB.p2,C);
		if(dot1 > 0) return distance(AB.p2,C);
		double dot2 = dot_product(AB.p1,AB.p2,C);
		if(dot2 > 0) return distance(AB.p1,C);
	}
	return fabs(dist);
}

Point lunabotics::projection(Point q, Line l)
{
	double L = distance(l.p1, l.p2);
	double r = ((l.p1.y-q.y)*(l.p1.y-l.p2.y)-(l.p1.x-q.x)*(l.p2.x-l.p1.x))/pow(L, 2);
	return CreatePoint(l.p1.x+r*(l.p2.x-l.p1.x), l.p1.y+r*(l.p2.y-l.p1.y));
}
