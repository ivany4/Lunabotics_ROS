#include "BezierCurve.h"
#include "basic.h"

using namespace lunabotics;

BezierCurve::BezierCurve(Point p0, Point p1, Point p2, int num_segments): 
_p0(p0), _p1(p1), _p2(p2), _cached_max_curvature(-1), _num_segments(num_segments)
{
}

BezierCurve::~BezierCurve()
{
}

float BezierCurve::maxCurvature()
{
	if (this->_cached_max_curvature < 0) {
		double A = areaOfTriangle(this->_p0, this->_p1, this->_p2);
		Point m = midPoint(this->_p0, this->_p2);
		double circle_radius = distance(this->_p0, m);
		bool monotone_curvature = in_circle(this->_p1, this->_p0, circle_radius) ||
								  in_circle(this->_p1, this->_p2, circle_radius);
		if (monotone_curvature) {
			this->_cached_max_curvature = pow(distance(this->_p1, m), 3)/pow(A, 2);
		}
		else {
			double k0 = A/pow(distance(this->_p0, this->_p1), 3);
			double k1 = A/pow(distance(this->_p1, this->_p2), 3);
			this->_cached_max_curvature = std::max(k0, k1);
		}		
	}
	return this->_cached_max_curvature;
}

PointArr BezierCurve::getPoints()
{
	if (!this->_cached_points.empty()) {
		for (int i = 0; i <= this->_num_segments; i++) {
			float lambda = i/(float)this->_num_segments;
			double a = pow(1-lambda, 2);
			double b = 2*lambda*(1-lambda);
			double c = pow(lambda, 2);
			this->_cached_points.push_back(CreatePoint(a*this->_p0.x + b*this->_p1.x + c*this->_p2.x, a*this->_p0.y + b*this->_p1.y + c*this->_p2.y));
		}
	}
	return this->_cached_points;
}
