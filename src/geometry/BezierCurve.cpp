#include "BezierCurve.h"
#include "ros/ros.h"
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
		this->_cached_max_curvature /= 10.0;
	}
	return this->_cached_max_curvature;
}

PointArr BezierCurve::getPoints()
{
	if (this->_cached_points.empty()) {
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

Point BezierCurve::p0()
{
	return this->_p0;
}

Point BezierCurve::p1()
{
	return this->_p1;
}

Point BezierCurve::p2()
{
	return this->_p2;
}

//Non-class methods


inline double alpha_p(double beta, double K_alpha, double K_beta)
{
	return K_alpha/pow(1-sqrt(K_beta/beta), 2);
}

inline double beta_p(double alpha, double K_alpha, double K_beta)
{
	return K_beta/pow(1-sqrt(K_alpha/alpha), 2);
}

BezierCurvePtr lunabotics::CreateConstrainedBezierCurve(Point q0, Point q1, Point q2, Point p, int num_segments)
{		
	//Constant values
	const float ALPHA_STEP = 0.1;
	
	
	//Preserve transformation parameters to reconstruct global coordinates for the curve
	double tx = -q1.x;
	double ty = -q1.y;
	Point w1 = q1;
	
	//Translate to make the origin at q1
	q0.x += tx;
	q0.y += ty;
	q2.x += tx;
	q2.y += ty;
	p.x += tx;
	p.y += ty;
	q1.x = 0;
	q1.y = 0;
	//ROS_INFO("after translation q0=%f,%f, q2=%f,%f, p=%f,%f", q0.x, q0.y, q2.x, q2.y, p.x, p.y);
	
	
	bool flipped = false; //Tells if control points were swapped and bezier curve shoul be flipped
	
	//Rotate to align q1q0 with x-axis
	double theta1 = atan2(q2.y, q2.x);
	double theta2 = atan2(q0.y, -q0.x);
	double residual_angle = M_PI-theta1-theta2;
	double rotate_by = theta1;
	if (residual_angle >= 0 && residual_angle <= M_PI) {
		//Swap points
		Point c = q2;
		q2 = q0;
		q0 = c;
		flipped = true;
	}
	else {
		rotate_by = -(M_PI+theta2);
	}
	
	q0 = rotatePoint(q0, rotate_by, CW);
	q2 = rotatePoint(q2, rotate_by, CW);
	p = rotatePoint(p, rotate_by, CW);
	
	//ROS_INFO("rotate_by=%f, q0=%f,%f, q2=%f,%f, p=%f,%f", rotate_by, q0.x, q0.y, q2.x, q2.y, p.x, p.y);
	
	//Tetragonal concave polygonal data
	double alpha_bar = q0.x;
	double theta = atan2(q2.y, -q2.x);
	double cos_theta = cos(theta);
	double sin_theta = sin(theta);
	double beta_bar = -q2.x/cos_theta;
	
	double alpha = alpha_bar;
	double beta = beta_bar;
	
	if (in_triangle(p, alpha_bar, beta_bar, theta)) {
		
		//Calculate substitutions
		double theta_big = (-cos_theta+sqrt(pow(cos_theta, 2)+8))/2;
		double K_alpha = p.x+p.y*1/tan(theta);
		double K_beta = p.y/sin_theta;
		
		double sqrt_K_alpha = sqrt(K_alpha);
		
		double alpha_m = pow(sqrt_K_alpha+sqrt(K_beta/fabs(cos_theta)), 2);
		double alpha_c = pow(sqrt_K_alpha+sqrt(K_beta/theta_big), 2);
		
		//Apply equation 34 from the paper
		double range_start = std::max(alpha_c, alpha_p(beta_bar, K_alpha, K_beta));
		double range_finish = std::min(alpha_m, alpha_bar);
		double alpha_star = DBL_MAX;
		for (double test_alpha = range_start; test_alpha <= range_finish; test_alpha += ALPHA_STEP) {
			double sqrt_test_alpha = sqrt(test_alpha);
			double new_alpha = pow(pow(sqrt_test_alpha-sqrt_K_alpha, 4)-2*K_beta*cos_theta*pow(sqrt_test_alpha-sqrt_K_alpha, 2)+pow(K_beta, 2), 3/2)/(2*pow(K_beta, 2)*pow(sin_theta, 2)*pow(sqrt_test_alpha-sqrt_K_alpha, 2)*test_alpha);
			if (new_alpha < alpha_star) {
				alpha_star = new_alpha;
				alpha = test_alpha;
			}
		}
		beta = std::min(beta_p(alpha, K_alpha, K_beta), beta_bar);
	}
	
	//Obtain control points
	q0.x = alpha;
	q0.y = 0;
	q2.x = -beta*cos_theta;
	q2.y = beta*sin_theta;
	
	//Transform control points to the global frame
	q1 = w1;
	
	//Rotate
	q0 = rotatePoint(q0, rotate_by, CCW);
	q2 = rotatePoint(q2, rotate_by, CCW);
	
	//Translate
	q0.x -= tx;
	q0.y -= ty;
	q2.x -= tx;
	q2.y -= ty;
	
	if (flipped) {
		Point c = q0;
		q0 = q2;
		q2 = c;
	}
	
	return new BezierCurve(q0, q1, q2, num_segments);
}
