#include "bezier_smooth.h"
#include "ros/ros.h"
#include "float.h"
using namespace std;

double alpha_p(double beta, double K_alpha, double K_beta)
{
	return K_alpha/pow(1-sqrt(K_beta/beta), 2);
}

double beta_p(double alpha, double K_alpha, double K_beta)
{
	return K_beta/pow(1-sqrt(K_alpha/alpha), 2);
}

bool point_is_within_triangle(geometry_msgs::Point p, double alpha_bar, double beta_bar, double theta)
{
	bool condition_1 = p.y > 0 && p.y < beta_bar*sin(theta);
	bool condition_2 = p.y*cos(theta) + p.x*sin(theta) > 0;
	bool condition_3 = p.y*(alpha_bar + beta_bar*cos(theta)) + p.x*beta_bar*sin(theta) - alpha_bar*beta_bar*sin(theta) < 0;
	return condition_1 && condition_2 && condition_3;
}

geometry_msgs::Point rotate_point(geometry_msgs::Point point, double angle)
{
	point.x = point.x*cos(angle)+point.y*sin(angle);
	point.y = -point.x*sin(angle)+point.y*cos(angle);
	return point;
}

std::vector<geometry_msgs::Point> planning::quadratic_bezier(geometry_msgs::Point q0, geometry_msgs::Point q1, geometry_msgs::Point q2, float step)
{
	std::vector<geometry_msgs::Point> points;
	for (float lambda = 0; lambda <= 1; lambda += step) {
		geometry_msgs::Point point;
		double a = pow(1-lambda, 2);
		double b = 2*lambda*(1-lambda);
		double c = pow(lambda, 2);
		point.x = a*q0.x + b*q1.x + c*q2.x;
		point.y = a*q0.y + b*q1.y + c*q2.y;
		points.push_back(point);
	}
	return points;
}

std::vector<geometry_msgs::Point> planning::trajectory_bezier(geometry_msgs::Point q0, geometry_msgs::Point q1, geometry_msgs::Point q2,  geometry_msgs::Point p)
{
	//Constant values
	const float ALPHA_STEP = 0.1;
	const float BEZIER_STEP = 0.01;
	
	
	//Preserve transformation parameters to reconstruct global coordinates for the curve
	double tx = -q1.x;
	double ty = -q1.y;
	geometry_msgs::Point w1 = q1;
	
	//Translate to make the origin at q1
	q0.x += tx;
	q0.y += ty;
	q2.x += tx;
	q2.y += ty;
	p.x += tx;
	p.y += ty;
	q1.x = 0;
	q1.y = 0;
	
	//Rotate to align q1q0 with x-axis
	double theta1 = atan2(q2.y, q2.x);
	double theta2 = atan2(q0.y, -q0.x);
	double residual_angle = M_PI-theta1-theta2;
	double rotate_by = theta1;
	if (residual_angle >= 0 && residual_angle <= M_PI) {
		//Swap points
		geometry_msgs::Point c = q2;
		q2 = q0;
		q0 = c;
	}
	else {
		rotate_by = -(M_PI+theta2);
	}
	
	q0 = rotate_point(q0, rotate_by);
	q2 = rotate_point(q2, rotate_by);
	p = rotate_point(p, rotate_by);
	
	//Tetragonal concave polygonal data
	double alpha_bar = q0.x;
	double theta = atan2(q2.y, -q2.x);
	double cos_theta = cos(theta);
	double sin_theta = sin(theta);
	double beta_bar = -q2.x/cos_theta;
	
	double alpha = alpha_bar;
	double beta = beta_bar;
	
	if (point_is_within_triangle(p, alpha_bar, beta_bar, theta)) {
		
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
	q0 = rotate_point(q0, -rotate_by);
	q2 = rotate_point(q2, -rotate_by);
	
	//Translate
	q0.x -= tx;
	q0.y -= ty;
	q2.x -= tx;
	q2.y -= ty;
	
	return planning::quadratic_bezier(q0, q1, q2, BEZIER_STEP);
}



	void planning::GetCurveControlPoints(point_arr knots, point_arr &firstControlPoints, point_arr &secondControlPoints)
	{
		unsigned int n = knots.size();
		if (n < 2) {
			ROS_WARN("At least two knot points required");
		}
		else if (n == 2) {
			ROS_INFO("Straight line");
			 // Special case: Bezier curve should be a straight line.
			// 3P1 = 2P0 + P3
			geometry_msgs::Point P0 = knots.at(0);
			geometry_msgs::Point P3 = knots.at(1);
			geometry_msgs::Point P1, P2;
			P1.x = (2*P0.x + P3.x) / 3.0;
			P1.y = (2*P0.y + P3.y) / 3.0;
			firstControlPoints.push_back(P1);

			// P2 = 2P1 – P0
			P2.x = 2*P1.x - P0.x;
			P2.y = 2*P1.y - P0.y;
			secondControlPoints.push_back(P2);

		}
		else {
			ROS_INFO("Polyline");

			// Calculate first Bezier control points
			// Right hand side vector
			vector<double> rhs;
	
			// Set right hand side X values
			rhs.push_back(knots.at(0).x + 2*knots.at(1).x);
			for (unsigned int i = 1; i < n-1; i++) {
				rhs.push_back(4*knots.at(i).x + 2*knots.at(i+1).x);
			}
			rhs.push_back((8*knots.at(n-2).x + knots.at(n-1).x)/2.0);
			
			
			// Get first control points X-values
			vector<double> x = planning::GetFirstControlPoints(rhs);
			
			rhs.clear();
			
			// Set right hand side Y values
			rhs.push_back(knots.at(0).y + 2*knots.at(1).y);
			for (unsigned int i = 1; i < n-1; i++) {
				rhs.push_back(4*knots.at(i).y + 2*knots.at(i+1).y);
			}
			rhs.push_back((8*knots.at(n-2).y + knots.at(n-1).y)/2.0);
			
			// Get first control points Y-values
			vector<double> y = planning::GetFirstControlPoints(rhs);
	
			// Fill output arrays.
			firstControlPoints.clear();
			secondControlPoints.clear();
			for (unsigned int i = 0; i < n; i++) {
				// First control point
				geometry_msgs::Point point;
				point.x = x.at(i);
				point.y = y.at(i);
				firstControlPoints.push_back(point);
				// Second control point
				if (i < n-1) {
					point.x = 2*knots.at(i+1).x - x.at(i+1);
					point.y = 2*knots.at(i+1).y - y.at(i+1);
					secondControlPoints.push_back(point);
				}
				else {
					point.x = (knots.at(n-1).x + x.at(n-1)) / 2;
					point.y = (knots.at(n-1).y + y.at(n-1)) / 2;
					secondControlPoints.push_back(point);
				}
			}
		}
	}

	vector<double> planning::GetFirstControlPoints(vector<double> rhs)
	{
		unsigned int n = rhs.size();
		vector<double> x;// Solution vector.
		vector<double> tmp; // Temp workspace.

		double b = 2.0;
		x.push_back(rhs.at(0) / b);
		tmp.push_back(0);
		for (unsigned int i = 1; i < n; i++) {// Decomposition and forward substitution.
			tmp.push_back(1 / b);
			b = (i < n - 1 ? 4.0 : 3.5) - tmp.at(i);
			x.push_back((rhs.at(i) - x.at(i-1)) / b);
		}
		for (unsigned int i = 1; i < n; i++) {
			x.at(n-i-1) -= tmp.at(n-i) * x.at(n-i); // Backsubstitution.
		}
		return x;
	}


geometry_msgs::Point planning::bezier_point(float u, point_arr ctrl_points) 
{
	geometry_msgs::Point point;
	point.x = pow(u,3)*(ctrl_points.at(3).x+3*(ctrl_points.at(1).x-ctrl_points.at(2).x)-ctrl_points.at(0).x)+3*pow(u,2)*(ctrl_points.at(0).x-2*ctrl_points.at(1).x+ctrl_points.at(2).x)+3*u*(ctrl_points.at(1).x-ctrl_points.at(0).x)+ctrl_points.at(0).x;
	point.y = pow(u,3)*(ctrl_points.at(3).y+3*(ctrl_points.at(1).y-ctrl_points.at(2).y)-ctrl_points.at(0).y)+3*pow(u,2)*(ctrl_points.at(0).y-2*ctrl_points.at(1).y+ctrl_points.at(2).y)+3*u*(ctrl_points.at(1).y-ctrl_points.at(0).y)+ctrl_points.at(0).y;
    return point;
}
