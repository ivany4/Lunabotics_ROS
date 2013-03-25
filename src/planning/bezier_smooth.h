#include "../types.h"

namespace planning {
	void GetCurveControlPoints(point_arr knots, point_arr &firstControlPoints, point_arr &secondControlPoints);
	std::vector<double> GetFirstControlPoints(std::vector<double> rhs);
	geometry_msgs::Point bezier_point(float u, point_arr ctrl_points);
	
	std::vector<geometry_msgs::Point> quadratic_bezier(geometry_msgs::Point q0, geometry_msgs::Point q1, geometry_msgs::Point q2, float step);
	std::vector<geometry_msgs::Point> trajectory_bezier(geometry_msgs::Point w0, geometry_msgs::Point w1, geometry_msgs::Point w2,  geometry_msgs::Point p);
}
