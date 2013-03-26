#include "../types.h"

namespace planning {
	void GetCurveControlPoints(point_arr knots, point_arr &firstControlPoints, point_arr &secondControlPoints);
	std::vector<double> GetFirstControlPoints(std::vector<double> rhs);
	geometry_msgs::Point bezier_point(float u, point_arr ctrl_points);
	
	point_arr quadratic_bezier(point_t q0, point_t q1, point_t q2, int segments);
	point_arr trajectory_bezier(point_t w0, point_t w1, point_t w2, point_t p);
}
