#include "../types.h"

namespace planning {
	void GetCurveControlPoints(point_arr knots, point_arr &firstControlPoints, point_arr &secondControlPoints);
	std::vector<double> GetFirstControlPoints(std::vector<double> rhs);
	geometry_msgs::Point bezier_point(float u, point_arr ctrl_points);
}
