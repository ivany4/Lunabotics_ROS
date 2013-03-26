#include "../types.h"

namespace planning {
	point_arr quadratic_bezier(point_t q0, point_t q1, point_t q2, int segments);
	point_arr trajectory_bezier(point_t w0, point_t w1, point_t w2, point_t p);
}
