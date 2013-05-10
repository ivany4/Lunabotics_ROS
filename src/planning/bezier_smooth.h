#ifndef _BEZIER_SMOOTH_H_
#define _BEZIER_SMOOTH_H_


#include "../types.h"

namespace lunabotics {
	PointArr quadratic_bezier(Point q0, Point q1, Point q2, int segments);
	PointArr trajectory_bezier(Point w0, Point w1, Point w2, Point p, int segments);
	PointArr smoothen(PointArr trajectory);
}

#endif
