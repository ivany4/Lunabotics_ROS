#ifndef _BEZIER_CURVE_H_
#define _BEZIER_CURVE_H_

#include "../types.h"

namespace lunabotics {

class BezierCurve {
private:
	Point _p0;
	Point _p1;
	Point _p2;
	PointArr _cached_points;
	double _cached_max_curvature;
	int _num_segments;

public:
	BezierCurve(Point p0, Point p1, Point p2, int num_segments);
	~BezierCurve();
	float maxCurvature();
	PointArr getPoints();
};

typedef BezierCurve *BezierCurvePtr;

BezierCurvePtr CreateConstrainedBezierCurve(Point q0, Point q1, Point q2, Point p, int num_segments);

}


#endif
