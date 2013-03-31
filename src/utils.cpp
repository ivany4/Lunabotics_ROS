#include "utils.h"

double normalize_angle(double angle)
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


double point_distance(point_t p1, point_t p2)
{
	return sqrt(pow(p2.x-p1.x, 2)+pow(p2.y-p1.y, 2));
}

