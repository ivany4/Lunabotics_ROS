//#include "common_msgs/Point.h"

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


